#include "AppLayer.h"
#include "LinkLayer.h"
#include "CharBuffer.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

//#define AL_PRINT_CPACKETS

#define MAX_FRAGMENT_SIZE 0xFFFF
#define DATA_HEADER_SIZE 4
#define MAX_FILE_NAME 256

#define CP_CFIELD 0x00
#define SEQ_FIELD 0x01
#define L2_FIELD 0x02
#define L1_FIELD 0x03

#define TLV_SIZE_T 0x00
#define TLV_NAME_T 0x01
#define CP_MIN_SIZE 7

#define AL_PRINT_ERROR(msg) fprintf(stderr, "alerror: %s\n", msg)

typedef unsigned char uchar;
typedef enum {
  CONTROL_START = 0x02,
  CONTROL_END = 0x03,
  CONTROL_DATA = 0x01
} ControlType;

typedef struct {
  ControlType type;
  char *name;
  int8_t nameLength;
  uint32_t size;
  uint8_t sizeLength;
} ControlPacket;

typedef struct {
  ControlType type;
  int8_t sequenceNr;
  uint16_t size;
  char *data;
} DataPacket;

ControlPacket fileCP;  // Control Packet with file information

void printControlPacket(ControlPacket *packet);
int readControlPacket(int fd, ControlPacket *packet);
int parseControlPacket(char *packetBuffer, int size, ControlPacket *cp);
int sendControlPacket(int fd, ControlType type);
void buildControlPacket(ControlType type, CharBuffer *packet);
int getFileInfo(const char *filename, FILE *fptr);
int readDataPacket(int fd, DataPacket *packet, char *buffer);
int sendDataPacket(int fd, DataPacket *packet);
void printProgress(int done, int total);

int sendFile(const char *filename) {
  int nameLength = strlen(filename);
  if (nameLength > MAX_FILE_NAME) {
    AL_PRINT_ERROR("Filename length exceeds limits(256 characters)");
    return -1;
  }
  fileCP.nameLength = nameLength;

  // Open File Stream
  FILE *fptr = fopen(filename, "r");
  if (fptr == NULL) {
    AL_PRINT_ERROR("Could not open selected file");
    return -1;
  }

  // Establish LL Connection
  int fd = llopen(11, TRANSMITTER);
  if (fd == -1) {
    AL_PRINT_ERROR("Failed to establish connection");
    return -1;
  }

  // Get File Information
  getFileInfo(filename, fptr);

  // Send start control packet
  if (sendControlPacket(fd, CONTROL_START) == -1) return -1;
  printf("al: starting file transmission\n");

  // Send data packets until the file is read
  DataPacket packet;
  packet.data = (char *)malloc(MAX_FRAGMENT_SIZE + DATA_HEADER_SIZE);
  packet.sequenceNr = 0;
  packet.size = 1;
  unsigned int bytesTransferred = 0;
  while (true) {
    packet.size = fread(&packet.data[L1_FIELD + 1], sizeof(uchar),
                        MAX_FRAGMENT_SIZE, fptr);
    if (packet.size <= 0) {
      break;
    }
    ++packet.sequenceNr;
    packet.sequenceNr %= 256;
    sendDataPacket(fd, &packet);

    // Progress
    bytesTransferred += packet.size;
    printProgress(bytesTransferred, fileCP.size);
  }
  printf("\n");
  free(packet.data);

  // Send end control packet
  if (sendControlPacket(fd, CONTROL_END) == -1) return -1;
  printf("al: file transmission is over\n");

  // Close connection and cleanup
  llclose(fd);
  free(fileCP.name);
  fclose(fptr);
  return 0;
}

int receiveFile(const char *filename) {
  // Establish LL Connection
  int fd = llopen(10, RECEIVER);
  if (fd == -1) {
    AL_PRINT_ERROR("Failed to establish connection");
    return -1;
  }
  printf("al: waiting for connection\n");
  // Wait for start control packet

  FILE *fptr = fopen(filename, "w");
  if (fptr == NULL) {
    AL_PRINT_ERROR("Could not write selected file");
    return -1;
  }

  fileCP.type = CONTROL_DATA;
  while (fileCP.type != CONTROL_START) {
    readControlPacket(fd, &fileCP);
  }
  printf("al: starting file transmission\n");

  unsigned int bytesTransferred = 0;
  DataPacket dataPacket;
  while (bytesTransferred < fileCP.size) {
    char *buffer = NULL;
    if (readDataPacket(fd, &dataPacket, buffer) == -1) return -1;

    fwrite(dataPacket.data, sizeof(char), dataPacket.size, fptr);

    bytesTransferred += dataPacket.size;
    free(buffer);
    printProgress(bytesTransferred, fileCP.size);
  }
  printf("\n");
  // Wait for end control packet
  ControlPacket packet;
  packet.type = CONTROL_DATA;
  while (packet.type != CONTROL_END) {
    readControlPacket(fd, &packet);
  }
  printf("al: file transmission is over\n");
  // Close connection and cleanup
  free(fileCP.name);
  llclose(fd);
  return 0;
}

int readDataPacket(int fd, DataPacket *packet, char *buffer) {
  int res = llread(fd, &buffer);
  if (res < 0) {
    AL_PRINT_ERROR("failed to read packet");
    return -1;
  }

  if (packet == NULL || buffer[CP_CFIELD] != CONTROL_DATA) {
    AL_PRINT_ERROR("unexpected control packet, aborting...");
    return -1;
  }

  packet->sequenceNr = buffer[SEQ_FIELD];
  packet->size = (uchar)buffer[L2_FIELD] * 256;
  packet->size += (uchar)buffer[L1_FIELD];
  packet->data = &buffer[L1_FIELD + 1];
  return 0;
}

int sendDataPacket(int fd, DataPacket *packet) {
  packet->data[CP_CFIELD] = CONTROL_DATA;
  packet->data[SEQ_FIELD] = (uchar)packet->sequenceNr;
  packet->data[L2_FIELD] = (uchar)(packet->size / 256);
  packet->data[L1_FIELD] = (uchar)(packet->size % 256);

  int res = llwrite(fd, (char *)packet->data, packet->size + DATA_HEADER_SIZE);

  if (res >= packet->size) return 0;
  return -1;
}

int sendControlPacket(int fd, ControlType type) {
  CharBuffer buffer;
  buildControlPacket(type, &buffer);
  printf("al: sent control Packet ");
  printControlPacket(&fileCP);
  if (llwrite(fd, buffer.buffer, buffer.size) == -1) {
    AL_PRINT_ERROR("Failed to send packet, aborting..");
    return -1;
  }
  CharBuffer_destroy(&buffer);
  return 0;
}

int readControlPacket(int fd, ControlPacket *packet) {
  char *buffer;
  int size = llread(fd, &buffer);
  if (size == -1) {
    free(buffer);
    AL_PRINT_ERROR("Failed to receive packet, aborting..");
    return -1;
  }
  parseControlPacket(buffer, size, packet);
#ifdef AL_PRINT_CPACKETS
  printf("al: received control Packet ");
  printControlPacket(packet);
#endif
  free(buffer);
  return 0;
}

void buildControlPacket(ControlType type, CharBuffer *packet) {
  /**
   * [C][T Size][L Size][  V Size  ][T Name][L Name][  V Name  ]
   * C - Control Field  T - Type  L - Length  V - Value
   *
   * Size: (C + 2*(T+L) = 5) + length of size + length of name
   */
  fileCP.type = type;
  int packetSize = 5 + fileCP.sizeLength + fileCP.nameLength;
  CharBuffer_init(packet, packetSize);
  CharBuffer_push(packet, (char)type);
  // SIZE TLV
  CharBuffer_push(packet, (char)TLV_SIZE_T);
  CharBuffer_push(packet, (char)fileCP.sizeLength);
  unsigned int size = fileCP.size;
  for (uint8_t i = 0; i < fileCP.sizeLength; ++i) {
    CharBuffer_push(packet, (char)size & 0x000000FF);
    size >>= 8;
  }
  // NAME TLV
  CharBuffer_push(packet, (char)TLV_NAME_T);
  CharBuffer_push(packet, (char)fileCP.nameLength);
  for (int i = 0; i < fileCP.nameLength; ++i) {
    CharBuffer_push(packet, fileCP.name[i]);
  }
}

void printControlPacket(ControlPacket *packet) {
  if (packet == NULL) return;

  printf("[C %d][T 0][L %d][V %d][T 1][V %d][V %s]\n", packet->type,
         packet->sizeLength, packet->size, packet->nameLength, packet->name);
}

int parseControlPacket(char *packetBuffer, int size, ControlPacket *cp) {
  if (packetBuffer == NULL) return -1;
  if (size < CP_MIN_SIZE) return -1;

  int index = 0;
  cp->type = packetBuffer[index++];

  if (packetBuffer[index++] != TLV_SIZE_T) {
    AL_PRINT_ERROR("Invalid control packet");
    return -1;
  }

  cp->sizeLength = (int8_t)packetBuffer[index++];

  cp->size = 0;
  for (int8_t i = 0; i < cp->sizeLength; ++i) {
    if (index > size - 1) {
      AL_PRINT_ERROR("Invalid Control Packet");
      return -1;
    }
    cp->size |= ((uchar)packetBuffer[index++]) << (8 * i);
  }

  if ((index > (size - 1)) || (packetBuffer[index++] != TLV_NAME_T)) {
    AL_PRINT_ERROR("Invalid control packet");
    return -1;
  }

  if (index > (size - 1)) {
    AL_PRINT_ERROR("Invalid control packet");
    return -1;
  }
  cp->nameLength = (int8_t)packetBuffer[index++];
  cp->name = (char *)malloc(cp->nameLength * sizeof(char) + 1);
  int namePos = index;

  for (int i = 0; index < (namePos + cp->nameLength); ++index) {
    if (index > size) {
      AL_PRINT_ERROR("Invalid control packet");
      return -1;
    }
    cp->name[i++] = packetBuffer[index];
  }
  cp->name[cp->nameLength] = 0x00;  // Terminate string
  return 0;
}

int getFileInfo(const char *filename, FILE *fptr) {
  if (fptr == NULL) return -1;

  // Name
  fileCP.name = (char *)malloc(sizeof(char) * fileCP.nameLength + 1);
  strcpy(fileCP.name, filename);

  // Size
  fseek(fptr, 0L, SEEK_END);
  fileCP.size = ftell(fptr);
  rewind(fptr);

  // Bytes needed for length
  fileCP.sizeLength = 1;
  int size = fileCP.size;
  for (unsigned int i = 1; i < sizeof(int); ++i) {
    size >>= 8;
    if (size > 0)
      ++fileCP.sizeLength;
    else
      break;
  }

  return 0;
}

void printProgress(int done, int total) {
  float percent = ((float)done / (float)total) * 100.0f;
  int blocks = percent / 10;

  printf("\rProgress ");
  for (int i = 0; i < blocks; ++i) {
    printf("#");
  }
  printf("[%.2f]", percent);
  fflush(stdout);
}
