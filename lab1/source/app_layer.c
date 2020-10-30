#include "app_layer.h"
#include "link_layer.h"
#include "char_buffer.h"

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>

//#define AL_PRINT_CPACKETS

#define MAX_FRAGMENT_SIZE 0xFFFF
#define MAX_BAUDRATE 460800
#define DATA_HEADER_SIZE 4
#define MAX_FILE_NAME 256

#define CP_CFIELD 0x00
#define SEQ_FIELD 0x01
#define L2_FIELD 0x02
#define L1_FIELD 0x03

#define TLV_SIZE_T 0x00
#define TLV_NAME_T 0x01
#define CP_MIN_SIZE 7

#define AL_LOG_INFORMATION

typedef unsigned char uchar;
typedef enum {
  CONTROL_START = 0x02,
  CONTROL_END = 0x03,
  CONTROL_DATA = 0x01
} control_type;

typedef struct {
  control_type type;
  char *name;
  int8_t nameLength;
  uint32_t size;
  uint8_t sizeLength;
} control_packet;

typedef struct {
  control_type type;
  int8_t sequenceNr;
  uint16_t size;
  char *data;
} data_packet;

control_packet fileCP;  // Control Packet with file information
static al_statistics al_stats;
static int al_frag_size = MAX_FRAGMENT_SIZE;

void print_control_packet(control_packet *packet);
int read_control_packet(int fd, control_packet *packet);
int parse_control_packet(char *packetBuffer, int size, control_packet *cp);
int send_control_packet(int fd, control_type type);
void build_control_packet(control_type type, char_buffer *packet);
int get_file_info(const char *filename, FILE *fptr);
int read_data_packet(int fd, data_packet *packet, char *buffer);
int send_data_packet(int fd, data_packet *packet);
void print_progress(int done, int total);

void al_log_msg(const char *msg) {
#ifdef AL_LOG_INFORMATION
  fprintf(stderr, "al: %s\n", msg);
#endif
}

double clock_seconds_since(clock_t start) {
  return ((double)(clock() - start) / (double)CLOCKS_PER_SEC);
}

void update_statistics(clock_t start_timer) {
  al_stats.file_size = fileCP.size;
  al_stats.transmission_duration_secs = clock_seconds_since(start_timer);
  ll_statistics ll_stats = ll_get_stats();
  al_stats.frames_total = ll_stats.frames_total;
  al_stats.frames_lost = ll_stats.frames_lost;
  al_stats.avg_bits_per_second =
      (double)(al_stats.file_size * 8) / al_stats.transmission_duration_secs;
}

al_statistics al_get_stats() { return al_stats; }

void al_print_stats() {
  printf("Statistics:\n");
  float eff = (float)al_stats.baudrate / (float)al_stats.avg_bits_per_second;
  printf(" baudrate %d bits/s \taverage bitrate %d bits/s \tEfficiency %.2f \n",
         al_stats.baudrate, al_stats.avg_bits_per_second, eff);
  printf(" file size %d bytes \tmax fragment size %d bytes \tpackets sent %d\n",
         al_stats.file_size, al_frag_size, al_stats.data_packet_count);
  printf(" transmission time %.2f seconds\n",
         al_stats.transmission_duration_secs);
  float floss = (float)al_stats.frames_lost / (float)al_stats.frames_total;
  printf(" total Frames %d \tlost frames %d \tframe loss %.2f\n",
         al_stats.frames_total, al_stats.frames_lost, floss);
}

void al_setup(int timeout, int baudrate, int max_retries, int frag_size) {
  al_stats.timeout = timeout;
  al_stats.retries = max_retries;
  al_stats.data_packet_count = 0;
  if (baudrate > MAX_BAUDRATE) baudrate = MAX_BAUDRATE;
  al_stats.baudrate = baudrate;
  al_frag_size = frag_size;
  if (al_frag_size > MAX_FRAGMENT_SIZE) al_frag_size = MAX_FRAGMENT_SIZE;
  ll_setup(timeout, max_retries, baudrate);
}

int al_sendFile(const char *filename, int port) {
  int nameLength = strlen(filename);
  if (nameLength > MAX_FILE_NAME) {
    al_log_msg("Filename length exceeds limits(256 characters)");
    return -1;
  }
  fileCP.nameLength = nameLength;

  // Open File Stream
  FILE *fptr = fopen(filename, "r");
  if (fptr == NULL) {
    al_log_msg("Could not open selected file");
    return -1;
  }

  // Establish LL Connection
  int fd = llopen(port, TRANSMITTER);
  if (fd == -1) {
    al_log_msg("Failed to establish connection");
    return -1;
  }

  // Get File Information
  get_file_info(filename, fptr);

  clock_t start_timer = clock();

  // Send start control packet
  if (send_control_packet(fd, CONTROL_START) == -1) return -1;
  printf("al: starting file transmission\n");

  // Send data packets until the file is read
  data_packet packet;
  packet.data = (char *)malloc(al_frag_size + DATA_HEADER_SIZE);
  packet.sequenceNr = 0;
  packet.size = 1;
  unsigned int bytesTransferred = 0;
  while (true) {
    packet.size =
        fread(&packet.data[L1_FIELD + 1], sizeof(uchar), al_frag_size, fptr);
    if (packet.size <= 0) {
      break;
    }
    ++al_stats.data_packet_count;
    ++packet.sequenceNr;
    packet.sequenceNr %= 256;
    send_data_packet(fd, &packet);

    // Progress
    bytesTransferred += packet.size;
    print_progress(bytesTransferred, fileCP.size);
  }
  printf("\n");
  free(packet.data);

  // Send end control packet
  if (send_control_packet(fd, CONTROL_END) == -1) return -1;
  printf("al: file transmission is over\n");

  update_statistics(start_timer);

  // Close connection and cleanup
  llclose(fd);
  free(fileCP.name);
  fclose(fptr);
  return 0;
}

int al_receiveFile(const char *filename, int port) {
  printf("al: waiting for connection\n");

  static int8_t seq_number = 1;

  // Establish LL Connection
  int fd = llopen(port, RECEIVER);
  if (fd == -1) {
    al_log_msg("Failed to establish connection");
    return -1;
  }

  FILE *fptr = fopen(filename, "w");
  if (fptr == NULL) {
    al_log_msg("Could not write selected file");
    return -1;
  }
  clock_t start_timer = clock();

  fileCP.type = CONTROL_DATA;
  while (fileCP.type != CONTROL_START) {
    read_control_packet(fd, &fileCP);
  }
  printf("al: starting file transmission\n");

  unsigned int bytesTransferred = 0;
  data_packet data_packet;
  while (bytesTransferred < fileCP.size) {
    char *buffer = NULL;
    if (read_data_packet(fd, &data_packet, buffer) == -1) return -1;

    if (data_packet.sequenceNr != seq_number) {
      al_log_msg("ignoring duplicate data packet");
      free(buffer);
      continue;
    }
    ++seq_number;
    seq_number %= 256;

    fwrite(data_packet.data, sizeof(char), data_packet.size, fptr);

    bytesTransferred += data_packet.size;
    free(buffer);
    print_progress(bytesTransferred, fileCP.size);
  }
  printf("\n");
  // Wait for end control packet
  control_packet packet;
  packet.type = CONTROL_DATA;
  while (packet.type != CONTROL_END) {
    read_control_packet(fd, &packet);
  }
  update_statistics(start_timer);
  printf("al: file transmission is over\n");
  // Close connection and cleanup
  free(fileCP.name);
  llclose(fd);
  return 0;
}

int read_data_packet(int fd, data_packet *packet, char *buffer) {
  int res = llread(fd, &buffer);
  if (res < 0) {
    al_log_msg("failed to read packet");
    return -1;
  }

  if (packet == NULL || buffer[CP_CFIELD] != CONTROL_DATA) {
    al_log_msg("unexpected control packet, aborting...");
    return -1;
  }

  packet->sequenceNr = buffer[SEQ_FIELD];
  packet->size = (uchar)buffer[L2_FIELD] * 256;
  packet->size += (uchar)buffer[L1_FIELD];
  packet->data = &buffer[L1_FIELD + 1];
  return 0;
}

int send_data_packet(int fd, data_packet *packet) {
  packet->data[CP_CFIELD] = CONTROL_DATA;
  packet->data[SEQ_FIELD] = (uchar)packet->sequenceNr;
  packet->data[L2_FIELD] = (uchar)(packet->size / 256);
  packet->data[L1_FIELD] = (uchar)(packet->size % 256);

  int res = llwrite(fd, (char *)packet->data, packet->size + DATA_HEADER_SIZE);

  if (res >= packet->size) return 0;
  return -1;
}

int send_control_packet(int fd, control_type type) {
  char_buffer buffer;
  build_control_packet(type, &buffer);
  printf("al: sent control Packet ");
  print_control_packet(&fileCP);
  if (llwrite(fd, buffer.buffer, buffer.size) == -1) {
    al_log_msg("Failed to send packet, aborting..");
    return -1;
  }
  char_buffer_destroy(&buffer);
  return 0;
}

int read_control_packet(int fd, control_packet *packet) {
  char *buffer;
  int size = llread(fd, &buffer);
  if (size == -1) {
    free(buffer);
    al_log_msg("Failed to receive packet, aborting..");
    return -1;
  }
  parse_control_packet(buffer, size, packet);
#ifdef AL_PRINT_CPACKETS
  printf("al: received control Packet ");
  print_control_packet(packet);
#endif
  free(buffer);
  return 0;
}

void build_control_packet(control_type type, char_buffer *packet) {
  /**
   * [C][T Size][L Size][  V Size  ][T Name][L Name][  V Name  ]
   * C - Control Field  T - Type  L - Length  V - Value
   *
   * Size: (C + 2*(T+L) = 5) + length of size + length of name
   */
  fileCP.type = type;
  int packetSize = 5 + fileCP.sizeLength + fileCP.nameLength;
  char_buffer_init(packet, packetSize);
  char_buffer_push(packet, (char)type);
  // SIZE TLV
  char_buffer_push(packet, (char)TLV_SIZE_T);
  char_buffer_push(packet, (char)fileCP.sizeLength);
  unsigned int size = fileCP.size;
  for (uint8_t i = 0; i < fileCP.sizeLength; ++i) {
    char_buffer_push(packet, (char)size & 0x000000FF);
    size >>= 8;
  }
  // NAME TLV
  char_buffer_push(packet, (char)TLV_NAME_T);
  char_buffer_push(packet, (char)fileCP.nameLength);
  for (int i = 0; i < fileCP.nameLength; ++i) {
    char_buffer_push(packet, fileCP.name[i]);
  }
}

void print_control_packet(control_packet *packet) {
  if (packet == NULL) return;

  printf("[C %d][T 0][L %d][V %d][T 1][V %d][V %s]\n", packet->type,
         packet->sizeLength, packet->size, packet->nameLength, packet->name);
}

int parse_control_packet(char *packetBuffer, int size, control_packet *cp) {
  if (packetBuffer == NULL) return -1;
  if (size < CP_MIN_SIZE) return -1;

  int index = 0;
  cp->type = packetBuffer[index++];

  if (packetBuffer[index++] != TLV_SIZE_T) {
    al_log_msg("Invalid control packet");
    return -1;
  }

  cp->sizeLength = (int8_t)packetBuffer[index++];

  cp->size = 0;
  for (int8_t i = 0; i < cp->sizeLength; ++i) {
    if (index > size - 1) {
      al_log_msg("Invalid Control Packet");
      return -1;
    }
    cp->size |= ((uchar)packetBuffer[index++]) << (8 * i);
  }

  if ((index > (size - 1)) || (packetBuffer[index++] != TLV_NAME_T)) {
    al_log_msg("Invalid control packet");
    return -1;
  }

  if (index > (size - 1)) {
    al_log_msg("Invalid control packet");
    return -1;
  }
  cp->nameLength = (int8_t)packetBuffer[index++];
  cp->name = (char *)malloc(cp->nameLength * sizeof(char) + 1);
  int namePos = index;

  for (int i = 0; index < (namePos + cp->nameLength); ++index) {
    if (index > size) {
      al_log_msg("Invalid control packet");
      return -1;
    }
    cp->name[i++] = packetBuffer[index];
  }
  cp->name[cp->nameLength] = 0x00;  // Terminate string
  return 0;
}

int get_file_info(const char *filename, FILE *fptr) {
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

void print_progress(int done, int total) {
  float percent = ((float)done / (float)total) * 100.0f;
  int blocks = percent / 10;

  printf("\rProgress ");
  for (int i = 0; i < blocks; ++i) {
    printf("#");
  }
  printf("[%.2f]", percent);
  fflush(stdout);
}
