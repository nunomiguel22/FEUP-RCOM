/*Non-Canonical Input Processing*/
#include "LinkLayer.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include "CharBuffer.h"

/* POSIX compliant source */
#define _POSIX_SOURCE 1

/* Framing */
#define CONTROL_FRAME_SIZE 5
#define FLAG 0x7E  // Flag for beggining and ending of frame
#define ESC 0x7D   // Escape character for byte stuffing
#define AF1 0x03   // Transmitter commands, Receiver replys
#define AF2 0x01   // Transmitter replys, Receiver commands

/* Serial Port */
#define DEFAULT_BAUDRATE B38400
#define MAX_PORT_LENGTH 20

/*  Port name prefix */
#ifdef __linux__
#define PORT_NAME "/dev/ttyS"
#elif _WIN32
#define PORT_NAME "COM"
#else
#define PORT_NAME "/dev/ttyS"
#endif

/* Types */
typedef struct {
  char port[MAX_PORT_LENGTH];
  int baudRate;
  unsigned int sequenceNumber;
  unsigned int timeout;
  unsigned int numTransmissions;
} LinkLayer;

typedef enum {
  INF = 0x00,
  SET = 0x03,
  DISC = 0x0B,
  UA = 0x07,
  RR = 0x05,
  REJ = 0x01
} ControlTypes;

typedef enum {
  FSFIELD,    // Start Flag
  AFIELD,     // Address
  CFIELD,     // Control
  BCC1FIELD,  // BCC1
  DFIELD,     // Data
  BCC2FIELD,  // BCC2
  FEFIELD     // End Flag
} FrameField;

/* Globals */
static LinkLayer ll;
static LinkType linkType;

/* Auxiliar Functions Declarations */
int initSerialPort(int port, LinkType type);
bool isControlCommand(ControlTypes type);
void createControlFrame(char* frame, ControlTypes controlType);
int sendControlFrame(int fd, ControlTypes controlType);
int readFrame(int fd, CharBuffer* charbuffer);

/* Link Layer Functions */
int llopen(int port, LinkType type) {
  int fd = initSerialPort(port, type);
  if (fd == -1) return fd;

  if (linkType == TRANSMITTER) {
    sendControlFrame(fd, SET);
  } else if (linkType == RECEIVER) {
    CharBuffer buffer;
    readFrame(fd, &buffer);
    sendControlFrame(fd, UA);
  }

  // TODO: SEND AND READ CONTROL FRAME DEPENDING ON TRANS/RECEIVER

  return fd;
}

int llclose(int fd);
int llwrite(int fd, char* buffer, int length);
int llread(int fd, char* buffer);

/* Auxiliar Functions */
int readFrame(int fd, CharBuffer* charbuffer) {
  if (charbuffer == NULL) return -1;
  CharBuffer_init(charbuffer, CONTROL_FRAME_SIZE);

  char incByte = 0x00;
  int readStatus = 0;
  // Clear buffer and wait for a flag
  while (readStatus == 0 || incByte != FLAG)  // TO-DO Implement timeout
    readStatus = read(fd, &incByte, 1);
  CharBuffer_push(charbuffer, incByte);
  // Reset vars
  readStatus = 0;
  incByte = 0x00;
  // Read serial until flag is found
  while (incByte != FLAG) {
    readStatus = read(fd, &incByte, 1);
    if (readStatus == 0) continue;

    CharBuffer_push(charbuffer, incByte);
    CharBuffer_printHex(charbuffer);
  }

  // TO-DO Validate Frame

  return 0;
}

int sendControlFrame(int fd, ControlTypes controlType) {
  char frame[CONTROL_FRAME_SIZE];
  createControlFrame(frame, controlType);
  return write(fd, frame, CONTROL_FRAME_SIZE);
}

void createControlFrame(char* frame, ControlTypes controlType) {
  // Frame start flag
  frame[0] = FLAG;

  // Address Field
  frame[1] = AF1;
  if (linkType == RECEIVER && isControlCommand(controlType))
    frame[1] = AF2;
  else if (linkType == TRANSMITTER && !isControlCommand(controlType))
    frame[1] = AF2;

  // Control Field
  frame[2] = controlType;
  // BCC Field (Address Field ^ Control Field)
  frame[3] = frame[1] ^ frame[2];
  // Frame end flag
  frame[4] = FLAG;
}

bool isControlCommand(ControlTypes type) {
  if (type == INF || type == DISC || type == SET) return true;
  return false;
}

int initSerialPort(int port, LinkType type) {
  // Init ll struct
  snprintf(ll.port, MAX_PORT_LENGTH, "%s%d", PORT_NAME, port);
  ll.baudRate = DEFAULT_BAUDRATE;
  linkType = type;

  int fd = open(ll.port, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror(ll.port);
    exit(-1);
  }

  struct termios oldtio, newtio;

  if (tcgetattr(fd, &oldtio) == -1) { /* save current port settings */
    perror("tcgetattr");
    exit(-1);
  }

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = ll.baudRate | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
  newtio.c_cc[VMIN] = 5;  /* blocking read until 5 chars received */

  /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) pr�ximo(s) caracter(es)
  */

  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  return fd;
}
