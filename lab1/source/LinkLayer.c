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
#include <signal.h>
#include "CharBuffer.h"

// DEBUG
#define DEBUG_PRINT_CONTROL_FRAMES
//#define DEBUG_PRINT_FRAMES

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
#define MAX_TRANSMISSION_ATTEMPS 3
#define TIMEOUT_DURATION 5

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
  FSFIELD = 0x00,    // Start Flag
  AFIELD = 0x01,     // Address
  CFIELD = 0x02,     // Control
  BCC1FIELD = 0x03,  // BCC1
  FEFIELD = 0x04     // End Flag
} ControlFrameField;

/* Globals */
static LinkLayer ll;
static LinkType linkType;
volatile bool alarmTriggered;
volatile unsigned int transmissionAttemps = 0;

/* Alarm */
void alarmHandler(int signum) {
  if (signum == SIGALRM) {
    ++transmissionAttemps;
    alarmTriggered = true;
    printf("Connection timed out, retrying...\n");
  }
}
void setAlarm(unsigned int seconds) {
  alarmTriggered = false;
  alarm(seconds);
}

void setAlarmHandler() {
  signal(SIGALRM, alarmHandler);
  alarmTriggered = false;
  transmissionAttemps = 0;
}

void resetAlarmHandler() {
  alarm(0);
  signal(SIGALRM, NULL);
  alarmTriggered = false;
  transmissionAttemps = 0;
}

/* Auxiliar Functions Declarations */
int initSerialPort(int port, LinkType type);
bool isControlCommand(ControlTypes type);
void createControlFrame(char* frame, ControlTypes controlType);
int exchangeControlFrame(int fd, ControlTypes send, ControlFrameField receive);
int readControlFrame(int fd, ControlTypes controlType);
int sendControlFrame(int fd, ControlTypes controlType);
int readFrame(int fd, CharBuffer* charbuffer);
int validateControlFrame(CharBuffer* charbuffer, ControlTypes type);
char getAddressField(LinkType lnk, ControlTypes type);
void printControlType(ControlTypes type);

/* Link Layer Functions */
int llopen(int port, LinkType type) {
  int fd = initSerialPort(port, type);
  if (fd == -1) return fd;

  if (linkType == TRANSMITTER) {
    int ret = exchangeControlFrame(fd, SET, UA);
    if (ret == -1) printf("LLOPEN failed\n");
  }
  if (linkType == RECEIVER) {
    if (readControlFrame(fd, SET) == -1) {
      printf("llopen timed out\n");
      return -1;
    } else {
      if (sendControlFrame(fd, UA) == -1)
        printf("llopen failed to send UA Packet\n");
      return -1;
    }
  }
  return fd;
}

int llclose(int fd) {
  if (linkType == TRANSMITTER) {
    int ret = exchangeControlFrame(fd, DISC, DISC);
    if (ret == -1) {
      printf("LLCLOSE failed\n");
      return -1;
    }
    ret = sendControlFrame(fd, UA);
    return 0;
  }
  if (linkType == RECEIVER) {
    if (readControlFrame(fd, DISC) == -1) {
      printf("LLCLOSE FAILED\n");
      return -1;
    }
    int ret = exchangeControlFrame(fd, DISC, UA);
    if (ret == -1) {
      printf("LLCLOSE failed\n");
      return -1;
    }
  }
  return 0;
}
int llwrite(int fd, char* buffer, int length);
int llread(int fd, char* buffer);

/* Auxiliar Functions */
int validateControlFrame(CharBuffer* charbuffer, ControlTypes type) {
  if (charbuffer == NULL) return -1;
  if (charbuffer->size != CONTROL_FRAME_SIZE) return -1;

  char expectedAF = getAddressField(linkType ^ 1, type);
  if (charbuffer->buffer[FSFIELD] != FLAG) return -1;
  if (charbuffer->buffer[AFIELD] != expectedAF) return -1;
  if (charbuffer->buffer[CFIELD] != (char)type) return -1;
  if (charbuffer->buffer[BCC1FIELD] != (char)(expectedAF ^ type)) return -1;
  if (charbuffer->buffer[FEFIELD] != (char)FLAG) return -1;

  return 0;
}

int exchangeControlFrame(int fd, ControlTypes send, ControlFrameField receive) {
  setAlarmHandler();
  while (transmissionAttemps < ll.numTransmissions) {
    int ret = sendControlFrame(fd, send);
    if (ret == -1) {
      printf("Failed to send frame");
      return -1;
    }
    setAlarm(ll.timeout);
    ret = readControlFrame(fd, receive);
    if (ret == 0) {
      resetAlarmHandler();
      return 0;
    }
  }
  resetAlarmHandler();
  printf("Connection timed out\n");
  return -1;
}

int readControlFrame(int fd, ControlTypes controlType) {
  CharBuffer charbuffer;
  if (readFrame(fd, &charbuffer) == -1) {
    CharBuffer_destroy(&charbuffer);
    return -1;
  }
  if (validateControlFrame(&charbuffer, controlType) == -1) return -1;
  CharBuffer_destroy(&charbuffer);

#ifdef DEBUG_PRINT_CONTROL_FRAMES
  printf("Received control packet ");
  printControlType(controlType);
  printf("\n");
#endif

  return 0;
}

int readFrame(int fd, CharBuffer* charbuffer) {
  if (charbuffer == NULL) return -1;
  CharBuffer_init(charbuffer, CONTROL_FRAME_SIZE);

  char incByte = 0x00;
  int readStatus = 0;
  // Clear buffer and wait for a flag
  while (readStatus <= 0 || incByte != FLAG) {  // TO-DO Implement timeout
    if (alarmTriggered) return -1;
    readStatus = read(fd, &incByte, 1);
  }
  CharBuffer_push(charbuffer, incByte);
  // Reset vars
  readStatus = 0;
  incByte = 0x00;
  // Read serial until flag is found
  while (incByte != FLAG) {
    readStatus = read(fd, &incByte, 1);
    if (alarmTriggered) return -1;
    if (readStatus <= 0) continue;

    CharBuffer_push(charbuffer, incByte);
  }

#ifdef DEBUG_PRINT_FRAMES
  CharBuffer_printHex(charbuffer);
#endif

  return 0;
}

int sendControlFrame(int fd, ControlTypes controlType) {
  char frame[CONTROL_FRAME_SIZE];
  createControlFrame(frame, controlType);
  if (write(fd, frame, CONTROL_FRAME_SIZE) == -1) return -1;

#ifdef DEBUG_PRINT_CONTROL_FRAMES
  printf("Sent control packet ");
  printControlType(controlType);
  printf("\n");
#endif

  return 0;
}

void createControlFrame(char* frame, ControlTypes controlType) {
  frame[0] = FLAG;
  frame[1] = getAddressField(linkType, controlType);
  frame[2] = controlType;
  frame[3] = frame[1] ^ frame[2];  // BCC Field (Address Field ^ Control Field)
  frame[4] = FLAG;
}

char getAddressField(LinkType lnk, ControlTypes type) {
  if (lnk == RECEIVER && isControlCommand(type))
    return AF2;
  else if (lnk == TRANSMITTER && !isControlCommand(type))
    return AF2;
  return AF1;
}

bool isControlCommand(ControlTypes type) {
  if (type == INF || type == DISC || type == SET) return true;
  return false;
}

void printControlType(ControlTypes type) {
  switch (type) {
    case INF: {
      printf("INF");
      break;
    }
    case SET: {
      printf("SET");
      break;
    }
    case DISC: {
      printf("DISC");
      break;
    }
    case UA: {
      printf("UA");
      break;
    }
    case RR: {
      printf("RR");
      break;
    }
    case REJ: {
      printf("REJ");
      break;
    }
    default:
      break;
  }
}

int initSerialPort(int port, LinkType type) {
  // Init ll struct
  snprintf(ll.port, MAX_PORT_LENGTH, "%s%d", PORT_NAME, port);
  ll.baudRate = DEFAULT_BAUDRATE;
  ll.numTransmissions = MAX_TRANSMISSION_ATTEMPS;
  ll.timeout = TIMEOUT_DURATION;
  linkType = type;

  int fd = open(ll.port, O_RDWR | O_NOCTTY | O_NONBLOCK);
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

  newtio.c_cc[VTIME] = ll.timeout * 10; /* inter-character timer unused */
  newtio.c_cc[VMIN] = 5; /* blocking read until 5 chars received */

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
