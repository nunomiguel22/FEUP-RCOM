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

/* DEBUG FLAGS */
//#define DEBUG_PRINT_BUFFER //Print hexadecimal value for each received frame
#define DEBUG_PRINT_FRAMES       // Print the control type of each frame
#define DEBUG_PRINT_INFORMATION  // General Information and errors in the ll
#define DEBUG_PRINT_ERROR(msg) fprintf(stderr, "llerror: %s\n", msg)

/* POSIX compliant source */
#define _POSIX_SOURCE 1

/* Framing */
#define INF_FRAME_SIZE 6
#define CONTROL_FRAME_SIZE 5
#define FLAG 0x7E     // Flag for beggining and ending of frame
#define ESC 0x7D      // Escape character for byte stuffing
#define ESC_MOD 0x20  // Stuffing byte
#define AF1 0x03      // Transmitter commands, Receiver replys
#define AF2 0x01      // Transmitter replys, Receiver commands

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
  DATAFIELD = 0x04   // Data field start
} ControlFrameField;

struct termios oldtio;
static LinkLayer ll;
static LinkType linkType;
volatile bool alarmTriggered;
volatile unsigned int transmissionAttemps = 0;

/* Declarations */
// Alarm
void alarmHandler(int signum);
void setAlarm(unsigned int seconds);
void setAlarmHandler();
void resetAlarmHandler();
bool wasAlarmTriggered();
// Link Layer
int llopenTransmitter(int fd);
int llopenReceiver(int fd);
int readFrame(int fd, CharBuffer *frame);
int validateControlFrame(CharBuffer *frame);
void buildControlFrame(CharBuffer *frame, ControlTypes type);
void buildDataFrame(CharBuffer *frame, char *buffer, int length);
char getAddressField(LinkType lnk, ControlTypes type);
bool isControlCommand(ControlTypes type);
bool isFrameControlType(CharBuffer *frame, ControlTypes type);
void printControlType(ControlTypes type);
int sendFrame(int fd, CharBuffer *frame);
int exchangeFrame(int fd, CharBuffer *frame, ControlTypes reply);
// Serial Port
int initSerialPort(int port, LinkType type);

/* Definitions */

/**
 *
 *
 * ALARM
 *
 */

void alarmHandler(int signum) {
  if (signum == SIGALRM) {
    ++transmissionAttemps;
    alarmTriggered = true;
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

bool wasAlarmTriggered() {
  if (alarmTriggered) {
#ifdef DEBUG_PRINT_INFORMATION
    DEBUG_PRINT_ERROR("Connection timed out...");
#endif
    return true;
  }
  return false;
}

/**
 *
 *
 * LINK LAYER
 *
 */

int llopen(int port, LinkType type) {
  int fd = initSerialPort(port, type);
  if (fd == -1) return -1;

  if (type == TRANSMITTER) return llopenTransmitter(fd);
  return llopenReceiver(fd);
}

int llclose(int fd) {
#ifdef DEBUG_PRINT_INFORMATION
  printf("llclose: communicating disconnect\n");
#endif

  if (linkType == TRANSMITTER) {
    // Send Disc, receive DISC
    CharBuffer discFrame;
    buildControlFrame(&discFrame, DISC);
    if (exchangeFrame(fd, &discFrame, DISC) == -1) {
#ifdef DEBUG_PRINT_INFORMATION
      DEBUG_PRINT_ERROR("llclose failed to communicate disconnect");
#endif
      CharBuffer_destroy(&discFrame);
      tcsetattr(fd, TCSANOW, &oldtio);
      close(fd);
      return -1;
    }
    CharBuffer_destroy(&discFrame);
    // Send UA
    CharBuffer uaFrame;
    buildControlFrame(&uaFrame, UA);
    sendFrame(fd, &uaFrame);
    CharBuffer_destroy(&uaFrame);
#ifdef DEBUG_PRINT_INFORMATION
    printf("llclose: Disconnected.\n");
#endif
    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    return 1;
  }

  if (linkType == RECEIVER) {
    // Receive DISC
    setAlarmHandler();
    setAlarm(ll.timeout * 5);
    while (true) {
      CharBuffer replyFrame;
      int res = readFrame(fd, &replyFrame);
      // Timeout or invalid frame received
      if (res == -1) {
        CharBuffer_destroy(&replyFrame);
        resetAlarmHandler();
#ifdef DEBUG_PRINT_INFORMATION
        DEBUG_PRINT_ERROR("llclose failed to communicate disconnect");
#endif
        tcsetattr(fd, TCSANOW, &oldtio);
        close(fd);
        return -1;
      }
      if (isFrameControlType(&replyFrame, DISC)) {
        CharBuffer_destroy(&replyFrame);
        break;
      }
      CharBuffer_destroy(&replyFrame);
    }
    resetAlarmHandler();

    // Send DISC, receive UA
    CharBuffer discFrame;
    buildControlFrame(&discFrame, DISC);
    if (exchangeFrame(fd, &discFrame, UA) == -1) {
      CharBuffer_destroy(&discFrame);
#ifdef DEBUG_PRINT_INFORMATION
      DEBUG_PRINT_ERROR("llclose failed to communicate disconnect");
#endif
      tcsetattr(fd, TCSANOW, &oldtio);
      close(fd);
      return -1;
    }
    CharBuffer_destroy(&discFrame);
    close(fd);
#ifdef DEBUG_PRINT_INFORMATION
    printf("llclose: Disconnected.\n");
#endif
    tcsetattr(fd, TCSANOW, &oldtio);
    return 1;
  }
#ifdef DEBUG_PRINT_INFORMATION
  printf("llclose: Disconnected.\n");
#endif
  tcsetattr(fd, TCSANOW, &oldtio);
  return 1;
}

int llwrite(int fd, char *buffer, int length) {
  CharBuffer frame;
  buildDataFrame(&frame, buffer, length);
  if (exchangeFrame(fd, &frame, RR) == -1) {
#ifdef DEBUG_PRINT_INFORMATION
    DEBUG_PRINT_ERROR("llwrite failed");
#endif
    CharBuffer_destroy(&frame);
    return -1;
  }
  int size = frame.size;
  ll.sequenceNumber ^= 1;
  CharBuffer_destroy(&frame);
  return size;
}

int llread(int fd, char **buffer) {
  CharBuffer frame;
  while (true) {
    if (readFrame(fd, &frame) == -1) {
      CharBuffer_destroy(&frame);
      continue;
    }
    if ((frame.buffer[CFIELD] & 0x0F) != INF) {
      CharBuffer_destroy(&frame);
      continue;
    }

    if ((frame.buffer[CFIELD] >> 6) != (char)(ll.sequenceNumber ^ 1)) {
      CharBuffer rrFrame;
      buildControlFrame(&rrFrame, RR);
      sendFrame(fd, &rrFrame);
      CharBuffer_destroy(&frame);
      continue;
    }

    CharBuffer packet;
    CharBuffer_init(&packet, INF_FRAME_SIZE);

    unsigned char bcc2 = 0x00;
    for (unsigned int i = DATAFIELD; i < frame.size - 2; ++i) {
      unsigned char temp;
      if (frame.buffer[i] == ESC)
        temp = (unsigned char)(frame.buffer[++i]) ^ ESC_MOD;
      else
        temp = (unsigned char)frame.buffer[i];

      bcc2 ^= temp;
      CharBuffer_push(&packet, temp);
    }

    if (bcc2 != (unsigned char)frame.buffer[frame.size - 2]) {
      CharBuffer rejFrame;
      buildControlFrame(&rejFrame, REJ);
      sendFrame(fd, &rejFrame);
      CharBuffer_destroy(&packet);
      CharBuffer_destroy(&frame);
      continue;
    }

    ll.sequenceNumber ^= 1;
    CharBuffer rrFrame;
    buildControlFrame(&rrFrame, RR);
    sendFrame(fd, &rrFrame);
    CharBuffer_destroy(&frame);
    *buffer = packet.buffer;
    return packet.size;
  }
  return -1;
}

/**
 *
 *
 * LINK LAYER AUXILIAR
 *
 */

int llopenTransmitter(int fd) {
  ll.sequenceNumber = 1;
  CharBuffer setFrame;
  buildControlFrame(&setFrame, SET);
  if (exchangeFrame(fd, &setFrame, UA) == -1) {
    CharBuffer_destroy(&setFrame);
    return -1;
  }
  CharBuffer_destroy(&setFrame);
#ifdef DEBUG_PRINT_INFORMATION
  printf("llopen: Connected.\n");
#endif
  return fd;
}

int llopenReceiver(int fd) {
  ll.sequenceNumber = 0;
  while (true) {
#ifdef DEBUG_PRINT_INFORMATION
    printf("llopen: Wainting for connection\n");
#endif

    ControlTypes type = DISC;
    while (type != SET) {
      CharBuffer setFrame;
      if (readFrame(fd, &setFrame) == -1) continue;

      type = setFrame.buffer[CFIELD];
      CharBuffer_destroy(&setFrame);
    }

    CharBuffer uaFrame;
    buildControlFrame(&uaFrame, UA);
    sendFrame(fd, &uaFrame);
    CharBuffer_destroy(&uaFrame);
#ifdef DEBUG_PRINT_INFORMATION
    printf("llopen: Connected.\n");
#endif
    return fd;
  }
}

/*
 *
 *
 * FRAMES
 *
 */

int exchangeFrame(int fd, CharBuffer *frame, ControlTypes reply) {
  setAlarmHandler();
  while (transmissionAttemps < ll.numTransmissions) {
    sendFrame(fd, frame);
    setAlarm(ll.timeout);

    while (true) {
      CharBuffer replyFrame;
      int res = readFrame(fd, &replyFrame);
      // Timeout or invalid frame received
      if (res == -1) {
        CharBuffer_destroy(&replyFrame);
        break;
      }
      // Received REJ, resend frame
      if (isFrameControlType(&replyFrame, REJ)) {
        CharBuffer_destroy(&replyFrame);
#ifdef DEBUG_PRINT_INFORMATION
        DEBUG_PRINT_ERROR("Frame Rejected by receiver");
#endif
        ++transmissionAttemps;
        break;
      }
      // Exchange was sucessful
      if (isFrameControlType(&replyFrame, reply)) {
        CharBuffer_destroy(&replyFrame);
        resetAlarmHandler();
        return 0;
      }
    }
  }

  resetAlarmHandler();
#ifdef DEBUG_PRINT_INFORMATION
  DEBUG_PRINT_ERROR("Exceeded transmission attempts, connection failed");
#endif

  return -1;
}

int sendFrame(int fd, CharBuffer *frame) {
  write(fd, frame->buffer, frame->size);

#ifdef DEBUG_PRINT_FRAMES
  printf("ll: Sent packet ");
  printControlType(frame->buffer[CFIELD]);
  printf("\n");
#endif

  return 0;
}

int readFrame(int fd, CharBuffer *frame) {
  if (frame == NULL) return -1;
  CharBuffer_init(frame, CONTROL_FRAME_SIZE);

  char incByte = 0x00;
  int readStatus = 0;
  // Clear buffer and wait for a flag
  while (readStatus <= 0 || incByte != FLAG) {  // TO-DO Implement timeout
    if (wasAlarmTriggered()) return -1;
    readStatus = read(fd, &incByte, 1);
  }
  CharBuffer_push(frame, incByte);
  // Reset vars
  readStatus = 0;
  incByte = 0x00;
  // Read serial until flag is found
  while (incByte != FLAG) {
    readStatus = read(fd, &incByte, 1);
    if (wasAlarmTriggered()) return -1;
    if (readStatus <= 0) continue;

    CharBuffer_push(frame, incByte);
  }

#ifdef DEBUG_PRINT_BUFFER
  CharBuffer_printHex(frame);
#endif

  if (validateControlFrame(frame) == -1) {
#ifdef DEBUG_PRINT_INFORMATION
    DEBUG_PRINT_ERROR("frame failed validation of header");
#endif
  }

#ifdef DEBUG_PRINT_FRAMES
  printf("ll: Received control packet ");
  printControlType(frame->buffer[CFIELD]);
  printf("\n");
#endif

  return 0;
}

int validateControlFrame(CharBuffer *frame) {
  if (frame == NULL || frame->buffer == NULL) return -1;
  if (frame->size < CONTROL_FRAME_SIZE) return -1;
  // Start Flag
  if (frame->buffer[FSFIELD] != FLAG) return -1;
  // Check address
  char expectedAF = getAddressField(linkType ^ 1, frame->buffer[CFIELD]);
  if (frame->buffer[AFIELD] != expectedAF) return -1;
  // Check BCC1
  if (frame->buffer[BCC1FIELD] != (char)(expectedAF ^ frame->buffer[CFIELD]))
    return -1;
  if (frame->buffer[frame->size - 1] != (char)FLAG)
    return -1;  // Last element flag

  return 0;
}

void buildControlFrame(CharBuffer *frame, ControlTypes type) {
  CharBuffer_init(frame, CONTROL_FRAME_SIZE);
  CharBuffer_push(frame, FLAG);                             // FLAG
  CharBuffer_push(frame, getAddressField(linkType, type));  // ADDRESS

  if (type == RR || type == REJ) type |= ll.sequenceNumber << 7;  // N(r)

  CharBuffer_push(frame, type);                                 // Control type
  CharBuffer_push(frame, frame->buffer[1] ^ frame->buffer[2]);  // BCC1
  CharBuffer_push(frame, FLAG);                                 // FLAG
}

void buildDataFrame(CharBuffer *frame, char *buffer, int length) {
  CharBuffer_init(frame, length + INF_FRAME_SIZE);
  CharBuffer_push(frame, FLAG);                            // FLAG
  CharBuffer_push(frame, getAddressField(linkType, INF));  // ADDRESS
  CharBuffer_push(frame, INF | (ll.sequenceNumber << 6));  // Control and N(s)
  CharBuffer_push(frame, frame->buffer[1] ^ frame->buffer[2]);  // BCC1

  // Add buffer to frame and calculate bcc2
  unsigned char bcc2 = 0x00;
  for (int i = 0; i < length; ++i) {
    // BCC2
    bcc2 ^= (unsigned char)buffer[i];
    // Byte stuffing
    if (buffer[i] == FLAG || buffer[i] == ESC) {
      CharBuffer_push(frame, ESC);
      CharBuffer_push(frame, buffer[i] ^ ESC_MOD);
    } else
      CharBuffer_push(frame, buffer[i]);
  }

  // Bytestuffin on BBC2 when needed
  if (bcc2 == ESC || bcc2 == FLAG) {
    CharBuffer_push(frame, ESC);
    CharBuffer_push(frame, bcc2 ^ ESC_MOD);
  } else
    CharBuffer_push(frame, bcc2);

  CharBuffer_push(frame, FLAG);
}

char getAddressField(LinkType lnk, ControlTypes type) {
  type &= 0x0F;
  if (lnk == RECEIVER && isControlCommand(type))
    return AF2;
  else if (lnk == TRANSMITTER && !isControlCommand(type))
    return AF2;
  return AF1;
}

bool isControlCommand(ControlTypes type) {
  type &= 0x0F;
  if (type == INF || type == DISC || type == SET) return true;
  return false;
}

bool isFrameControlType(CharBuffer *frame, ControlTypes type) {
  if (frame == NULL || frame->buffer == NULL) return false;
  type &= 0x0F;
  ControlTypes frameType = frame->buffer[CFIELD] &= 0x0F;
  return frameType == type;
}

void printControlType(ControlTypes type) {
  type &= 0x0F;
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

/**
 *
 *
 * SERIAL PORT
 *
 */

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

  struct termios newtio;

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

  /* Empty serial port buffer */
  int ret = 3;
  char tempchar;
  while (ret >= 0) ret = read(fd, &tempchar, 1);

  return fd;
}
