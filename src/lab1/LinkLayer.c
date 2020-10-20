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

/* POSIX compliant source */
#define _POSIX_SOURCE 1

/* Framing */
#define MAX_FRAME_SIZE 6

/* Serial Port */
#define DEFAULT_BAUDRATE B38400
#define MAX_PORT_LENGTH 20

/*  Port Name */
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
  char frame[MAX_FRAME_SIZE];
  bool init;
} LinkLayer;

/* Globals */
LinkLayer ll;

/* Link Layer Functions */
int llopen(int port, linkType type) {
  int fd = initSerialPort(port);
  if (fd == -1) return fd;

  // TODO: SEND AND READ CONTROL FRAME DEPENDING ON TRANS/RECEIVER

  return fd;
}

int llclose(int fd);
int llwrite(int fd, char* buffer, int length);
int llread(int fd, char* buffer);

/* Auxiliar Functions */
int initSerialPort(int port) {
  // Init ll struct
  snprintf(ll.port, MAX_PORT_LENGTH, "%s%d", PORT_NAME, port);
  ll.baudRate = DEFAULT_BAUDRATE;

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
