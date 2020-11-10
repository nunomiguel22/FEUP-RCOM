#include "link_layer.h"

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

#define LL_LOG_INFORMATION  // Log general information
//#define LL_LOG_BUFFER       // Log entire frame
#define LL_LOG_FRAMES  // Log frame headers

/* POSIX compliant source */
#define _POSIX_SOURCE 1

#define INF_FRAME_SIZE 6
#define CONTROL_FRAME_SIZE 5
#define MAX_PORT_LENGTH 20

#define MAX_TRANSMISSION_ATTEMPS 3
#define TIMEOUT_DURATION 3
#define DEFAULT_BAUDRATE B38400

/*  Port name prefix */
#ifdef __linux__
#define PORT_NAME "/dev/ttyS"
#elif _WIN32
#define PORT_NAME "COM"
#else
#define PORT_NAME "/dev/ttyS"
#endif

typedef unsigned char uchar_t;
typedef struct {
  char port[MAX_PORT_LENGTH];
  int baud_rate;
  unsigned int sequence_number;
  unsigned int timeout;
  unsigned int num_transmissions;
} link_layer;

typedef enum {
  FD_FIELD = 0x00,    // Start Flag
  AF_FIELD = 0x01,    // Address
  C_FIELD = 0x02,     // Control
  BCC1_FIELD = 0x03,  // BCC1
  DATA_FIELD = 0x04   // Data field start
} ll_control_frame_field;

struct termios oldtio;
static link_layer ll;
static link_type ltype;
static ll_statistics stats;
static bool ll_init = false;
static int afd;

volatile bool alarm_triggered;
volatile unsigned int transmission_attempts = 0;

/* Declarations */
// Link Layer
int ll_open_transmitter(int fd);
int ll_open_receiver(int fd);
int read_frame(int fd, char_buffer *frame);
int validate_control_frame(char_buffer *frame);
void build_control_frame(char_buffer *frame, ll_control_type type);
void build_data_frame(char_buffer *frame, char *buffer, int length);
char get_address_field(link_type lnk, ll_control_type type);
bool is_control_command(ll_control_type type);
bool is_frame_control_type(char_buffer *frame, ll_control_type type);
void printControlType(ll_control_type type);
int send_frame(int fd, char_buffer *frame);
int frame_exchange(int fd, char_buffer *frame, ll_control_type reply);
int send_control_frame(int fd, ll_control_type type);
void log_frame(char_buffer *frame, const char *type);
int get_termios_baudrate(int baudrate);

void log_msg(const char *msg) {
#ifdef LL_LOG_INFORMATION
  fprintf(stderr, "ll: %s\n", msg);
#endif
}

/**
 *
 *
 * SIGNALS
 *
 */

void sig_alarm_handler(int sig_num) {
  if (sig_num == SIGALRM) {
    ++transmission_attempts;
    alarm_triggered = true;
  }
}
void set_alarm(unsigned int seconds) {
  alarm_triggered = false;
  alarm(seconds);
}

void set_alarm_handler() {
  signal(SIGALRM, sig_alarm_handler);
  alarm_triggered = false;
  transmission_attempts = 0;
}

void reset_alarm_handler() {
  alarm(0);
  signal(SIGALRM, NULL);
  alarm_triggered = false;
  transmission_attempts = 0;
}

bool was_alarm_triggered() {
  if (alarm_triggered) {
    log_msg("connection timed out...");
    return true;
  }
  return false;
}

// Reset termios on CTRL+C
void sig_int_handler(int sig) {
  if (sig == SIGINT) {
    tcsetattr(afd, TCSANOW, &oldtio);
    exit(-1);
  }
}

/**
 *
 *
 * LINK LAYER
 *
 */

void llabort(int fd) { close_serial_port(fd); }

void ll_setup(int timeout, int max_retries, int baudrate) {
  ll.timeout = timeout;
  ll.num_transmissions = max_retries;
  ll.baud_rate = get_termios_baudrate(baudrate);
  ll_init = true;
}

// LLOPEN
int llopen(int port, link_type type) {
  signal(SIGINT, sig_int_handler);
  afd = init_serial_port(port, type);
  if (afd == -1) return -1;

  if (type == TRANSMITTER) return ll_open_transmitter(afd);
  return ll_open_receiver(afd);
}

// LLCLOSE
int llclose(int fd) {
  log_msg("llclose - communicating disconnect\n");

  if (ltype == TRANSMITTER) {
    // Send Disc, receive DISC
    char_buffer discFrame;
    build_control_frame(&discFrame, LL_DISC);
    if (frame_exchange(fd, &discFrame, LL_DISC) == -1) {
      log_msg("warning - failed to communicate disconnect");
      char_buffer_destroy(&discFrame);
      close_serial_port(fd);
      return LL_ERROR_GENERAL;
    }

    char_buffer_destroy(&discFrame);

    // Send UA
    send_control_frame(fd, LL_UA);
    usleep(50);
    log_msg("llclose - disconnected.\n");
    close_serial_port(fd);
    return 1;
  }

  if (ltype == RECEIVER) {
    while (true) {
      /* Wait for DISC */
      char_buffer reply_frame;
      int res = read_frame(fd, &reply_frame);
      bool is_disc = is_frame_control_type(&reply_frame, LL_DISC);
      char_buffer_destroy(&reply_frame);

      if (!is_disc) log_msg("frame ignored - unexpected control field");

      // If invalid frame or not a DISC command, retry
      if (res != -1 && is_disc) break;
    }
    /* Send DISC, receive UA */
    char_buffer disc_frame;
    build_control_frame(&disc_frame, LL_DISC);
    int res = frame_exchange(fd, &disc_frame, LL_UA);
    if (res == -1) log_msg("llclose - failed to communicate disconnect");

    log_msg("llclose - disconnected.\n");
    close_serial_port(fd);
    return res;
  }
  close_serial_port(fd);
  return LL_ERROR_GENERAL;
}

// LLWRITE
int llwrite(int fd, char *buffer, int length) {
  char_buffer frame;
  build_data_frame(&frame, buffer, length);
  if (frame_exchange(fd, &frame, LL_RR) == -1) {
    log_msg("llwrite failed");

    char_buffer_destroy(&frame);
    return -1;
  }

  int size = frame.size;
  ll.sequence_number ^= 1;
  char_buffer_destroy(&frame);
  return size;
}

// LLREAD
int llread(int fd, char **buffer) {
  char_buffer frame;
  while (true) {
    if (read_frame(fd, &frame) == -1) {
      char_buffer_destroy(&frame);
      continue;
    }

    // Ignore
    if (!is_frame_control_type(&frame, LL_INF)) {
      if (is_frame_control_type(&frame, LL_SET)) {
        log_msg("frame ignored - unexpected SET control");
        send_control_frame(fd, LL_UA);
      } else
        log_msg("frame ignored - unexpected control field");
      char_buffer_destroy(&frame);
      continue;
    }

    // Check seq number for duplicate frames
    if ((frame.buffer[C_FIELD] >> 6) == (uchar_t)ll.sequence_number) {
      log_msg("frame ignored - duplicate");
      send_control_frame(fd, LL_RR);
      continue;
    }

    char_buffer packet;
    char_buffer_init(&packet, INF_FRAME_SIZE);

    uchar_t bcc2 = 0x00;  // Calculated BCC2
    // Get the packet BBC2 value, check for ESC_MOD
    uchar_t packet_bcc2 = frame.buffer[frame.size - 2];
    unsigned int dataLimit = frame.size - 2;

    // Adjust for BCC2 escape flag
    if (frame.buffer[frame.size - 3] == LL_ESC) {
      packet_bcc2 ^= LL_ESC_MOD;
      --dataLimit;
#ifdef DEBUG_PRINT_INFORMATION
      printf("BCC2 after byte destuffing: %x\n", packet_bcc2);
#endif
    }
    // Destuff bytes and calculate BCC2
    for (unsigned int i = DATA_FIELD; i < dataLimit; ++i) {
      uchar_t temp;
      if (frame.buffer[i] == LL_ESC) {
        temp = (uchar_t)(frame.buffer[++i]) ^ LL_ESC_MOD;

      } else
        temp = (uchar_t)frame.buffer[i];

      bcc2 ^= temp;
      char_buffer_push(&packet, temp);
    }

    // BCC2 check
    if (bcc2 != packet_bcc2) {
      printf(
          "ll: frame rejected - failed BBC2 check, expected: %x, received %x\n",
          bcc2, packet_bcc2);
      send_control_frame(fd, LL_REJ);
      char_buffer_destroy(&packet);
      char_buffer_destroy(&frame);
      continue;
    }

    // Frame read successfuly, flip seq number and reply with RR
    ll.sequence_number ^= 1;
    send_control_frame(fd, LL_RR);
    char_buffer_destroy(&frame);
    *buffer = (char *)packet.buffer;
    return packet.size;
  }
  return LL_ERROR_GENERAL;
}

ll_statistics ll_get_stats() { return stats; }

/**
 *
 *
 * LINK LAYER AUXILIAR
 *
 */

int ll_open_transmitter(int fd) {
  ll.sequence_number = 1;

  char_buffer set_frame;
  build_control_frame(&set_frame, LL_SET);
  if (frame_exchange(fd, &set_frame, LL_UA) == -1) {
    char_buffer_destroy(&set_frame);
    return LL_ERROR_GENERAL;
  }
  char_buffer_destroy(&set_frame);

  log_msg("llopen - Connected.\n");
  return fd;
}

int ll_open_receiver(int fd) {
  ll.sequence_number = 0;
  while (true) {
    log_msg("llopen - Wainting for connection\n");

    ll_control_type type = LL_DISC;
    while (type != LL_SET) {
      char_buffer set_frame;
      if (read_frame(fd, &set_frame) == -1) continue;

      type = set_frame.buffer[C_FIELD];

      if (type != LL_SET) log_msg("frame ignored - unexpected control field");

      char_buffer_destroy(&set_frame);
    }
    if (send_control_frame(fd, LL_UA) == -1) {
      log_msg("llopen - was unable to send UA packet");
      return LL_ERROR_GENERAL;
    }

    log_msg("llopen - connected.\n");
    return fd;
  }
}

/*
 *
 *
 * FRAMES
 *
 */

int frame_exchange(int fd, char_buffer *frame, ll_control_type reply) {
  set_alarm_handler();
  while (transmission_attempts < ll.num_transmissions) {
    send_frame(fd, frame);
    set_alarm(ll.timeout);
    while (true) {
      char_buffer reply_frame;
      int res = read_frame(fd, &reply_frame);

      // Timeout or invalid frame received
      if (res == -1) {
        tcflush(fd, TCIOFLUSH);
        char_buffer_destroy(&reply_frame);
        break;
      }

      bool is_rej = is_frame_control_type(&reply_frame, LL_REJ);
      bool is_rr = is_frame_control_type(&reply_frame, LL_RR);

      // Verify if RR or REJ is a duplicate
      if (is_rej || is_rr) {
        uchar_t replySeq = ((uchar_t)(reply_frame.buffer[C_FIELD]) >> 7);
        if (replySeq != (uchar_t)(ll.sequence_number)) {
          log_msg("frame ignored - duplicate");
          ++stats.frames_lost;
          char_buffer_destroy(&reply_frame);
          continue;
        }
      }
      // Received REJ, resend frame
      if (is_rej) {
        char_buffer_destroy(&reply_frame);
        log_msg("frame Rejected by receiver");
        ++stats.frames_lost;
        ++transmission_attempts;
        break;
      }
      // Exchange was sucessful
      if (is_frame_control_type(&reply_frame, reply)) {
        char_buffer_destroy(&reply_frame);
        reset_alarm_handler();
        return 1;
      } else {
        log_msg("frame ignored - unexpected control field");
        ++stats.frames_lost;
      }
    }
  }

  reset_alarm_handler();
  log_msg("error: exceeded transmission attempts, connection failed");

  return LL_ERROR_GENERAL;
}

int send_control_frame(int fd, ll_control_type type) {
  char_buffer frame;
  build_control_frame(&frame, type);
  int res = send_frame(fd, &frame);
  char_buffer_destroy(&frame);
  return res;
}

int send_frame(int fd, char_buffer *frame) {
  if (write(fd, frame->buffer, frame->size) < 0) {
    log_msg("warning - unable to write frame to port");
    return LL_ERROR_GENERAL;
  }
  log_frame(frame, "sent");
  return 0;
}

int read_frame(int fd, char_buffer *frame) {
  if (frame == NULL) return LL_ERROR_GENERAL;
  char_buffer_init(frame, CONTROL_FRAME_SIZE);

  char inc_byte = 0x00;
  int read_status = 0;
  // Clear buffer and wait for a flag
  while (read_status <= 0 ||
         (uchar_t)inc_byte != LL_FLAG) {  // TO-DO Implement timeout
    if (was_alarm_triggered()) return LL_ERROR_GENERAL;
    read_status = read(fd, &inc_byte, 1);
  }
  char_buffer_push(frame, (uchar_t)inc_byte);
  // Reset vars
  read_status = 0;
  inc_byte = 0x00;
  // Read serial until flag is found
  while (inc_byte != LL_FLAG) {
    if (was_alarm_triggered()) return LL_ERROR_GENERAL;
    read_status = read(fd, &inc_byte, 1);
    if (read_status <= 0) continue;
    char_buffer_push(frame, inc_byte);
  }

  ++stats.frames_total;

#ifdef LL_LOG_BUFFER
  char_buffer_printHex(frame);
#endif

  if (validate_control_frame(frame) < 0) {
    log_msg("frame ignored - failed validation of header");
    ++stats.frames_lost;
    return LL_ERROR_GENERAL;
  }

  return LL_ERROR_OK;
}

int validate_control_frame(char_buffer *frame) {
  if (frame == NULL || frame->buffer == NULL) return -1;

  if (frame->size < CONTROL_FRAME_SIZE) {
    char_buffer_printHex(frame);
    return LL_ERROR_FRAME_TOO_SMALL;
  }

#ifdef LL_LOG_FRAMES
  log_frame(frame, "received");
#endif

  // Start Flag
  if (frame->buffer[FD_FIELD] != LL_FLAG) return LL_ERROR_BAD_START_FLAG;
  // Check address
  char expectedAF = get_address_field(ltype ^ 1, frame->buffer[C_FIELD]);
  if (frame->buffer[AF_FIELD] != expectedAF) return LL_ERROR_BAD_ADDRESS;
  // Check BCC1
  if (frame->buffer[BCC1_FIELD] !=
      (uchar_t)(expectedAF ^ frame->buffer[C_FIELD]))
    return LL_ERROR_BAD_BCC1;
  // Last element flag
  if (frame->buffer[frame->size - 1] != (uchar_t)LL_FLAG)
    return LL_ERROR_BAD_END_FLAG;

  return LL_ERROR_OK;
}

void build_control_frame(char_buffer *frame, ll_control_type type) {
  char_buffer_init(frame, CONTROL_FRAME_SIZE);
  char_buffer_push(frame, LL_FLAG);                         // FLAG
  char_buffer_push(frame, get_address_field(ltype, type));  // ADDRESS

  if (type == LL_RR || type == LL_REJ) type |= ll.sequence_number << 7;  // N(r)

  char_buffer_push(frame, type);                                 // Control type
  char_buffer_push(frame, frame->buffer[1] ^ frame->buffer[2]);  // BCC1
  char_buffer_push(frame, LL_FLAG);                              // FLAG
}

void build_data_frame(char_buffer *frame, char *buffer, int length) {
  char_buffer_init(frame, length + INF_FRAME_SIZE);
  char_buffer_push(frame, LL_FLAG);                           // FLAG
  char_buffer_push(frame, get_address_field(ltype, LL_INF));  // ADDRESS
  char_buffer_push(frame,
                   LL_INF | (ll.sequence_number << 6));  // Control and N(s)
  char_buffer_push(frame, frame->buffer[1] ^ frame->buffer[2]);  // BCC1

  // Add buffer to frame and calculate bcc2
  uchar_t bcc2 = 0x00;
  for (int i = 0; i < length; ++i) {
    // BCC2
    bcc2 ^= (uchar_t)buffer[i];
    // Byte stuffing
    if (buffer[i] == (uchar_t)LL_FLAG || buffer[i] == (uchar_t)LL_ESC) {
      char_buffer_push(frame, (uchar_t)LL_ESC);
      char_buffer_push(frame, buffer[i] ^ (uchar_t)LL_ESC_MOD);
    } else
      char_buffer_push(frame, buffer[i]);
  }

  // Bytestuffin on BBC2 when needed
  if (bcc2 == (uchar_t)LL_ESC || bcc2 == (uchar_t)LL_FLAG) {
    char_buffer_push(frame, (uchar_t)LL_ESC);
    char_buffer_push(frame, (uchar_t)(bcc2 ^ (uchar_t)LL_ESC_MOD));
  } else
    char_buffer_push(frame, bcc2);

  char_buffer_push(frame, (uchar_t)LL_FLAG);

#ifdef LL_LOG_BUFFER
  char_buffer_printHex(frame);
#endif
}

char get_address_field(link_type lnk, ll_control_type type) {
  type &= 0x0F;
  if (lnk == RECEIVER && is_control_command(type))
    return LL_AF2;
  else if (lnk == TRANSMITTER && !is_control_command(type))
    return LL_AF2;
  return LL_AF1;
}

bool is_control_command(ll_control_type type) {
  type &= 0x0F;
  if (type == LL_INF || type == LL_DISC || type == LL_SET) return true;
  return false;
}

bool is_frame_control_type(char_buffer *frame, ll_control_type type) {
  if (frame == NULL || frame->buffer == NULL) return false;
  type &= 0x0F;
  ll_control_type frameType = frame->buffer[C_FIELD] & 0x0F;
  return frameType == type;
}

const char *get_control_type_str(ll_control_type type) {
  type &= 0x0F;
  switch (type) {
    case LL_INF: {
      return "INF";
    }
    case LL_SET: {
      return "SET";
    }
    case LL_DISC: {
      return "DISC";
    }
    case LL_UA: {
      return "UA";
    }
    case LL_RR: {
      return "RR";
    }
    case LL_REJ: {
      return "REJ";
    }
    default:
      return "UNK";
  }
}

void log_frame(char_buffer *frame, const char *type) {
#ifdef LL_LOG_FRAMES
  printf("ll: %s packet %s", type,
         get_control_type_str(frame->buffer[C_FIELD]));

  // Header
  printf("\t\t\t[F %x][A %x][C %x][BCC1 %x]", frame->buffer[FD_FIELD],
         frame->buffer[AF_FIELD], (uchar_t)frame->buffer[C_FIELD],
         (uchar_t)frame->buffer[BCC1_FIELD]);
  // BCC2
  if ((frame->buffer[C_FIELD] & 0x0F) == LL_INF)
    printf("[BCC2 %x]", (uchar_t)frame->buffer[frame->size - 2]);

  // Tail
  printf("[F %x] Frame size: %d bytes\n", frame->buffer[frame->size - 1],
         frame->size);
#endif
}

/**
 *
 *
 * SERIAL PORT
 *
 */

int get_termios_baudrate(int baudrate) {
  switch (baudrate) {
    case 0:
      return B0;
    case 50:
      return B50;
    case 75:
      return B75;
    case 110:
      return B110;
    case 134:
      return B134;
    case 150:
      return B150;
    case 200:
      return B200;
    case 300:
      return B300;
    case 600:
      return B600;
    case 1200:
      return B1200;
    case 1800:
      return B1800;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    default:
      return DEFAULT_BAUDRATE;
  }
}

int init_serial_port(int port, link_type type) {
  // Init ll struct
  snprintf(ll.port, MAX_PORT_LENGTH, "%s%d", PORT_NAME, port);
  if (!ll_init)
    ll_setup(TIMEOUT_DURATION, MAX_TRANSMISSION_ATTEMPS, DEFAULT_BAUDRATE);

  ltype = type;

  int fd = open(ll.port, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror(ll.port);
    return -1;
  }

  struct termios newtio;

  if (tcgetattr(fd, &oldtio) == -1) { /* save current port settings */
    perror("tcgetattr");
    close(fd);
    return -1;
  }

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = ll.baud_rate | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME] = 20; /* inter-character timer unused */
  newtio.c_cc[VMIN] = 0;   /* blocking read until 5 chars received */

  /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) pr�ximo(s) caracter(es)
  */

  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
    perror("tcsetattr");
    close(fd);
    return -1;
  }

  return fd;
}

void close_serial_port(int fd) {
  tcsetattr(fd, TCSANOW, &oldtio);
  close(fd);
}

/** Testing **/
int send_raw_data(int fd, char *buffer, int length) {
  return write(fd, buffer, length);
}
