#ifndef LINK_LAYER_H
#define LINK_LAYER_H

#include "char_buffer.h"

#define LL_FLAG 0x7E     // Flag for beggining and ending of frame
#define LL_ESC 0x7D      // Escape character for byte stuffing
#define LL_ESC_MOD 0x20  // Stuffing byte
#define LL_AF1 0x03      // Transmitter commands, Receiver replys
#define LL_AF2 0x01      // Transmitter replys, Receiver commands

typedef enum {
  LL_INF = 0x00,
  LL_SET = 0x03,
  LL_DISC = 0x0B,
  LL_UA = 0x07,
  LL_RR = 0x05,
  LL_REJ = 0x01
} ll_control_type;

typedef enum {
  LL_ERROR_GENERAL = -1,
  LL_ERROR_OK = 0,
  LL_ERROR_FRAME_TOO_SMALL = -2,
  LL_ERROR_BAD_START_FLAG = -3,
  LL_ERROR_BAD_ADDRESS = -4,
  LL_ERROR_BAD_BCC1 = -5,
  LL_ERROR_BAD_END_FLAG = -6
} ll_error_code;

typedef struct {
  unsigned int frames_total;
  unsigned int frames_lost;
} ll_statistics;

/**
 * Enum of types of Link Layer connection. RECEIVER(0x01) or TRANSMITTER(0x01).
 */
typedef enum { TRANSMITTER = 0x00, RECEIVER = 0x01 } link_type;

/**
 * Sets connection settings
 *
 * @param timeout Seconds until a frame with no response times out
 * @param max_retries Number of frame retransmission attempts until failure
 * @param baudrate Serial port baudrate
 */
void ll_setup(int timeout, int max_retries, int baudrate);
/**
 * Establish connection between ports.
 *
 * @param port Number of the serial port.
 * @param type This will set the connection type to RECEIVER or TRANSMITTER
 * @return file descriptor of the connection on success, -1 on failure
 */
int llopen(int port, link_type type);
/**
 * Close connection between ports, frees memory and closes file descriptor.
 *
 * @param fd File Descriptor of an established connection
 * @return 1 on success, -1 on failure
 */
int llclose(int fd);
/**
 * Send a buffer of a given length through the connection.
 *
 * @param fd File descriptor of the connection, given by llopen.
 * @param buffer A byte array of data to transmit
 * @param length Length of the buffer
 * @return Number of bytes transmitted or -1 on failure
 */
int llwrite(int fd, char* buffer, int length);
/**
 * Read a buffer through the connection. buffer should be freed after use.
 *
 * @param fd File descriptor of the connection, given by llopen.
 * @param buffer this should be the address of a (char *), the pointer will be
 * changed to the buffer location
 * @return number of read bytes or -1 on failure
 */
int llread(int fd, char** buffer);
/**
 * Statistics of transferred bytes and accepted/rejected/ignored frames
 *
 * @returns a struct with link layer statistics
 */
ll_statistics ll_get_stats();

/** TESTING **/

/**
 * Builds a data frame given a buffer
 *
 * @param frame char_buffer where frame will be stored
 * @param buffer Buffer with data to frame
 * @param length Data buffer length
 */
void build_data_frame(char_buffer* frame, char* buffer, int length);
/**
 * Sends a buffer of raw data through the port
 *
 * @param fd File descriptor of the connection, given by llopen.
 * @param buffer Buffer with data to send
 * @param length Data buffer length
 */
int send_raw_data(int fd, char* buffer, int length);
/**
 * Initiates serial port connection
 *
 * @param port Number of the serial port.
 * @param type This will set the connection type to RECEIVER or TRANSMITTER
 * @return Number of bytes written, -1 on failure
 */
int init_serial_port(int port, link_type type);
/**
 * Closes serial port connection
 *
 *  @param fd File descriptor of the connection, given by llopen.
 */
void close_serial_port(int fd);

#endif
