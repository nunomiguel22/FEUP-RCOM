#ifndef LINKLAYER_H
#define LINKLAYER_H

/**
 * Enum of types of Link Layer connection. RECEIVER(0x01) or TRANSMITTER(0x01).
 */
typedef enum { TRANSMITTER = 0x00, RECEIVER = 0x01 } LinkType;

/**
 * Establish connection between ports.
 *
 * @param port Number of the serial port.
 * @param type This will set the connection type to RECEIVER or TRANSMITTER
 * @return file descriptor of the connection on success, -1 on failure
 */
int llopen(int port, LinkType type);
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

#endif
