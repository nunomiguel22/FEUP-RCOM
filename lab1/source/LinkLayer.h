#ifndef LINKLAYER_H
#define LINKLAYER_H

typedef enum { TRANSMITTER = 0x00, RECEIVER = 0x01 } LinkType;

int llopen(int port, LinkType type);
int llclose(int fd);
int llwrite(int fd, char* buffer, int length);
int llread(int fd, char** buffer);

#endif
