#ifndef LINKLAYER_H
#define LINKLAYER_H

typedef enum { TRANSMITTER, RECEIVER } LinkType;

int llopen(int port, LinkType type);
int llclose(int fd);
int llwrite(int fd, char* buffer, int length);
int llread(int fd, char* buffer);

#endif
