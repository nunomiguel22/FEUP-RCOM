#ifndef LINKLAYER_H
#define LINKLAYER_H

typedef enum { TRANSMITTER, RECEIVER } linkType;

int llopen(int port, linkType type);
int llclose(int fd);
int llwrite(int fd, char* buffer, int length);
int llread(int fd, char* buffer);

#endif
