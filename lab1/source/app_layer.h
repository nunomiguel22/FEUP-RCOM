#ifndef APP_LAYER_H
#define APP_LAYER_H

#include <stdint.h>
#include <stdio.h>

void al_setup(int timeout, int baudrate, int max_retries, int frag_size);
int al_sendFile(const char *filename, int port);
int al_receiveFile(const char *filename, int port);

#endif
