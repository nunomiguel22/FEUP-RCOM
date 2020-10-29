#ifndef APP_LAYER_H
#define APP_LAYER_H

#include <stdint.h>
#include <stdio.h>

int sendFile(const char *filename);
int receiveFile(const char *filename);

#endif
