#ifndef APP_LAYER_H
#define APP_LAYER_H

#include <stdint.h>
#include <stdio.h>

typedef struct {
  unsigned int baudrate;
  unsigned int timeout;
  unsigned int retries;
  unsigned int avg_bits_per_second;
  unsigned int data_packet_count;
  unsigned int file_size;
  unsigned int frames_total;
  unsigned int frames_lost;
  double transmission_duration_secs;
} al_statistics;

void al_setup(int timeout, int baudrate, int max_retries, int frag_size);
int al_sendFile(const char *filename, int port);
int al_receiveFile(const char *filename, int port);
al_statistics al_get_stats();
void al_print_stats();

#endif
