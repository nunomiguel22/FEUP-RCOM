#include "link_layer.h"
#include "app_layer.h"
#include "char_buffer.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define DEFAULT_MAX_TRANSMISSION_ATTEMPS 3
#define DEFAULT_TIMEOUT_DURATION 3
#define DEFAULT_BAUDRATE 38400
#define MAX_FRAGMENT_SIZE 0xFFFF

void print_usage(const char* arg) {
  printf("Usage:\n");
  printf("%s <file_name> <T|R> <port_number> [options]\n", arg);
  printf("T - Transmitter, R - Receiver\n");
  printf("Options:\n");
  printf("  -timeout=<seconds> \t\tSeconds until a frame is timed out\n");
  printf("  -baudrate=<rate> \t\tSerial port rate\n");
  printf(
      "  -max_retries=<retries> \tTimes a frame transmission can be "
      "retried\n");
  printf("  -frag_size=<size> \t\tMax size for data fragments\n");
  printf("\nExample: '%s pinguim.gif T 10'\n", arg);
}

int main(int argc, char** argv) {
  if (argc < 4) {
    print_usage(argv[0]);
    return -1;
  }

  char* file_name = argv[1];  // File name

  // Read link type
  link_type type = RECEIVER;
  if (argv[2][0] == 'T') type = TRANSMITTER;

  int port = atoi(argv[3]);  // Read Port

  /* Options */
  int timeout = DEFAULT_TIMEOUT_DURATION;
  int retries = DEFAULT_MAX_TRANSMISSION_ATTEMPS;
  int baudrate = DEFAULT_BAUDRATE;
  int frag_size = MAX_FRAGMENT_SIZE;

  for (int i = 4; i < argc; ++i) {
    if (!strncmp(argv[i], "-timeout=", 9) && strlen(argv[i]) > 9) {
      timeout = atoi(&argv[i][9]);
      continue;
    }

    if (!strncmp(argv[i], "-max_retries=", 13) && strlen(argv[i]) > 13) {
      retries = atoi(&argv[i][13]);
      continue;
    }

    if (!strncmp(argv[i], "-baudrate=", 10) && strlen(argv[i]) > 10) {
      baudrate = atoi(&argv[i][10]);
      continue;
    }

    if (!strncmp(argv[i], "-frag_size=", 11) && strlen(argv[i]) > 11) {
      frag_size = atoi(&argv[i][11]);
      continue;
    }
  }

  al_setup(timeout, baudrate, retries, frag_size);

  int res;
  if (type == RECEIVER) {
    res = al_receiveFile(file_name, port);
  } else {
    res = al_sendFile(file_name, port);
  }

  if (res >= 0) al_print_stats();

  return 0;
}
