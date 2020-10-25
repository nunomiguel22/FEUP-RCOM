#include "LinkLayer.h"
#include "CharBuffer.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {
  if (argc != 3) {
    printf("First argument: 'PORTNUMBER' Second argument 'T' or 'R");
    return -1;
  }

  int port = atoi(argv[1]);

  LinkType type = RECEIVER;
  if (argv[2][0] == 'T') type = TRANSMITTER;

  int fd = llopen(port, type);

  llclose(fd);

  return 0;
}
