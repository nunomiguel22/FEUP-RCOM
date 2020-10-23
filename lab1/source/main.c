#include "LinkLayer.h"
#include "CharBuffer.h"
#include <unistd.h>

int main(int argc, char** argv) {
  int fd = llopen(11, RECEIVER);

  sleep(3);

  llclose(fd);

  return 0;
}
