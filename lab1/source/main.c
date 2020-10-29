#include "LinkLayer.h"
#include "AppLayer.h"
#include "CharBuffer.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char** argv) {
  if (argc != 3) {
    printf("First argument: 'PORTNUMBER' Second argument 'T' or 'R");
    return -1;
  }

  int port = atoi(argv[1]);

  LinkType type = RECEIVER;
  if (argv[2][0] == 'T') type = TRANSMITTER;

  if (type == RECEIVER) {
    receiveFile("files/downloads/large.jpg");
  } else {
    /*     char buffer[5] = {FLAG, AF2, SET, (AF1 ^ SET), FLAG};
        int fd = initSerialPort(port, TRANSMITTER);
        sendRawData(fd, buffer, 5);
        closeSerialPort(fd);

        sleep(3); */

    /*     int fd = llopen(11, TRANSMITTER);
        unsigned char bcc2 = 0x00;

        char buffer[11] = {FLAG, AF1, INF | 0x40, (AF1 ^ (INF | 0x40)),
                           'H',  'E', 'L',        'L',
                           'O',  2,   FLAG};

        sendRawData(fd, buffer, 11);
        closeSerialPort(fd); */

    sendFile("files/large.jpg");
  }

  return 0;
}

/** *
 * Tests
 *
 */
