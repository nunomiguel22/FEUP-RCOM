#include "ftp_app/ftp_app.h"
#include "utils/utils.h"



int main(int argc, char *argv[]) {

  char *user, *password, *server, *filepath, *filename;
  parseArguments(argc, argv, &user, &password, &server, &filepath, &filename);
  int socket_fd = tcp_connect(server);


  cleanup(user, password, server, socket_fd);
  return 0;
}



