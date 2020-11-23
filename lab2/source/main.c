#include "ftp_app/ftp_app.h"
#include "utils/utils.h"

int main(int argc, char *argv[]) {

  char *username, *password, *server, *filepath, *filename;
  parseArguments(argc, argv, &username, &password, &server, &filepath, &filename);
  
  int socket_fd = ftp_connect(server);
  
  ftp_login(socket_fd, username, password);
  
  char client_ip[16];
  int client_port;
  ftp_enter_passive_mode(socket_fd, client_ip, &client_port);

  int data_socket_fd = ftp_connect_socket(client_ip, client_port);

  /* RETRIEVE FILE retr */
  /* Download */

  cleanup(username, password, server, socket_fd, data_socket_fd);
  return 0;
}

