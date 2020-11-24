#ifndef FTP_APP_H
#define FTP_APP_H

#include <stdbool.h>

int ftp_connect(char *server);
int ftp_connect_socket(char *ip, int port);
void ftp_login(int socket_fd, char * username, char* password);
void ftp_enter_passive_mode(int socket_fd, char *client_ip, int *client_port);
void ftp_retrieve_file(int socket_fd, char *filepath, char *filename);
void ftp_download(int data_fd, char * filename);

#endif
