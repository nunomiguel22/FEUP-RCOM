#include "utils.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <libgen.h>
#include <unistd.h>

#define FTP_COMMAND_MIN_SIZE 12
#define FTP_PREFIX "ftp://"
#define FTP_PREFIX_SIZE 6

void abort_bad_url(){
    fprintf(stderr, "Invalid URL\n");
    exit(-1);
}

int chop_string(char *string, char **result, char delim){
  int size =0;
  const int string_length = strlen(string);

  bool delim_found = false;
  for (int i = 0; i < string_length; ++i){
    if (string[i] == delim){
      delim_found = true;
      size = i;
      break;
    }
  }
  if (!delim_found)
    return -1;

  *result = (char *)malloc((size + 1)*sizeof(char));
  memcpy(*result, string, size);
  result[size] = 0;
  return size;
}

void parseArguments(int argc, char *argv[], char **user, char** password, char **server, char **filepath, char **filename){
  
  if (argc < 2){
    abort_bad_url();
  }

  char * ftp_command = argv[1];

  if (strlen(ftp_command) < FTP_COMMAND_MIN_SIZE)
    abort_bad_url();

  // Verify and advance ftp prefix
  if (strncmp(ftp_command, FTP_PREFIX, FTP_PREFIX_SIZE) != 0)
    abort_bad_url();
  ftp_command += FTP_PREFIX_SIZE * sizeof(char);

  // Read username
  int res = chop_string(ftp_command, user, ':');
  if (res == -1){
    *user = (char *)malloc(10 * sizeof(char));
    memcpy(*user, "anonymous", 9);
  }
  ftp_command += res + 1; 

  // Read password
  res = chop_string(ftp_command, password, '@');
  if (res == -1){
    *password = (char *)malloc(2 * sizeof(char));
    memcpy(*password, "", 1);
  }
  ftp_command += res + 1; 

  // Read server name
  res = chop_string(ftp_command, server, '/');
  if (res == -1)
    abort_bad_url();
  ftp_command += res + 1; 

  //Read file name
  *filename = basename(ftp_command);
  *filepath = dirname(ftp_command);
  
}

void cleanup(char *user, char* password, char *server, int socket_fd, int data_socket_fd){
  close(socket_fd);
  close(data_socket_fd);
   if (server != NULL)
    free(server);
   if (password != NULL)
    free(password);
   if (user != NULL)
    free(user);
}
