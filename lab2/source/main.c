#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#define FTP_COMMAND_MIN_SIZE 12
#define FTP_PREFIX "ftp://"
#define FTP_PREFIX_SIZE 6

// "ftp://test:pw@host.com/file.f"

int chop_string(char *string, char **result, char delim);
void abort_bad_url();

int main(int argc, char *argv[]) {

  if (argc < 2){
    return -1;
  }

  char * ftp_command = argv[1];

  if (strlen(ftp_command) < FTP_COMMAND_MIN_SIZE)
    abort_bad_url();

  // Verify and advance ftp prefix
  if (strncmp(ftp_command, FTP_PREFIX, FTP_PREFIX_SIZE) != 0)
    abort_bad_url();
  ftp_command += FTP_PREFIX_SIZE * sizeof(char);

  // Read username
  char *user = NULL;
  int res = chop_string(ftp_command, &user, ':');
  if (res == -1)
    abort_bad_url();
  ftp_command += res + 1; 

  // Read password
  char *password = NULL;
  res = chop_string(ftp_command, &password, '@');
  if (res == -1)
    abort_bad_url();
  ftp_command += res + 1; 

  // Read server name
  char *server = NULL;
  res = chop_string(ftp_command, &server, '/');
  if (res == -1)
    abort_bad_url();
  ftp_command += res + 1; 

  //Read file name
  char *filename = ftp_command;


  free(server);
  free(password);
  free(user);
  return 0;
}

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

