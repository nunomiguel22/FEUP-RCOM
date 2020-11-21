#ifndef UTILS_H
#define UTILS_H


void abort_bad_url();
int chop_string(char *string, char **result, char delim);
void parseArguments(int argc, char *argv[], char **user, char** password, char **server, char **filepath, char **filename);
void cleanup(char *user, char* password, char *server, int socket_fd);

#endif


