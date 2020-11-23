#include "ftp_app.h"

#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <netdb.h> 
#include <strings.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#define FTP_PORT_NUMBER 21
#define FTP_RETURN_CODE_SIZE 4
#define FTP_CODE_CONFIRM "2"
#define FTP_CODE_NEED_PASSWORD "331"
#define FTP_CODE_LOGIN_SUCCESS "230"
#define FTP_CODE_PASSIVE_MODE "227"
 
bool read_return_code(int socket_fd, char expected_return[]){

    char code[FTP_RETURN_CODE_SIZE];
    
    if (expected_return == NULL)
        return false;

    do {
        memset(&code, 0, FTP_RETURN_CODE_SIZE);
        read(socket_fd, &code, FTP_RETURN_CODE_SIZE);
    } while (code[3] != ' ' || code[0] < '1' || code[0] > '5');

    printf("received return code: %s\n", code);

    return !strncmp(code, expected_return, strlen(expected_return));
}

bool send_command(int socket_fd, char *command, char* expected_return_code){

    write(socket_fd, command, strlen(command));
    
    if (expected_return_code != NULL)
        return read_return_code(socket_fd, expected_return_code);

    return true;
}

int ftp_connect(char *server){

    struct hostent *h = gethostbyname(server);;
    if (h == NULL)
        exit(-1);

    /*get ip address*/
    printf("\nHost name  : %s\n", h->h_name);
    char *ip_addr = inet_ntoa(*((struct in_addr *)h->h_addr));

    int socket_fd = ftp_connect_socket(ip_addr, FTP_PORT_NUMBER);

    if (!read_return_code(socket_fd, FTP_CODE_CONFIRM)){
        fprintf(stderr, "unexpected FTP return code\n");
        exit(-1);
    }
    
    return socket_fd;
}

int ftp_connect_socket(char *ip, int port){
    printf("IP Address : %s\n", ip);
    printf("connecting...\n");
    
    /*server address handling*/
    struct	sockaddr_in server_addr;
    bzero((char*)&server_addr,sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(ip);	/*32 bit Internet address network byte ordered*/
    server_addr.sin_port = htons(port);		/*server TCP port must be network byte ordered */

    /*open a TCP socket*/
    int socketfd = socket(AF_INET,SOCK_STREAM,0);
    if (socketfd < 0)
        exit(-1);

    /*connect to the server*/
    if(connect(socketfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){
        perror("connect()");
		exit(-1);
	}

    printf("connected.\n");

    return socketfd;
}

void ftp_login(int socket_fd, char * username, char* password){

if (username == NULL || password == NULL)
    exit(-1);

printf("\nloging in as user %s\n", username);

char usr_cmd[strlen(username) + 8];
sprintf(usr_cmd, "USER %s\r\n", username);

if (!send_command(socket_fd, usr_cmd, FTP_CODE_NEED_PASSWORD)){
    fprintf(stderr, "login failed.\n");
    exit(-1);
}

char pass_cmd[strlen(password) + 8];
sprintf(pass_cmd, "PASS %s\r\n", password);

if (!send_command(socket_fd, pass_cmd, FTP_CODE_LOGIN_SUCCESS)){
    fprintf(stderr, "login failed.\n");
    exit(-1);
}

printf("login sucessful\n");
}

void ftp_enter_passive_mode(int socket_fd, char *client_ip, int *client_port){
    printf("\nentering passive mode\n");

    if (client_ip == NULL){
        fprintf(stderr, "failed to enter passive mode\n");
        exit(-1);
    }

    send_command(socket_fd, "PASV\r\n", NULL);

    
    
    char return_code[256];
    do {
        memset(&return_code, 0, 256);
        read(socket_fd, &return_code, 256);
    } while (return_code[3] != ' ' || return_code[0] < '1' || return_code[0] > '5');


    if (strncmp(return_code, FTP_CODE_PASSIVE_MODE, strlen(FTP_CODE_PASSIVE_MODE)) != 0){
        fprintf(stderr, "failed to enter passive mode\n");
        printf("received passive mode data: %s\n", return_code);
        exit(-1);
    }

	char* client_info = strchr(return_code, '(');
    int ip_values[6];
	sscanf(client_info, "(%d, %d, %d, %d, %d, %d)", &ip_values[0],&ip_values[1],&ip_values[2],&ip_values[3],&ip_values[4],&ip_values[5]);
	sprintf(client_ip, "%d.%d.%d.%d", ip_values[0],ip_values[1],ip_values[2],ip_values[3]);
	*client_port = ip_values[4]*256+ip_values[5];
    
    printf("entered passive mode: %s:%d\n", client_ip, *client_port);
}
