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


#define FTP_PORT 21

int tcp_connect(char *server){

    struct hostent *h = gethostbyname(server);;
    if (h == NULL)
        exit(-1);

    /*get ip address*/
    printf("Host name  : %s\n", h->h_name);
    char *ip_addr = inet_ntoa(*((struct in_addr *)h->h_addr));
    printf("IP Address : %s\n", ip_addr);

    /*server address handling*/
    struct	sockaddr_in server_addr;
	bzero((char*)&server_addr,sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(ip_addr);	/*32 bit Internet address network byte ordered*/
	server_addr.sin_port = htons(FTP_PORT);		/*server TCP port must be network byte ordered */

    /*open a TCP socket*/
    int socketfd = socket(AF_INET,SOCK_STREAM,0);
    if (socketfd < 0)
        exit(-1);

    /*connect to the server*/
    if(connect(socketfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0){
        perror("connect()");
		exit(-1);
	}

    return socketfd;
}
