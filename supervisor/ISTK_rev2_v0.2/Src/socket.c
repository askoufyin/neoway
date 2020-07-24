#include "socket.h"
#include <socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include "rtl.h"
//#include <netdb.h>

//struct in_addr {
//    unsigned long s_addr;  // load with inet_aton()
//};

//struct sockaddr_in {
//    short            sin_family;   // e.g. AF_INET
//    unsigned short   sin_port;     // e.g. htons(3490)
//    struct in_addr   sin_addr;     // see struct in_addr, below
//    char             sin_zero[8];  // zero this if you want to
//};

void foo(void)
{
	int listenfd = 0, connfd = 0;
	struct sockaddr_in serv_addr;
	
	char send_buff[1025];
	time_t ticks;
	
	listenfd = socket(AF_INET, SOCK_STREAM, 0);
}

