#ifndef _CLIENT_H_
#define _CLIENT_H_


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <time.h>
#include <string.h>
#include <signal.h>

namespace skywell{
class Client
{
	public:
		Client(void);
		~Client(void);
	public:
		int ClientInit(void);
		int ClientFree(void);
	public:
		int State(void);
		int Connect(char *ip,int port);
		int DisConnect(void);
		int Send(unsigned char *buf,int len);
		int Recv(unsigned char *buf,int len);
	public:
		int m_fd;
};


}









#endif

