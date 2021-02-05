#ifndef TCP_SERVER_H_
#define TCP_SERVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <pthread.h>


#include "param.h"
#include "thread.h"



#define INVALID_SOCKET (-1)
#define INVALID_FD (-1)


namespace skywell{


class TcpServer:public Thread
{
	public:
		TcpServer(skywell::Param *param);
		~TcpServer();
	public:
		int Init(const std::string &ip,int port);
		void UnInit(void);
		virtual int Process(void);
	private:
		void AcceptClient(int client, const struct sockaddr_in& addr);
		void DeleteClient(const struct epoll_event* evs); 
		int SetNonBlocking(int fd);
		int RecvData(int fd);
		int SendHead(int fd,int cmd,int datalen);
		int SendData(int cmd,int datalen,unsigned char *databuf);
		int DoWork(int cmd,int datalen,unsigned char *databuf);
		int WriteData(int fd, unsigned char *databuf, int datalen);
	private:
		int m_pc_fd{INVALID_SOCKET};
		int m_socket{INVALID_SOCKET};
		int m_epoll{INVALID_FD};
	private:
		skywell::Param *m_param;
};




}



#endif
