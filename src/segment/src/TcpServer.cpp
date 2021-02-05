#include "TcpServer.h"

namespace skywell{



TcpServer::TcpServer(skywell::Param *param){
	m_param = param;
};
TcpServer::~TcpServer(){};
int TcpServer::Init(const std::string &ip,int port)
{
	m_socket = socket(AF_INET,SOCK_STREAM,0);
	if (INVALID_SOCKET == m_socket)
	{
		return -1;
	}
	int ret = SetNonBlocking(m_socket);
	if (0 > ret)
	{
		return -1;
	}
	int optval = 1;
	ret = setsockopt(m_socket,SOL_SOCKET,SO_REUSEADDR,&optval,sizeof(optval));
	if (0 > ret)
	{
		return -1;
	}

	struct sockaddr_in addr;
	socklen_t addrlen = sizeof(addr);
	memset(&addr,0x00,addrlen);
    if(ip.empty() || ip == "INADDR_ANY")
    {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    else
    {
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
    }
    addr.sin_port = htons(port);

	ret = bind(m_socket,(const struct sockaddr*)&addr,addrlen);
	if (0 > ret)
	{
		return -1;
	}
	ret = listen(m_socket,0X10000);
	if (0 > ret)
	{
		return -1;
	}

	if (0 >= m_epoll)
	{
		m_epoll = epoll_create(0X10000);
		if (0 >= m_epoll)
		{
			return -1;
		}
	}

	struct epoll_event ev;
	memset(&ev, 0x00, sizeof(ev));
	ev.events = EPOLLIN | EPOLLHUP | EPOLLERR | EPOLLRDHUP | EPOLLET;
	ev.data.fd = m_socket;
	ret = epoll_ctl(m_epoll, EPOLL_CTL_ADD, m_socket, &ev);
	if(0 > ret)
	{
		return -1;
	}
	ThreadStart();
	return 0;
};

void TcpServer::UnInit(void)
{
	if (INVALID_FD != m_epoll)
	{
		close(m_epoll);
		m_epoll = INVALID_FD;
	}
	ThreadStop();
};

int TcpServer::Process(void)
{
	int client = -1;
	struct sockaddr_in addr;
	socklen_t addrlen = sizeof(addr);
	struct epoll_event evs[0x10000];
	int nfds = 0;
	memset(evs,0x00,sizeof(evs));
	nfds = epoll_wait(m_epoll, evs, 0x10000, 1000);
	if (nfds > 0)
	{
		for (int i = 0; i < nfds ; i++)
		{
			if (evs[i].data.fd == m_socket)
			{
				addrlen = sizeof(addr);
				memset(&addr, 0x00, addrlen);
				while ((client = accept(m_socket,(struct sockaddr *) &addr,&addrlen)) > 0)
				{
					AcceptClient(client,addr);
					memset(&addr,0x00,addrlen);
				}
			}else
			{
				if(evs[i].events & EPOLLRDHUP || evs[i].events & EPOLLERR || evs[i].events & EPOLLHUP)
				{
					DeleteClient(&evs[i]);
				}
				// 调测端数据接收
				int ret = RecvData(evs[i].data.fd);
				if (ret != 0)
				{
					DeleteClient(&evs[i]);
				}
			}
		}
	}
	return 0;
};




void TcpServer::AcceptClient(int client, const struct sockaddr_in& addr)
{
	int ret = SetNonBlocking(client);
	if (ret < 0)
	{
		close(client);
		return;
	}
	struct epoll_event ev;
	memset(&ev,0x00,sizeof(ev));
	ev.data.fd = client;
	ev.events = EPOLLIN | EPOLLET | EPOLLHUP | EPOLLERR | EPOLLRDHUP ;
	epoll_ctl(m_epoll, EPOLL_CTL_ADD, client, &ev);
}


void TcpServer::DeleteClient(const struct epoll_event* evs)
{
	struct epoll_event ev;
	memset(&ev,0x00,sizeof(ev));
	int client_socket = evs->data.fd;
	ev.data.fd = client_socket;
	epoll_ctl(m_epoll, EPOLL_CTL_DEL, client_socket, &ev);
	close(evs->data.fd);
}


int TcpServer::RecvData(int fd)
{
	int ret = 0;
	int len = 0;
	unsigned char headerbuf[8];
	len = recv(fd,headerbuf,8,0);
	if (len > 0)
	{
		if ((headerbuf[0] == '$') && (headerbuf[1] == '$'))
		{
			int cmd = (headerbuf[2]<< 8)|headerbuf[3];
			int datasize = (headerbuf[4] << 24)|(headerbuf[5] << 16)|(headerbuf[6] << 8)|headerbuf[7];
			unsigned char *databuf = (unsigned char *)malloc(datasize + 1);
			memset(databuf,0x00,datasize + 1);
			int recvsize = 0;
			int totalrecvsize = 0;
			while (totalrecvsize < datasize)
			{
				recvsize = recv(fd,&databuf[totalrecvsize],datasize-totalrecvsize,0);
				if (recvsize > 0)
				{
					totalrecvsize += recvsize;
				}else
				{
					if (recvsize == 0)
					{
						ret = -1;
						break;
					}
					if ((errno != EINTR) && (errno != EAGAIN) && (errno != EWOULDBLOCK))
					{
						ret = -1;
						break;
					}
					ret = 0;
				}
			}
			if (totalrecvsize == datasize)
			{
				DoWork(cmd,datasize,databuf);
			}
			free(databuf);
			databuf = NULL;
		}
	}
	return ret;
};

int TcpServer::WriteData(int fd, unsigned char *databuf, int datalen)
{
	int ret = 0;
	int pos;
	int len;
	if ((fd <= 0) || (datalen <= 0))
	{
		return -1;
	}
	pos = 0;
	while (1)
	{
		
		len = send(fd, &(databuf[pos]), datalen - pos, 0);
		if (len > 0)
		{
			pos = pos + len;
			if (pos >= datalen)
			{
				break;
			}
		}
		else
		{
			if (len == -1)
			{
				if ((errno != EINTR) && (errno != EAGAIN) && (errno != EWOULDBLOCK))
				{
					ret = -1;
					break;
				}
			}
			else
			{
				if (len == 0)
				{
					ret = -1;
					break;
				}
			}
		}
	}
	return ret;
};


int TcpServer::SendData(int cmd,int datalen,unsigned char *databuf)
{
	if (m_pc_fd > 0)
	{
		int ret = SendHead(m_pc_fd,cmd,datalen);
		if (ret > 0)
		{
			return WriteData(m_pc_fd,databuf,datalen);
		}
	}
	return -1;
};


int TcpServer::SendHead(int fd,int cmd,int datalen)
{
	if (fd > 0)
	{
		unsigned char headerbuf[8] = {0};
		headerbuf[0] = '$';
		headerbuf[1] = '$';
		headerbuf[2] = (cmd >> 8)&0xFF;
		headerbuf[3] = cmd & 0xFF;

		headerbuf[4] = (datalen >> 24)&0xFF;
		headerbuf[5] = (datalen >> 16)&0xFF;
		headerbuf[6] = (datalen >> 8)&0xFF;
		headerbuf[7] = datalen & 0xFF;
		return WriteData(fd,headerbuf,8);
	}
	return -1;

};





int TcpServer::DoWork(int cmd,int datalen,unsigned char *databuf)
{
	switch (cmd&0xFF)
	{
		case 0://设置聚类方式
			m_param->m_clustertype = databuf[0];
			break;
		case 1:
			break;
		case 2:
			break;
		default:
			break;
	}
	m_param->dumpyaml();
	return 0;
};

int TcpServer::SetNonBlocking(int fd)
{
	int t_opts;
	t_opts = fcntl(fd, F_GETFL);

	if (t_opts < 0)
	{
		return -1;
	}
	t_opts = t_opts | O_NONBLOCK;
	if (fcntl(fd, F_SETFL, t_opts) < 0)
	{
		return -1;
	}
	return 0;
};






}



