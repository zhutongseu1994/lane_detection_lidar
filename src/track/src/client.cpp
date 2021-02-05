
#include "client.h"
namespace skywell{

Client::Client(void)
{
    ClientInit();
};

Client::~Client(void)
{
    ClientFree();
};

int Client::ClientInit()
{
    m_fd = 0;
    return 0;
};

int Client::ClientFree(void)
{
    m_fd = 0;
    return 0;
};

int Client::State(void)
{
    if (m_fd > 0)
    {
        return 0;
    }
    return -1;
};

int Client::Connect(char *ip,int port)
{
    struct sockaddr_in serv_addr;
    if (m_fd > 0)
    {
        close(m_fd);
        m_fd = 0;
    }
    signal(SIGPIPE, SIG_IGN);
    serv_addr.sin_family=AF_INET;
    serv_addr.sin_port=htons(port);
    inet_aton(ip, &serv_addr.sin_addr);
    bzero(&(serv_addr.sin_zero), 8);
    m_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_fd > 0)
    {
        if (connect(m_fd, (struct sockaddr *)&serv_addr, sizeof(struct sockaddr)) == 0)
        {
            return 0;
        }
        else
        {
           DisConnect();
        }
    }
    return -1;
};


int Client::DisConnect(void)
{
    if (m_fd > 0)
    {
        close(m_fd);
        m_fd = 0;
    }
    return 0;
};


 int Client::Send(unsigned char *buf,int len)
{
    int _rs = 0;
    int _pos = 0;
    int _len = 0;
    if (m_fd > 0)
    {
        _pos = 0;
		signal(SIGPIPE, SIG_IGN);
        while (_pos < len)
        {
            _len = len - _pos;
            _rs = send(m_fd, &buf[_pos], _len, 0);
            if (_rs > 0)
            {
                _pos = _pos + _rs;
            }
            else
            {
                DisConnect();//断开连接
                return -1;
            }
        }
        return 0;
    }
    return -1;
};


/*
解阻塞与非阻塞recv返回值没有区分，都是 
<0 出错
=0 连接关闭
>0 接收到数据大小，
特别：返回值<0时并且(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN)的情况下认为连接是正常的，继续接收。
只是阻塞模式下recv会阻塞着接收数据，非阻塞模式下如果没有数据会返回，不会阻塞着读，因此需要循环读取）。
*/


int Client::Recv(unsigned char *buf,int len)
{
    int _rs;
    struct timeval _timeout;
    fd_set _readset;
    if (m_fd <= 0)
    {
        return -1;
    }
    FD_ZERO(&_readset);
    FD_SET(m_fd, &_readset);
    _timeout.tv_sec = 0;
    _timeout.tv_usec = 500000;
    _rs = select(m_fd + 1, &_readset, NULL, NULL, &_timeout);
    if (_rs > 0)
    {
        if (FD_ISSET(m_fd, &_readset))
        {
            int _len = recv(m_fd, buf, len,0);
            if (_len > 0)
            {
                return _len;
            }
            else
            {
				if (_len == 0)
				{
					return -1;
				}
                if ((errno != EINTR) && (errno != EAGAIN) && (errno != EWOULDBLOCK))
                {
                    return -1;
                }
            }
        }
    }
    return 0;
};

}

