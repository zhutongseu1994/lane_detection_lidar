#pragma once
#ifndef _TCP_SERVER_H_
#define _TCP_SERVER_H_

#include <string>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <map>
#include <sys/stat.h>
#include <ifaddrs.h>
#include <linux/ioctl.h>
#include <netdb.h> // gethostbyname, gethostbyname2, gethostbyname_r, gethostbyname_r2
#include <errno.h>

namespace Skywell
{
    typedef char sint8;
    typedef unsigned char uint8;
    typedef short sint16;
    typedef unsigned short uint16;
    typedef int sint32;
    typedef unsigned int uint32;

    typedef unsigned long uint8_least;
    typedef unsigned long uint16_least;
    typedef unsigned long uint32_least;

    typedef signed long sint8_least;
    typedef signed long sint16_least;
    typedef signed long sint32_least;

    typedef float float32;
    typedef double float64;

#ifndef NULL
#define NULL (0)
#endif

#ifndef NULL_PTR
#define NULL_PTR ((void *)0)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define BUFF_LEN 2048
/*****************************************************************************
*                                头文件引用                                  *
*****************************************************************************/

/*****************************************************************************
*                                宏定义                                  *
*****************************************************************************/
#define BUFF_LEN 2048
#define MAX_LISTEN_COUNT 10
#define MAX_EPOLL_EVENTS MAX_LISTEN_COUNT + 1

    using namespace std; //for map

    typedef std::map<int, string> Map;
    typedef Map::iterator MapIt;

    /*****************************************************************************
*                                类定义                                  *
*****************************************************************************/
    class CTcpServer
    {
    public:
        CTcpServer();
        virtual ~CTcpServer();

        sint32 init_tcp_server(const sint8 *ip, uint32 port); //供外部调用
        void close_server(void);

        sint32 send_client_data(uint8 *str, uint32 len);

        typedef void (*pFunc_ReadCallback)(uint8 *data, uint32 len);

        void set_read_callback_func(const pFunc_ReadCallback pReadFunc);
        sint32 get_tcp_server_status();

        uint32 m_connect_interval; //自动重连间隔时间，单位秒

    private:
        sint32 init_tcp_server(void); //供线程使用
        void close_tcp(void);
        sint32 start_connect_thread(void);

        static void *tcp_server_listen_thread(void *arg);
        static void *tcp_connect_thread(void *arg);
        void set_nonblocking(int sock);
        sint8 get_local_ip();
        void add_client(sint32 fd, string ip);
        void delete_client(sint32 fd);
        // int check_fd_fine(int fd);
        sint32 m_epfd; //epoll fd
        Map m_clients; //MAP管理客户端

        sint32 m_tcp_sock_server;
        struct sockaddr_in socket_server_addr;

        pthread_t tid_listen;  //接受数据线程的Id
        pthread_t tid_connect; //连接线程的Id
        pthread_mutex_t mutex;

        sint8 m_status;        //是否需要连接
        sint8 m_listen_status; //连接状态

        sint32 m_thread_listen_avtived; //线程退出开关

        sint8 m_ip_address[31];
        sint8 m_hostname[101];
        uint32 m_port;

        pFunc_ReadCallback m_pReadCallback;
    };

};     // namespace Skywell
#endif //_TCP_SERVER_H_