/*****************************************************************************/
/** 
* \file       TcpServer.cpp
* \author     张海军
* \date       2020/01/10
* \version    1.0
* \brief      文件描述，TCP Server类实现
* \note       Copyright (c) 2010-2030  金龙客车制造有限公司
* \remarks    无
******************************************************************************/

/*****************************************************************************
*                               头文件引用                                   *
*****************************************************************************/

#include "tcp_server.h"

namespace Skywell
{

    /*****************************************************************************
*                                局部宏定义                                  *
*****************************************************************************/

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       类构造函数
* \param[in]   none
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    CTcpServer::CTcpServer()
    {
        m_tcp_sock_server = -1;
        m_status = 0;
        m_listen_status = 0;
        m_thread_listen_avtived = 0;
        m_connect_interval = 3;
        m_pReadCallback = NULL;

        m_epfd = epoll_create(MAX_EPOLL_EVENTS);
        if (m_epfd == -1)
        {
            printf("epoll create error, fatal error!\n");
        }

        pthread_mutex_init(&mutex, NULL);
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       类析构函数
* \param[in]   none
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    CTcpServer::~CTcpServer()
    {
        close_server();

        if (m_epfd != -1)
            close(m_epfd);

        pthread_mutex_destroy(&mutex);
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       获取本机联网状态
* \param[in]   none
* \return      0:正常，负数:联网不正常
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    sint8 CTcpServer::get_local_ip()
    {
        struct ifaddrs *ifaddr, *ifa;
        int family, s;

        if (getifaddrs(&ifaddr) == -1)
        {
            return -1;
        }
        int ret_value = -2;
        /* Walk through linked list, maintaining head pointer so we can free list later */
        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == NULL)
                continue;

            family = ifa->ifa_addr->sa_family;

            if (strcmp(ifa->ifa_name, "enos1") == 0) //检测eth0的类型
            {
                if (family == AF_INET)
                {
                    ret_value = 0;
                    break;
                }
            }
        }
        freeifaddrs(ifaddr);

        return ret_value;
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       设置SOCKET为非阻塞模式
* \param[in]   sock     SOCKET的文件描述符
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    void CTcpServer::set_nonblocking(int sock)
    {
        int opts;
        opts = fcntl(sock, F_GETFL);
        if (opts < 0)
        {
            printf("fcntl(sock,GETFL) error");
            return;
        }
        opts = opts | O_NONBLOCK;
        if (fcntl(sock, F_SETFL, opts) < 0)
        {
            printf("fcntl(sock,SETFL,opts) error");
            return;
        }
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       初始化TCP服务端
* \param[in]   ip   IP地址，IPV4格式
* \param[in]   port   端口号
* \return      0:初始化成功，非0:初始化失败
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    sint32 CTcpServer::init_tcp_server(const sint8 *ip, uint32 port)
    {
        int len;

        if (m_epfd == -1) //epoll 没创建成功
            return -1;

        len = strlen(ip);
        if (len > 30)
            return -2;
        if (port < 1)
            return -3;

        if (m_status == 1) //已经初始化过了
        {
            //比较与当前IP和PROT是否一致，如果一致，则直接退出
            if ((strcmp(m_ip_address, ip) == 0) && (port == m_port))
                return 0;
        }

        //复制IP和PORT
        memset(m_ip_address, 0, sizeof(m_ip_address)); //IP清零
        memcpy(m_ip_address, ip, len);
        m_port = port;

        if (m_status == 1) //已经初始化过了
        {
            if (m_listen_status == 1) //如果已经处于监听状态，则关闭当前监听
            {
                close_tcp();
            }
            else //目前连接线程正在连接，就不必再起连接线程了
            {
                return 0;
            }
        }
        else
        {
            m_status = 1;
        }

        return start_connect_thread();
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       实际创建TCP Server, 供线程调用
* \param[in]   none
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    sint32 CTcpServer::init_tcp_server(void)
    {
        sint32 res;
        sint32 opt;

        if (m_listen_status)
            close_tcp();

        // if (get_local_ip() != 0)
        // {
        //     printf(" server RJ45 failure\n");
        //     return -1;
        // }

        /* create a socket */
        m_tcp_sock_server = socket(AF_INET, SOCK_STREAM, 0);
        if (m_tcp_sock_server == -1)
        {
            return -2;
        }

        opt = 1;
        setsockopt(m_tcp_sock_server, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt)); //port reuse address

        /* bind IP */
        memset(&socket_server_addr, 0, sizeof(struct sockaddr_in));
        socket_server_addr.sin_family = AF_INET;
        socket_server_addr.sin_port = htons(m_port);
        socket_server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(m_tcp_sock_server, (struct sockaddr *)&socket_server_addr, sizeof(struct sockaddr_in)) == -1)
        { //bind error
            close(m_tcp_sock_server);
            return -3;
        }

        if (listen(m_tcp_sock_server, MAX_LISTEN_COUNT) == -1)
        {
            close(m_tcp_sock_server);
            return -4;
        }

        struct epoll_event event;
        //设置与要处理的事件相关的文件描述符
        event.data.fd = m_tcp_sock_server;
        //设置要处理的事件类型
        event.events = EPOLLIN | EPOLLET;
        //注册epoll事件
        res = epoll_ctl(m_epfd, EPOLL_CTL_ADD, m_tcp_sock_server, &event);
        if (res == -1)
        {
            printf("epoll_ctl error!");
            close(m_tcp_sock_server);
            return -1;
        }

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        res = pthread_create(&tid_listen, &attr, tcp_server_listen_thread, (void *)this);
        pthread_attr_destroy(&attr);
        if (res != 0)
        {
            close(m_tcp_sock_server);
            return -5;
        }

        m_thread_listen_avtived = 1;
        m_listen_status = 1;

        return 0;
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       创建连接线程
* \param[in]   none
* \return      线程创建是否成功，0:成功，-1:失败
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    sint32 CTcpServer::start_connect_thread(void)
    {
        uint32 res;

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        res = pthread_create(&tid_connect, &attr, tcp_connect_thread, (void *)this);
        pthread_attr_destroy(&attr);
        if (res != 0)
        {
            return -1;
        }

        return 0;
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       关闭服务端
* \param[in]   none
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    void CTcpServer::close_server(void)
    {
        close_tcp();
        m_status = 0;
    }

    // int CTcpServer::check_fd_fine(int fd)
    // {
    //     struct stat _stat;
    //     int ret = -1;
    //     if (!fcntl(fd, F_GETFL))
    //     {
    //         if (!fstat(fd, &_stat))
    //         {
    //             if (_stat.st_nlink >= 1)
    //                 ret = 0;
    //             //else
    //             //    printf("File was deleted!\n");
    //         }
    //     }
    //     //if (errno != 0)
    //     //    perror("check_fd_fine");
    //     return ret;
    // }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       关闭SOCKET，删除EPOLL监控，删除客户端
* \param[in]   none
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    void CTcpServer::close_tcp(void)
    {
        int fd;
        struct epoll_event ev;

        if (m_listen_status)
        {
            //删除epoll监控
            ev.data.fd = m_tcp_sock_server;
            epoll_ctl(m_epfd, EPOLL_CTL_DEL, m_tcp_sock_server, &ev);

            m_thread_listen_avtived = 0; //退出接收线程

            //关闭客户端
            close(m_tcp_sock_server);
            m_tcp_sock_server = -1;
            m_listen_status = 0;
        }

        if (!m_clients.empty())
        {
            for (MapIt it = m_clients.begin(); it != m_clients.end(); ++it)
            {
                fd = it->first; //get map key

                //删除epoll中的监控
                ev.data.fd = fd;
                epoll_ctl(m_epfd, EPOLL_CTL_DEL, fd, &ev);

                //关闭SOCKET
                // if (check_fd_fine(fd) == 0)
                close(fd);
            }

            pthread_mutex_lock(&mutex);
            m_clients.clear(); //清除MAP中所有Key
            pthread_mutex_unlock(&mutex);
        }
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       根据文件描述符删除客户端，同时删除EPOLL监控
* \param[in]   fd   SOCKET描述符
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    void CTcpServer::delete_client(sint32 fd)
    {
        struct epoll_event ev;
        MapIt it;
        int num;

        //删除epoll监控
        ev.data.fd = fd;
        epoll_ctl(m_epfd, EPOLL_CTL_DEL, fd, &ev);

        // if (check_fd_fine(fd) == 0)
        close(fd);

        //MAP中删除客户端fd
        it = m_clients.find(fd);

        pthread_mutex_lock(&mutex);
        m_clients.erase(it);
        pthread_mutex_unlock(&mutex);

        printf("new client num is %d!\n", (int)m_clients.size());
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       增加客户端EPOLL监控，同时保存客户端信息
* \param[in]   fd   SOCKET描述符
* \param[in]   IP   客户端的IP地址
* \return      none
* \ingroup     TcpServer
* \remarks     其它信息
******************************************************************************/
    void CTcpServer::add_client(sint32 fd, string ip)
    {
        struct epoll_event ev;
        MapIt it;

        //设置用于读操作的文件描述符
        ev.data.fd = fd;
        //设置用于注测的读操作事件
        ev.events = EPOLLIN | EPOLLET;
        //ev.events=EPOLLIN;
        //注册ev
        epoll_ctl(m_epfd, EPOLL_CTL_ADD, fd, &ev);

        pthread_mutex_lock(&mutex);
        //MAP中增加客户端
        m_clients.insert(make_pair(fd, ip)); //可考虑增加IP
        pthread_mutex_unlock(&mutex);

        printf("client left num is %d!\n", (int)m_clients.size());
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       TCP SERVER LISTEN监听线程函数
* \param[in]   arg  TcpServer类地址指针
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    void *CTcpServer::tcp_server_listen_thread(void *arg)
    {
        //printf("------------start tcp server listen thread--------------\n");
        CTcpServer *p_tcp = (CTcpServer *)arg;

        sint32 client_sock = -1;
        sint32 nfds, len;

        struct sockaddr_in client_addr;
        socklen_t clilen = sizeof(client_addr);

        uint8 recv_buf[BUFF_LEN];

        //生成用于处理accept的epoll专用的文件描述符
        struct epoll_event events[MAX_EPOLL_EVENTS];

        while (1)
        {
            //等待epoll事件的发生
            nfds = epoll_wait(p_tcp->m_epfd, events, MAX_EPOLL_EVENTS, 500);

            //处理所发生的所有事件
            for (uint32 i = 0; i < nfds; ++i)
            {
                if (events[i].data.fd == p_tcp->m_tcp_sock_server) //如果新监测到一个SOCKET用户连接到了绑定的SOCKET端口，建立新的连接。
                {
                    client_sock = accept(p_tcp->m_tcp_sock_server, (sockaddr *)&client_addr, &clilen);
                    if (client_sock < 0)
                    {
                        printf("tcp server accept error!");
                        p_tcp->close_tcp();
                        p_tcp->start_connect_thread();
                        return (void *)-1; //退出线程
                    }

                    fcntl(client_sock, F_SETFL, fcntl(client_sock, F_GETFL, 0) | O_NONBLOCK); //设置为非阻塞模式

                    char *str = inet_ntoa(client_addr.sin_addr); //IP
                    string str_ip = str;
                    printf("accapt a connection from %s\n", str);
                    p_tcp->add_client(client_sock, str_ip);
                }
                else if (events[i].events & EPOLLIN) //如果是已经连接的用户，并且收到数据，那么进行读入。
                {
                    //printf("EPOLL DATA IN\n");
                    if ((client_sock = events[i].data.fd) < 0)
                        continue;
                    if ((len = read(client_sock, recv_buf, BUFF_LEN)) < 0) //网络断后几秒会触发
                    {
                        //if (errno == ECONNRESET)
                        {
                            printf("client delete in thread1!\n");
                            p_tcp->delete_client(client_sock);
                        }
                    }
                    else if (len == 0) //socket client主动关闭会触发
                    {
                        printf("client delete in thread2!\n");
                        p_tcp->delete_client(client_sock);
                    }

                    if ((p_tcp->m_pReadCallback != NULL) && (len > 0))
                        p_tcp->m_pReadCallback(recv_buf, len);

                    /*  epoll 不监控写事件
                //设置用于写操作的文件描述符
                ev.data.fd = sockfd;
                //设置用于注测的写操作事件
                ev.events = EPOLLOUT | EPOLLET;
                //修改sockfd上要处理的事件为EPOLLOUT
                //epoll_ctl(epfd,EPOLL_CTL_MOD,sockfd,&ev);
                */
                }
                /*  epoll 不监控写事件
            else if (events[i].events & EPOLLOUT) // 如果有数据发送
            {
                if ((sockfd = events[i].data.fd) < 0)
                    continue;
                write(sockfd, line, n);
                //设置用于读操作的文件描述符
                ev.data.fd = sockfd;
                //设置用于注测的读操作事件
                ev.events = EPOLLIN | EPOLLET;
                //写完后修改sockfd上要处理的事件为EPOLIN
                epoll_ctl(epfd, EPOLL_CTL_MOD, sockfd, &ev);
            }*/
            } //for (uint32 i = 0; i < nfds; ++i)

            if (p_tcp->m_thread_listen_avtived == 0)
                break;
        }
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       TCP SERVER连接线程函数
* \param[in]   arg  TcpServer类地址
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    void *CTcpServer::tcp_connect_thread(void *arg)
    {
        //printf("------------start tcp server connect Thread--------------\n");
        CTcpServer *p_tcp = (CTcpServer *)arg;

        while (1)
        {
            if (p_tcp->init_tcp_server() == 0) //起TCP服务成功
            {
                printf("tcp_server started successful!\n");
                break;
            }
            else
            {
                printf("tcp_server started error!\n");
            }

            if (p_tcp->m_status == 0)
                break;                        //关闭SERVER
            sleep(p_tcp->m_connect_interval); //休眠时间单位是秒
        }
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       设置接收处理的回调函数
* \param[in]   pReadFunc  接收处理回调函数的函数指针
* \return      none
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    void CTcpServer::set_read_callback_func(const pFunc_ReadCallback pReadFunc)
    {
        m_pReadCallback = pReadFunc;
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       获取TCP Server的连接状态
* \param[in]   无
* \return      SOCKET的监听状态
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    sint32 CTcpServer::get_tcp_server_status()
    {
        return m_listen_status;
    }

    /*****************************************************************************/
    /** 
* \author      zhanghaijun
* \date        2020/01/10
* \brief       向所有连接发送数据
* \param[in]   str  发送数据内容的起始地址
* \param[in]   len  数据的长度，以字节为单位
* \return      返回0
* \ingroup     TcpServer
* \remarks     无
******************************************************************************/
    sint32 CTcpServer::send_client_data(uint8 *str, uint32 len)
    {
        int client_fd, ret;

        if (!m_clients.empty())
        {
            for (MapIt it = m_clients.begin(); it != m_clients.end(); ++it)
            {
                client_fd = it->first; //get map key

                //ret = send(client_fd, str, len, 0);
                ret = send(client_fd, str, len, MSG_DONTWAIT);
                if (ret <= 0)
                {
                    printf("client delete in send!\n");
                    delete_client(client_fd);
                }
            }
        }

        return 0;
    }

}; // namespace Skywell