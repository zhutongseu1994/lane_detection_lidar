#ifndef _TCP_CLIENT_H_
#define _TCP_CLIENT_H_

#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <ifaddrs.h>
#include <linux/ioctl.h>
#include <netdb.h> // gethostbyname, gethostbyname2, gethostbyname_r, gethostbyname_r2
#include <string>
#include <vector>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <sys/time.h>
// #include "Std_Types.h"
namespace skywell
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

class CTcpClient
{
public:
    CTcpClient();
    virtual ~CTcpClient();

    sint32 init_tcp_client(void);                                  //供线程使用
    sint32 init_tcp_client(const sint8 *ip, uint32 port);          //供外部调用
    sint32 init_tcp_client_hostname(sint8 *hostname, uint32 port); //供外部调用
    sint32 start_connect_thread(void);

    void close_tcp(void);

    sint32 send_tcp_data(unsigned char *str, uint32 len);

    typedef void (*pFunc_ReadCallback)(sint8 *data, uint32 len);
    void tcp_recv_callback(sint8 *buf, uint32 len);
    void time_set(int year, int month, int day, int hour, int min, int sec, long usec);

    void set_read_callback_func(const pFunc_ReadCallback pReadFunc);

    sint32 get_tcp_status();
    sint32 get_last_error();

    uint32 m_connect_interval; //自动重连间隔时间，单位秒

private:
    static void *tcp_read_thread(void *arg);
    static void *tcp_connect_thread(void *arg);
    sint8 get_local_ip();

    sint32 m_tcp_sock_client;
    struct sockaddr_in socket_server_addr;

    sint32 m_last_error;

    pthread_t tid_recv; //接受数据线程的Id
    pthread_t tid_conn; //连接线程的ID

    sint8 m_status;           //是否需要连接
    sint8 m_connected_status; //连接状态

    sint32 m_thread_read_avtived;

    sint8 rev_buf[BUFF_LEN];

    sint8 m_ip_address[31];
    sint8 m_hostname[101];
    uint32 m_port;

    pFunc_ReadCallback m_pReadCallback;
};

} // namespace skywell
#endif