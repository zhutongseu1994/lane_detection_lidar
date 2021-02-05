
#include "tcp_client.h"

namespace skywell
{
    
CTcpClient::CTcpClient()
{
    m_tcp_sock_client = -1;
    m_last_error = 0;
    m_status = 0;
    m_connected_status = 0;
    m_thread_read_avtived = 0;
    m_connect_interval = 1;
    m_pReadCallback = NULL;
    tid_recv = -1;
    tid_conn = -1;
}

CTcpClient::~CTcpClient()
{
    close_tcp();
}

void CTcpClient::tcp_recv_callback(sint8 *buf, uint32 len)
{

    static int init_sys_time = 0;
    int sys_year;
    int sys_month;
    int sys_day;
    int sys_hour;
    int sys_min;
    int sys_sec;
    long sys_usec;
    init_sys_time++;
    if (init_sys_time > 10)
    {
        init_sys_time = 10;
    }
    if (0x33 == buf[0] && 9 == len)
    {
        sys_year = buf[1];
        sys_year = sys_year + 2000;
        sys_month = buf[2];
        sys_day = buf[3];
        sys_hour = buf[4];
        sys_min = buf[5];
        sys_sec = buf[6];
        long temp = 0;
        temp = buf[8] << 8;
        temp = (buf[7] + temp) * 1000;
        sys_usec = temp;
        if (init_sys_time < 3)
        {
            time_set(sys_year, sys_month, sys_day, sys_hour, sys_min, sys_sec, (long)sys_usec);
        }
    }
}

void CTcpClient::time_set(int year, int month, int day, int hour, int min, int sec, long usec)
{
    struct tm tptr;
    struct timeval tv;

    tptr.tm_year = year - 1900;
    tptr.tm_mon = month - 1;
    tptr.tm_mday = day;
    tptr.tm_hour = hour;
    tptr.tm_min = min;
    tptr.tm_sec = sec;

    tv.tv_sec = mktime(&tptr);
    tv.tv_usec = usec;
    int ret_set_tm = settimeofday(&tv, NULL);
    if (-1 == ret_set_tm)
    {
        printf("time set error!!!");
    }
}

sint32 CTcpClient::init_tcp_client_hostname(sint8 *hostname, uint32 port)
{
    int len;

    memset(m_hostname, 0, sizeof(m_hostname)); //域名清零
    len = strlen(hostname);
    if (len > 100)
        len = 100;
    memcpy(m_hostname, hostname, len);

    m_port = port;

    m_status = 1;

    return start_connect_thread();
}

sint8 CTcpClient::get_local_ip()
{
    struct ifaddrs *ifaddr, *ifa;
    int family, s;

    if (getifaddrs(&ifaddr) == -1)
    {
        return -1;
    }
    int ret_value = -2;
    /* Walk through linked list, maintaining head pointer so we
    *              can free list later */
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        if (strcmp(ifa->ifa_name, "eno1") == 0) //检测网卡的类型，随着机器改变
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

sint32 CTcpClient::init_tcp_client(const sint8 *ip, uint32 port)
{
    int len;

    memset(m_hostname, 0, sizeof(m_hostname));     //域名清零
    memset(m_ip_address, 0, sizeof(m_ip_address)); //IP清零
    len = strlen(ip);
    if (len > 30)
        len = 30;
    memcpy(m_ip_address, ip, len);
    m_port = port;

    m_status = 1;

    return start_connect_thread();
}

sint32 CTcpClient::init_tcp_client(void)
{
    sint32 res;

    if (m_connected_status) //实际连接状态
    {
        close(m_tcp_sock_client);
        m_tcp_sock_client = -1;
        m_connected_status = 0;
    }

    // if (get_local_ip() != 0)
    // {
    //     printf("RJ45 failure\n");
    //     return -1;
    // }

    //检查是否是域名模式，如果是则先进行域名转换
    if (strlen(m_hostname) > 0)
    {
        struct hostent *h;
        if ((h = gethostbyname(m_hostname)) == NULL)
        { // get the host info error,域名错误或网络有问题
            return -2;
        }
        const char *hostip = inet_ntoa(*((struct in_addr *)h->h_addr));
        memset(m_ip_address, 0, sizeof(m_ip_address)); //IP清零
        memcpy(m_ip_address, hostip, strlen(hostip));
    }

    m_tcp_sock_client = socket(AF_INET, SOCK_STREAM, 0);
    if (m_tcp_sock_client == -1)
    {
        m_last_error = -1;
        return -3;
    }

    memset(&socket_server_addr, 0, sizeof(socket_server_addr));
    socket_server_addr.sin_family = AF_INET;
    socket_server_addr.sin_port = htons(m_port); /* host to net, short */
    socket_server_addr.sin_addr.s_addr = inet_addr(m_ip_address);
    if (0 == inet_aton(m_ip_address, &socket_server_addr.sin_addr))
    { //IP address error
        m_last_error = -2;
        close(m_tcp_sock_client);
        return -4;
    }

    if (connect(m_tcp_sock_client, (struct sockaddr *)&socket_server_addr, sizeof(socket_server_addr)) < 0)
    {
        m_last_error = -3;
        close(m_tcp_sock_client);
        return -5;
    }

    if (tid_recv = -1) //没起接收线程
    {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        res = pthread_create(&tid_recv, &attr, tcp_read_thread, (void *)this);
        pthread_attr_destroy(&attr);
        if (res != 0)
        {
            tid_recv = -1;
            m_last_error = -3;
            close(m_tcp_sock_client);
            return -6;
        }
        m_thread_read_avtived = 1;
    }

    m_connected_status = 1;
    m_last_error = 0;

    return 0;
}

sint32 CTcpClient::start_connect_thread(void)
{
    uint32 res;

    if (tid_conn == -1) //连接线程已经退出
    {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        res = pthread_create(&tid_conn, &attr, tcp_connect_thread, (void *)this);
        pthread_attr_destroy(&attr);
        if (res != 0)
        {
            m_last_error = -1;
            return -1;
        }
    }
    return 0;
}

void CTcpClient::close_tcp(void)
{
    if (m_connected_status)
    {
        m_thread_read_avtived = 0;
        close(m_tcp_sock_client);
        m_tcp_sock_client = -1;
        m_connected_status = 0;
    }
    m_status = 0;
}

void *CTcpClient::tcp_read_thread(void *arg)
{
    //SKY_TRACE_INFO("------------start Udp Read Thread--------------\n");
    CTcpClient *p_tcp = (CTcpClient *)arg;

    struct sockaddr src_addr;
    uint32 addr_len;
    sint32 res;

    while (1)
    {
        if (p_tcp->m_tcp_sock_client != -1)
        {
            res = recv(p_tcp->m_tcp_sock_client, p_tcp->rev_buf, BUFF_LEN, 0);
            if (res <= 0)
            {
                p_tcp->m_last_error = -100 + res;
                if (p_tcp->m_tcp_sock_client != -1)
                    close(p_tcp->m_tcp_sock_client);
                p_tcp->m_tcp_sock_client = -1;
                p_tcp->start_connect_thread(); //自动重连
                sleep(p_tcp->m_connect_interval + 1);
            }
            else
            {
                if (p_tcp->m_pReadCallback != NULL)
                    p_tcp->m_pReadCallback(p_tcp->rev_buf, res);
            }
        }

        if (p_tcp->m_thread_read_avtived == 0)
        {
            break;
        }
    }
}

void *CTcpClient::tcp_connect_thread(void *arg)
{
    CTcpClient *p_tcp = (CTcpClient *)arg;

    while (1)
    {
        if (p_tcp->init_tcp_client() == 0) //TCP连接成功
        {
            p_tcp->tid_conn = -1;
            break;
        }

        if (p_tcp->m_status == 0)
            break;                        //关闭TCP
        sleep(p_tcp->m_connect_interval); //休眠时间单位是秒
    }
}

void CTcpClient::set_read_callback_func(const pFunc_ReadCallback pReadFunc)
{
    m_pReadCallback = pReadFunc;
}

sint32 CTcpClient::get_tcp_status()
{
    return m_connected_status;
}
sint32 CTcpClient::get_last_error()
{
    return m_last_error;
}

sint32 CTcpClient::send_tcp_data(unsigned char *str, uint32 len)
{
    if (m_connected_status == 0)
        return -1;

    if (m_tcp_sock_client == -1)
    {
        start_connect_thread(); //自动重连
        return -2;
    }

    if (send(m_tcp_sock_client, str, len, 0) != len)
    {
        m_last_error = -200;
        if (m_tcp_sock_client != -1)
            close(m_tcp_sock_client);
        m_tcp_sock_client = -1;
        start_connect_thread(); //自动重连
        return -3;
    }
    return 0;
}
} // namespace skywell