#ifndef _COMM_CLIENT_H_
#define _COMM_CLIENT_H_

#include <string>
#include <iostream> 
#include <fstream> 
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "client.h"
#include "thread.h"



using namespace std;
namespace skywell{

#define COMM_CLIENT_RECV_BUF	1024*1024

class CommClient:public Thread
{
	public:
		CommClient(void);
		virtual ~CommClient(void);
	public:
		void Init(string ip,int port);
		void UnInit(void);
		virtual int Process(void);
	public:
		void Heart(unsigned int pid);
		void UpInfo(unsigned int decv_id,unsigned int decv_state,unsigned int error_level,unsigned int flag);
	public:
		Client *m_client;
	private:
		string m_ip;
		int m_port;
	private:
		time_t m_heart_dt;
		time_t m_last_heart_dt;
		int m_heart_time_out;
		int m_heart_count;
	private:
		unsigned char m_recv_buf[COMM_CLIENT_RECV_BUF];
		int m_recv_len;
	private:
		pthread_mutex_t send_mutex;

};

}








#endif
