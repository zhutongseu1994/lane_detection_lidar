#include "commclient.h"

namespace skywell{

	CommClient::CommClient()
	{
		m_client = NULL;
		m_ip = "";
		m_port = 0;
		m_heart_time_out = 1;// 1s
		m_heart_dt = time(NULL);
		m_last_heart_dt = time(NULL);
		m_heart_count = 0;
	};

	CommClient::~CommClient()
	{
		UnInit();
	};

	void CommClient::Init(string ip,int port)
	{
		m_ip = ip;
		m_port = port;
		pthread_mutex_init(&send_mutex,NULL);
		m_client = new Client;
		ThreadStart();
	};
	void CommClient::UnInit(void)
	{
		if (m_client == NULL)
		{
			delete m_client;
		}
		pthread_mutex_destroy(&send_mutex);
	};
	int CommClient::Process(void)
	{
		int ret = -1;
		if (m_client->State() < 0) // 不在线
		{
			ret = m_client->Connect((char*)m_ip.c_str(),m_port);
			if (ret != 0)
			{
				sleep(1);
				return -1;
			}
		}
		ret = m_client->Recv(m_recv_buf,m_recv_len);
		if (ret != 0)
		{
			m_client->DisConnect();
			return -1;
		}else
		{
			// 不用解析，有数据就判断连接正常
			if (m_recv_len > 0)
			{
				m_last_heart_dt = time(NULL);
			}
		}
		if ((abs(time(NULL) - m_last_heart_dt) >= m_heart_time_out)&&(abs(time(NULL) - m_heart_dt ) >= m_heart_time_out))
		{
			// 发送心跳时间到
			//Heart(getpid());
			Heart(1003);
			m_heart_dt = time(NULL);
		}
		if (abs(time(NULL) - m_last_heart_dt) > 2 * m_heart_time_out)
		{
			m_client->DisConnect();
			return -1;
		}

		usleep(100*1000);
		return 0;
	};
	void CommClient::Heart(unsigned int pid)
	{
		struct timeval now;
		gettimeofday(&now, NULL);
		struct tm _tm = {0};
		localtime_r(&now.tv_sec, &_tm);
		unsigned int time = (_tm.tm_hour * 3600 + _tm.tm_min * 60 + _tm.tm_sec) * 1000 + now.tv_usec / 1000; //usecond

		unsigned char heartbuf[] = {
		0x23,0x23,              // 起始字符
		0x93,                   // 命令单元
		0xFE,                   // 应当标识 需要应答
		0x00,0x00,0x00,0x00,    // 时间戳
		0x01,                   // 加密方式 01 不加密
		0x00,0x0C,              // 数据长度 固定8个字节
		0x00,0x00,0x00,0x01,    // 模块号
		0x00,0x00,0x00,0x00,    // 子模块号
		0x00,0x00,0x00,0x00,    // 心跳值
		0x24,0x24               // 结束字符
		};
		heartbuf[4] = (time >> 24) & 0xff;
		heartbuf[5] = (time >> 16) & 0xff;
		heartbuf[6] = (time >> 8) & 0xff;
		heartbuf[7] = time & 0xff;


		heartbuf[15] = (pid >> 24) & 0xff;
		heartbuf[16] = (pid >> 16) & 0xff;
		heartbuf[17] = (pid >> 8) & 0xff;
		heartbuf[18] = pid & 0xff;
		m_heart_count++;
		m_heart_count = m_heart_count%65536;

		heartbuf[19] = (m_heart_count >> 24) & 0xff;
		heartbuf[20] = (m_heart_count >> 16) & 0xff;
		heartbuf[21] = (m_heart_count >> 8) & 0xff;
		heartbuf[22] = m_heart_count & 0xff;
		if (0 == pthread_mutex_lock(&send_mutex))
		{
			m_client->Send(heartbuf,25);
			pthread_mutex_unlock(&send_mutex);
		}
	};

	void CommClient::UpInfo(unsigned int decv_id,unsigned int decv_state,unsigned int error_level,unsigned int flag)
	{
		struct timeval now;
		gettimeofday(&now, NULL);
		struct tm _tm = {0};
		localtime_r(&now.tv_sec, &_tm);
		unsigned int time = (_tm.tm_hour * 3600 + _tm.tm_min * 60 + _tm.tm_sec) * 1000 + now.tv_usec / 1000; //usecond

		unsigned char upinfobuf[] = {
		0x23,0x23,              // 起始字符
		0x91,                   // 命令单元
		0xFE,                   // 应当标识 需要应答
		0x00,0x00,0x00,0x00,    // 时间戳
		0x01,                   // 加密方式 01 不加密
		0x00,0x14,              // 数据长度 固定20个字节
		0x00,0x00,0x00,0x01,    // 模块号
		0x00,0x00,0x00,0x00,    // 设备编号
		0x00,0x00,0x00,0x00,    // 模块状态
		0x00,0x00,0x00,0x00,    // 错误级别
		0x00,0x00,0x00,0x00,    // 告警产生或消除
		0x24,0x24               // 结束字符
		};
		upinfobuf[4] = (time >> 24) & 0xff;
		upinfobuf[5] = (time >> 16) & 0xff;
		upinfobuf[6] = (time >> 8) & 0xff;
		upinfobuf[7] = time & 0xff;

		// 添加设备编号，各模块根据自己模块连接的硬件类型自行规划（例如感知的主激光雷达，使用编号0x01,两侧激光雷达编号0x02 0x03）
		upinfobuf[15] = (decv_id >> 24) & 0xff;
		upinfobuf[16] = (decv_id >> 16) & 0xff;
		upinfobuf[17] = (decv_id >> 8) & 0xff;
		upinfobuf[18] = decv_id & 0xff;

		// 添加模块状态，0x00000000表示正常状态，各模块统一使用该告警码作为状态正常的上报指令
		// 感知：预留错误码范围0x00010000-0x0001FFFF
		unsigned int pc_state = 0x00010000 + decv_state;
		upinfobuf[19] = (pc_state >> 24) & 0xff;
		upinfobuf[20] = (pc_state >> 16) & 0xff;
		upinfobuf[21] = (pc_state >> 8) & 0xff;
		upinfobuf[22] = pc_state & 0xff;

		//添加错误级别，0:无故障（错误码为0x00000000时该字段填0）1:INFO（信息提示）2:WARNING(告警)3:ERROR（错误）
		upinfobuf[23] = (error_level >> 24) & 0xff;
		upinfobuf[24] = (error_level >> 16) & 0xff;
		upinfobuf[25] = (error_level >> 8) & 0xff;
		upinfobuf[26] = error_level & 0xff;
		//添加告警产生或消除，0：告警消除 1：告警产生 当错误码为0x00000000时，该字段不关注，可以填任意值
		upinfobuf[27] = (flag >> 24) & 0xff;
		upinfobuf[28] = (flag >> 16) & 0xff;
		upinfobuf[29] = (flag >> 8) & 0xff;
		upinfobuf[30] = flag & 0xff;
		if (0 == pthread_mutex_lock(&send_mutex))
		{
			m_client->Send(upinfobuf,33);
			pthread_mutex_unlock(&send_mutex);
		}
	};
}
