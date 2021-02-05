#include "decvcheck.h"


namespace skywell{




int DecvCheck::Init(ros::Publisher pub)
{
	decv_state_pub = pub;
};

DecvCheck::DecvCheck()
{
	online_num = 0;
	
}

DecvCheck::~DecvCheck()
{
	Decv *decv = NULL;
	while (!m_decv_map.empty())
	{
		decv = (*m_decv_map.begin()).second;
		m_decv_map.erase(m_decv_map.begin());
		if (decv != NULL)
		{
			delete decv;
			decv = NULL;
		}
	}	
}
int DecvCheck::Process(void)
{
	ros::Time now_time = ros::Time::now();
	map<string,Decv*>::iterator iter;
	for (iter = m_decv_map.begin();iter != m_decv_map.end(); iter++)
	{
		double alive_interval_time = (now_time - iter->second->alive_time).toSec()*1000;
		if (alive_interval_time > 500.0)// 0.5s,设备离线
		{
			double alarm_interval_time = (now_time - iter->second->alarm_time).toSec()*1000;
			if (alarm_interval_time > iter->second->alarm_interval*1000.0)
			{
				if (iter->second->online_status)
				{
					online_num = online_num - 1;
					iter->second->online_status = false;
					segment::OnLineState online_state;
					online_state.decv_id = iter->second->decv_id;
					online_state.decv_state = 0; // 0 离线，定义故障为 雷达无数据
					decv_state_pub.publish(online_state);
				}
				{
					// 上报离线消息 
					ROS_INFO("ERROR %s is Online  0 \n",iter->second->decv_name.c_str());
				}
				iter->second->alarm_time = ros::Time::now();
			}
		}else
		{
			if (!iter->second->online_status)
			{
				segment::OnLineState online_state;
				online_state.decv_id = iter->second->decv_id;
				online_state.decv_state = 1; // 0 上线，定义故障为 雷达无数据故障消除
				decv_state_pub.publish(online_state);
				// 上报上线消息
				ROS_INFO("ERROR %s is Online  1 \n",iter->second->decv_name.c_str());
			}
		}
	}
	usleep(500*1000);
	ThreadStop();
	return 0;
};

void DecvCheck::addDecv(string decv_name,int decv_id)
{
	map<string,Decv*>::iterator iter= m_decv_map.find(decv_name);
	if (iter == m_decv_map.end())
	{
		Decv *decv = new Decv;
		decv->decv_name = decv_name;
		decv->alive_time = ros::Time::now();
		decv->alarm_time = ros::Time::now();
		decv->alarm_interval = 1;
		decv->decv_id = decv_id;
		decv->online_status = false;
		m_decv_map.insert(make_pair(decv_name,decv));
	}
	ThreadStart();
};

void DecvCheck::aliveDecv(string decv_name)
{
	map<string,Decv*>::iterator iter= m_decv_map.find(decv_name);
	if (iter != m_decv_map.end())
	{
		iter->second->alive_time = ros::Time::now();
		if (!iter->second->online_status)
		{
			iter->second->online_status = true;
			online_num = online_num +1;
		}
	}
};


}