#ifndef DECV_CHECK_H
#define DECV_CHECK_H
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <time.h>
#include <map>
#include <segment/OnLineState.h>
#include "thread.h"


using namespace std;


namespace skywell{

class Decv
{
	public:
		string decv_name;
		ros::Time alive_time;
		ros::Time alarm_time;
		bool online_status;
		int alarm_interval;
		int decv_id;

};

class DecvCheck:public Thread
{
	public:
		DecvCheck();
		~DecvCheck();
	public:
		int Init(ros::Publisher pub);
		int Process(void);
		void addDecv(string decv_name,int decv_id);
		void aliveDecv(string decv_name);
		int  getOnlineNum(void){return online_num;};
	private:
		int online_num;
	private:
		map<string,Decv*> m_decv_map;
	private:
		ros::Publisher decv_state_pub;
};


}








#endif

