#include "param.h"

namespace skywell{
	Param::Param()
	{
		m_radar_num = 1;
		m_leafsize = 0.1;
		m_clustertype = 0;
	}

	Param::~Param()
	{
		
	}

	void Param::setDistanceMap(int distance,double value)
	{
		std::map<int,double>::iterator iter;
		iter = m_distance_map.find(distance);
		if (iter != m_distance_map.end())
		{
			iter->second = value;
			return;
		}
		m_distance_map.insert(std::make_pair(distance,value));
	};

	//数字转字符串
	std::string i2s(int number) {
		char str[20] = {0};
		sprintf(str, "%d", number);
		return string(str);
	}
	std::string f2s(const double value, unsigned int precisionAfterPoint)
	{
		std::ostringstream out;
		// 清除默认精度
		out.precision(std::numeric_limits<double>::digits10);
		out << value;
	 
		std::string res = std::move(out.str());
		auto pos = res.find('.');
		if (pos == std::string::npos)
			return res;
	 
		auto splitLen = pos + 1 + precisionAfterPoint;
	   if (res.size() <= splitLen)
			return res;
	 
		return res.substr(0, splitLen);
	}


	double Param::round(double f, int bits)
	{
		stringstream ss;
		ss << fixed << setprecision(bits) << f;
		ss >> f;
	 
		return f;
	}
	static bool DEBUG_PRINT	= true;
	int Param::loadcfg(void)
	{
		int ret = -1;
		TCfgFileParser cfg;
		ret = cfg.parseFile("./conf/perception.cfg");
		if (ret != 0)
		{
			//配置文件加载失败
			ROS_INFO("perception_track loadcfg error ,exit(1)!\n");
			return -1;
		}
		server_ip =  cfg.getValue("TRACK_NODE","server_ip");
		server_port = atoi(cfg.getValue("TRACK_NODE","server_port").c_str());
		sub_vehicle_state = cfg.getValue("TRACK_NODE","sub_vehicle_state");
		sub_obst_set =  cfg.getValue("TRACK_NODE","sub_obst_set");
		pub_track_set =  cfg.getValue("TRACK_NODE","pub_track_set");
		pub_bounding_boxs = cfg.getValue("TRACK_NODE","pub_bounding_boxs");
		domain_control_serverip =  cfg.getValue("TRACK_NODE","domain_control_serverip");
		domain_control_serverport = atoi(cfg.getValue("TRACK_NODE","domain_control_serverport").c_str());
		interactive_interface_protocol = cfg.getValue("TRACK_NODE", "interactive_interface_protocol");

		if (DEBUG_PRINT)
		{
			ROS_INFO("########################TRACK_NODE#########################\n");
			ROS_INFO("track::server_ip:%s\n",server_ip.c_str());
			ROS_INFO("track::server_port:%d\n",server_port);
			ROS_INFO("track::sub_vehicle_state:%s\n",sub_vehicle_state.c_str());
			ROS_INFO("track::sub_obst_set:%s\n",sub_obst_set.c_str());
			ROS_INFO("track::pub_track_set:%s\n",pub_track_set.c_str());
			ROS_INFO("track::pub_bounding_boxs:%s\n",pub_bounding_boxs.c_str());
			ROS_INFO("track::domain_control_serverip:%s\n",domain_control_serverip.c_str());
			ROS_INFO("track::domain_control_serverport:%d\n",domain_control_serverport);
			ROS_INFO("track::interactive_interface_protocol:%s\n", interactive_interface_protocol.c_str());
			DEBUG_PRINT = false;
		}
		return 0;
	};


	//从字符串中提取关键字值
	string Param::findkeyvalue(string source,string key)
	{
	   string value = "";
	   string::size_type loc1 = source.find(key);
	   if (loc1 !=string::npos)
	   {
			string::size_type pos = loc1 + key.size() + 1;
			for (int i = pos; i < source.size(); i++)
			{
				if ((source.at(i) >= '0' && source.at(i) <= '9')||(source.at(i) == '.')||(source.at(i) == '-'))
				{
					value += source.at(i);
				}
				else
				{
					break;
				}
			}
	   }
	   return value;
	};
}
