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
			ROS_INFO("perception_seg loadcfg error ,exit(1)!\n");
			return -1;
		}
		front_rslidar_topic = cfg.getValue("SEG_NODE","front_rslidar_topic");
		left_rslidar_topic = cfg.getValue("SEG_NODE","left_rslidar_topic");
		right_rslidar_topic = cfg.getValue("SEG_NODE","right_rslidar_topic");

		pub_ground = cfg.getValue("SEG_NODE","pub_ground");
		pub_no_ground = cfg.getValue("SEG_NODE","pub_no_ground");
		leaf_size =   atof(cfg.getValue("SEG_NODE","leaf_size").c_str());
		clip_height =   atof(cfg.getValue("SEG_NODE","clip_height").c_str());
		string clip_body_range = cfg.getValue("SEG_NODE","clip_body_range");
		string value = "";
		value = findkeyvalue(clip_body_range,"minx");
		if (value != "")
		{
			clip_min_x = atof(value.c_str());
		}else
		{
			clip_min_x = -0.1f;
		}
		value = findkeyvalue(clip_body_range,"miny");
		if (value != "")
		{
			clip_min_y = atof(value.c_str());
		}else
		{
			clip_min_y = -0.1f;
		}
		value = findkeyvalue(clip_body_range,"maxx");
		if (value != "")
		{
			clip_max_x = atof(value.c_str());
		}else
		{
			clip_max_x = 0.1f;
		}
		value = findkeyvalue(clip_body_range,"maxy");
		if (value != "")
		{
			clip_max_y = atof(value.c_str());
		}else
		{
			clip_max_y = 0.1f;
		}

		seg_model = atoi(cfg.getValue("SEG_NODE","seg_model").c_str());
		check_angle = atof(cfg.getValue("SEG_NODE","check_angle").c_str());

		ray_angle = atof(cfg.getValue("RAY_SEG","ray_angle").c_str());
		initial_distance = atof(cfg.getValue("RAY_SEG","initial_distance").c_str());
		max_distance = atof(cfg.getValue("RAY_SEG","max_distance").c_str());
		height_threshold = atof(cfg.getValue("RAY_SEG","height_threshold").c_str());
		slope_threshold = atof(cfg.getValue("RAY_SEG","slope_threshold").c_str());


		if (DEBUG_PRINT)
		{
			ROS_INFO("########################SEG_NODE#########################\n");
			ROS_INFO("front_rslidar_topic:%s\n",front_rslidar_topic.c_str());
			ROS_INFO("left_rslidar_topic:%s\n",left_rslidar_topic.c_str());
			ROS_INFO("right_rslidar_topic:%s\n",right_rslidar_topic.c_str());
			ROS_INFO("Clip:height:%f,minx:%f,miny:%f,maxx:%f,maxy:%f\n",clip_height,clip_min_x,clip_min_y,clip_max_x,clip_max_y);
			ROS_INFO("seg_model:%d\n",seg_model);
			ROS_INFO("check_angle:%f\n",check_angle);
			ROS_INFO("ray_seg:ray_angle:%f,initial_distance:%f,max_distance:%f,height_threshold:%f,slope_threshold:%f\n",ray_angle,initial_distance,max_distance,height_threshold,slope_threshold);
			DEBUG_PRINT = false;
		}
		return 0;
	};

	void Param::loadparam(ros::NodeHandle node, ros::NodeHandle private_nh)
	{
		private_nh.param("front_rslidar_topic",front_rslidar_topic,std::string(""));
		private_nh.param("left_rslidar_topic",left_rslidar_topic,std::string(""));
		private_nh.param("right_rslidar_topic",right_rslidar_topic,std::string(""));
		private_nh.param("seg_model",seg_model,int(0));
		private_nh.param("check_angle",check_angle,float(0));
		private_nh.param("pub_ground",pub_ground,std::string(""));
		private_nh.param("pub_no_ground",pub_no_ground,std::string(""));

		ROS_INFO("front_rslidar_topic:%s\n",front_rslidar_topic.c_str());
		ROS_INFO("left_rslidar_topic:%s\n",left_rslidar_topic.c_str());
		ROS_INFO("right_rslidar_topic:%s\n",right_rslidar_topic.c_str());
		ROS_INFO("seg_model:%d\n",seg_model);
		ROS_INFO("check_angle:%f\n",check_angle);

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
