#include <ros/ros.h>
#include <track/Cluster.h>
#include <track/ObstSet.h>
#include <track/Tk_State.h>
#include <track/Heart.h>
#include <track/OnLineState.h>
#include <boost/make_shared.hpp>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <time.h>
#include <iostream> 
#include <fstream> 
#include "match.h"
#include "feature.h"
#include "priority_rank.h"
#include "tcp_client.h"
#include "tcp_server.h"
#include "send_protocal_32960.h"
#include "param.h"
#include "vehicle_state.h"
#include "easylogging++.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/stringbuffer.h"
#include "commclient.h"

INITIALIZE_EASYLOGGINGPP

using namespace rapidjson;
using namespace std;

//64位 微妙时间戳格式化 %G-%m-%d %H:%M:%S.%N
string gettimestamp(uint64_t timestamp)
{
	char szTmp[50] = {0};
	time_t tmptime = timestamp/1000000;
	struct tm _tm = {0};
	localtime_r(&tmptime, &_tm);
	strftime(szTmp,50,"%G-%m-%d %H:%M:%S",&_tm);
	sprintf(&szTmp[strlen(szTmp)],".%ld",(timestamp%1000000)/1000);
	string str(szTmp);
	return str;
};
	//数字转字符串
	std::string uint64_to_string(uint64_t value)
	{
		char str[20] = {0};
		sprintf(str, "%ld", value);
		return string(str);
	};


	std::string int_to_string (int value) {
		char str[20] = {0};
		sprintf(str, "%d", value);
		return string(str);
	}
	std::string double_to_string(const double value, unsigned int precisionAfterPoint)
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

void rolloutHandler(const char *filename, std::size_t size)  
{
    std::string fn(filename);

    std::stringstream ss;
    ss << "cp " << filename << " "
    << fn.substr(0, fn.find_last_of('.'))
    << "_"
    << "`date +'%Y%m%d%H%M%S"
    << ".log'`";
    system(ss.str().c_str());
}



//全局变量
skywell::Feature Featurer;	   //取特征
skywell::Match Matcher;		   //匹配追踪
skywell::Rank cluster_rank(0); //聚类后排序去边上小物体
Skywell::CTcpServer tcp_send;  //发送障碍物到以太网
skywell::CommClient comm_client;
skywell::VehicleState global_vehicle_state;
ros::Publisher pub;
ros::Publisher pub_bounding_boxs_;
ros::Subscriber sub_obst;
ros::Subscriber sub_vehicle_state;
ros::Subscriber sub_segment_heart_state;
ros::Subscriber sub_cluster_heart_state;
ros::Subscriber sub_lidar_state;
std::string flag_interface_protocol; //用于接口协议更改判断




void recv_heart_state(const track::Heart &heart)
{
	//LOG(WARNING)<<"process_id:"<<heart.process_id;
	comm_client.Heart(heart.process_id);
}

void recv_lidar_state(const track::OnLineState &onlinestate)
{
	//LOG(WARNING)<<"decv_id:"<<onlinestate.decv_id<<",decv_state:"<<onlinestate.decv_state;
	// 杨昆还不支持
	//comm_client.UpInfo(onlinestate.decv_id,onlinestate.decv_state,0,0);
}

void recv_vehicle_state(const track::Tk_State &input)
{
	//LOG(INFO)<<"input.longitude:"<<input.longitude<<",input.latitude:"<<input.latitude<<",input.velocity:"<<input.velocity<<",input.velocity_x:"<<input.velocity_x<<",input.velocity_y:"<<input.velocity_y;

	global_vehicle_state.updateVehicleState(input.time_stamp_1,
								input.roll,
								input.pitch,
								input.yaw,
								input.longitude,
								input.latitude,
								input.X,
								input.Y,
								input.wheel_angle,
								input.velocity,
								input.velocity_x,
								input.velocity_y);
}


void transfer( boost::shared_ptr<std::vector<skywell::Object>> &target)
{
	// 先获取
	uint64_t time_stamp;
	float roll;
	float pitch;
	float yaw;
	double longitude;
	double latitude;
	float x;
	float y;
	int wheel_angle;
	int velocity;
	float velocity_x;
	float velocity_y;
	global_vehicle_state.getVehicleState(time_stamp,
		roll,
		pitch,
		yaw,
		longitude,
		latitude,
		x,
		y,
		wheel_angle,
		velocity,
		velocity_x,
		velocity_y);
	if (target->size() > 0)
	{
		int diff_timestamp = (*target)[0].timestamp/1000 - time_stamp;
		if ( abs(diff_timestamp) < 300 )//// 误差300ms
		{
			for (size_t i = 0; i < target->size(); i++)
			{
				(*target)[i].abs_speedx = (*target)[i].speedx;//+ velocity_x;
				(*target)[i].abs_speedy = (*target)[i].speedy;//+ velocity_y;
				ROS_INFO("---------------->velocity_x = %f,velocity_y = %f ,abs_velocity = %f,velocity = %d\n",velocity_x,velocity_y,sqrt(velocity_x * velocity_x + velocity_y * velocity_y),velocity);

				double  abs_speed = sqrt((*target)[i].speedx * (*target)[i].speedx + (*target)[i].speedy * (*target)[i].speedy);
				ROS_INFO("=================>target_id = %d,speedx = %f,speedy = %f,abs_speed = %f,%f\n",(*target)[i].number,(*target)[i].speedx,(*target)[i].speedy,abs_speed,abs_speed*3.6);
			}
		}
		else
		{
				ROS_INFO("target.timestamp = %ld,vehicle_state.timestamp = %ld\n",(*target)[0].timestamp/1000,time_stamp);
		}
	}

};


void match(const track::ObstSet &input)
{
	uint64_t timestamp = 0;
	struct timeval start;
	struct timeval end;
	unsigned long timer;
	gettimeofday(&start, NULL);
	struct tm _tm = {0};
	localtime_r(&start.tv_sec, &_tm);
	//ROS_INFO("input.obsts.size() = %ld\n",input.obsts.size());
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ObstVector;
	timestamp = input.header.stamp.toSec() * 1000000;
	for (int i = 0; i < input.obsts.size(); i++)
	{
		if (input.obsts[i].points.size() > 0)
		{
			//ROS_INFO("input.obsts[%d].points.size() = %ld\n",i,input.obsts[i].points.size());
			pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
			cloud->width = input.obsts[i].points.size();
			cloud->height = 1;
			cloud->points.resize(cloud->width * cloud->height);
			cloud->header.stamp = input.obsts[i].header.stamp.toSec() * 1000000;
			for (int index = 0; index < input.obsts[i].points.size(); index++)
			{
				cloud->points[index].x = input.obsts[i].points[index].x;
				cloud->points[index].y = input.obsts[i].points[index].y;
				cloud->points[index].z = input.obsts[i].points[index].z;
				cloud->points[index].intensity = input.obsts[i].points[index].intensity;
			}
			ObstVector.push_back(cloud);
		}
	}
	boost::shared_ptr<std::vector<skywell::Object>> objects = boost::make_shared<std::vector<skywell::Object>>();

	Featurer.makeCubes(ObstVector, objects);

	//两场数据目标匹配
	boost::shared_ptr<std::vector<skywell::Object>> target = boost::make_shared<std::vector<skywell::Object>>();
	Matcher.match(objects,target);

	//transfer(target);

	//ROS_INFO("target num = %ld\n",target->size());
	//目标障碍物相对车身速度转化为相对大地的绝对速度
	jsk_recognition_msgs::BoundingBoxArray bbox_array;
	std_msgs::Header point_cloud_header_;

	for (size_t i = 0; i < target->size(); i++)
	{
		jsk_recognition_msgs::BoundingBox bounding_box_;
		bounding_box_.header = point_cloud_header_;
		bounding_box_.header.frame_id = "livox_frame";
		bounding_box_.pose.position.x = (*target)[i].transform.x();
		bounding_box_.pose.position.y = (*target)[i].transform.y();
		bounding_box_.pose.position.z = (*target)[i].transform.z();

		bounding_box_.dimensions.x =  (*target)[i].length;
		bounding_box_.dimensions.y =  (*target)[i].width;
		bounding_box_.dimensions.z =  (*target)[i].height;
		bbox_array.boxes.push_back(bounding_box_);
	}
	bbox_array.header = point_cloud_header_;
	bbox_array.header.frame_id = "livox_frame";
	pub_bounding_boxs_.publish(bbox_array);

	unsigned short target_num = target->size();
	unsigned short datalen = 0;
	unsigned short send_datalen = 0;
	unsigned char *databuf = NULL;
	unsigned char *send_databuf = NULL;

	if (flag_interface_protocol == "old_720")
	{
		datalen = target_num * 28 + 8;
		send_datalen = datalen + TCP_32960_HEADER_LEN + TCP_32960_TAIL_LEN;
		databuf = new unsigned char[datalen];
		send_databuf = new unsigned char[send_datalen];
		long long time = (_tm.tm_hour * 3600 + _tm.tm_min * 60 + _tm.tm_sec) * 1000 + start.tv_usec / 1000; //usecond
		cluster_rank.objects_header_fill_720(databuf, target_num, 1, time);
	}
	else if (flag_interface_protocol == "new_915")
	{
		datalen = target_num * PER_OBSTANCLE_LEN + OBSTANCLE_HEADER_LEN;
		send_datalen = datalen + TCP_32960_HEADER_LEN + TCP_32960_TAIL_LEN;
		databuf = new unsigned char[datalen];
		send_databuf = new unsigned char[send_datalen];
		//const unsigned long  USEC_PER_DAY = (24 * 3600 * 1000);
		//unsigned int today_msec = ((*target)[0].timestamp/1000)%USEC_PER_DAY;
		cluster_rank.objects_header_fill_915(databuf, target_num, (timestamp / 1000));
	}
	StringBuffer buffer;
	Writer<StringBuffer> writer(buffer);
	writer.StartObject();
	writer.Key("group_name"); 
	writer.String("pc");
	writer.Key("timestamp"); writer.String(gettimestamp(timestamp).c_str());
	writer.Key("up_info");
	writer.StartObject();
	writer.Key("obst_time"); writer.String(uint64_to_string(timestamp/1000).c_str());
	writer.Key("obst_num");writer.String(int_to_string(target_num).c_str());
	writer.Key("obst_list");
	writer.StartArray();
	for (size_t i = 0; i < target_num; i++)
	{
		if (flag_interface_protocol == "old_720")
		{
			cluster_rank.objects_send_fill_720(databuf, (*target)[i], i);
		}
		else if (flag_interface_protocol == "new_915")
		{
			cluster_rank.objects_send_fill_915(databuf, (*target)[i], i);
		}
		writer.StartObject();
		writer.Key("target_id"); writer.String(int_to_string((*target)[i].number).c_str());
		writer.Key("confidence"); writer.String(int_to_string((*target)[i].life).c_str());
		writer.Key("type"); writer.String(int_to_string((*target)[i].type).c_str());
		writer.Key("centre_pos");
			writer.StartObject();
			writer.Key("x"); writer.String(double_to_string((*target)[i].transform(0),3).c_str());
			writer.Key("y"); writer.String(double_to_string((*target)[i].transform(1),3).c_str());
			writer.Key("z"); writer.String(double_to_string((*target)[i].transform(2),3).c_str());
			writer.EndObject();
		writer.Key("boundingbox");
			writer.StartObject();
			writer.Key("width"); writer.String(double_to_string((*target)[i].width,3).c_str());
			writer.Key("length"); writer.String(double_to_string((*target)[i].length,3).c_str());
			writer.Key("height"); writer.String(double_to_string((*target)[i].height,3).c_str());
			writer.EndObject();
		writer.Key("speed");
			writer.StartObject();
			writer.Key("x"); writer.String(double_to_string((*target)[i].speedx,3).c_str());
			writer.Key("y"); writer.String(double_to_string((*target)[i].speedy,3).c_str());
			writer.EndObject();
		writer.Key("acceleration");
			writer.StartObject();
			writer.Key("x"); writer.String(double_to_string((*target)[i].accelerationx,3).c_str());
			writer.Key("y"); writer.String(double_to_string((*target)[i].accelerationy,3).c_str());
			writer.EndObject();
		writer.Key("angular_speed"); writer.String("0.00");
		writer.Key("angular_acceleration"); writer.String("0.00");
		writer.Key("cut"); writer.String(int_to_string((*target)[i].cut_in_out).c_str());
		writer.Key("polygon_vertex");
		writer.StartArray();
		// 多边形顶点，暂时按4个发送
		for (int j = 0; j < 4 ; j++)
		{
			writer.StartObject();
			writer.Key("x"); writer.String(double_to_string((*target)[i].P[j].x,3).c_str());
			writer.Key("y"); writer.String(double_to_string((*target)[i].P[j].y,3).c_str());
			writer.EndObject();
		}
		writer.EndArray();
		writer.EndObject();
	}
	writer.EndArray();
	writer.EndObject();
	writer.EndObject();
	std::string sbuffer = buffer.GetString();
	gettimeofday(&end, NULL);
	timer = 1000000 * (end.tv_sec - start.tv_sec) + end.tv_usec - start.tv_usec;
 
	LOG(TRACE)<<"[start:"<<gettimestamp(start.tv_sec*1000000+start.tv_usec)<<"]--[end:"<<gettimestamp(end.tv_sec*1000000+end.tv_usec)<<"]:"<<sbuffer;
	size_t sbufflen = sbuffer.size();
	Skywell::set_frame_data(COMMAND_FLAG, REPLY_FLAG, databuf, datalen, send_databuf, &send_datalen);
	//Skywell::test_read_one_frame(target_num, send_databuf);
	tcp_send.send_client_data(send_databuf, send_datalen);

	delete[] databuf;
	delete[] send_databuf;
	databuf = NULL;
	send_databuf = NULL;


	// struct tm _tm = {0};
	localtime_r(&start.tv_sec, &_tm);
	printf("处理耗时 = %ld ms\n", (timer / 1000));
}


int main(int argc, char **argv)
{
	struct sigaction sa;
	sa.sa_handler = SIG_IGN;
	sigaction( SIGPIPE, &sa, 0 );
	skywell::Param param; 
	param.loadcfg();
	el::Configurations conf("./conf/log.conf");  
    el::Loggers::reconfigureAllLoggers(conf);  
	el::Loggers::addFlag(el::LoggingFlag::StrictLogFileSizeCheck);
	el::Helpers::installPreRollOutCallback(rolloutHandler);
	int rnt_int_ser = tcp_send.init_tcp_server(param.server_ip.c_str(), param.server_port);
	flag_interface_protocol = param.interactive_interface_protocol;
	if (0 != rnt_int_ser)
		std::cout << "init_tcp_server error!!!!" << std::endl;
	ros::init(argc, argv, "track_node");
	ros::NodeHandle nh;
	if (param.sub_obst_set != "")
	{
		sub_obst = nh.subscribe(param.sub_obst_set, 10, match);
	}
	if (param.sub_vehicle_state != "")
	{
		sub_vehicle_state = nh.subscribe(param.sub_vehicle_state, 10, recv_vehicle_state);
	}
	sub_segment_heart_state = nh.subscribe("/segment_node/heart", 10, recv_heart_state);
	sub_cluster_heart_state = nh.subscribe("/cluster_node/heart", 10, recv_heart_state);
	sub_lidar_state = nh.subscribe("lidar_oline_state", 10, recv_lidar_state);
	if (param.pub_track_set != "")
	{
		pub = nh.advertise<track::Cluster>(param.pub_track_set, 10);
	}
	if (param.pub_bounding_boxs != "")
	{
		pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(param.pub_bounding_boxs, 10);
	}
	comm_client.Init(param.domain_control_serverip.c_str(),param.domain_control_serverport);
	ros::MultiThreadedSpinner spinner(3);
	spinner.spin();
}

