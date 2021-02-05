#ifndef _PARAM_H_
#define _PARAM_H_
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include "config.h"



namespace skywell{


using namespace std;
class Pbox
{
	public:
		Pbox():m_xmin(0),m_ymin(0),m_zmin(0),m_xmax(0),m_ymax(0),m_zmax(0),m_leafsize(0.1){};
		void setmin(double xmin,double ymin,double zmin){
			m_xmin = xmin;
			m_ymin = ymin;
			m_zmin = zmin;
		};
		void setmax(double xmax,double ymax,double zmax){
			m_xmax = xmax;
			m_ymax = ymax;
			m_zmax = zmax;
		};
		void setleafsize(float leafsize){
			m_leafsize = leafsize;
		};
	public:
		Pbox& operator=(const Pbox &box)
		{
			if (this != &box)
			{
				this->m_xmin = box.m_xmin;
				this->m_ymin = box.m_ymin;
				this->m_zmin = box.m_zmin;
				this->m_xmax = box.m_xmax;
				this->m_ymax = box.m_ymax;
				this->m_zmax = box.m_zmax;
			}
			return *this;
		}
	public:
	double m_xmin,m_ymin,m_zmin;
	double m_xmax,m_ymax,m_zmax;
	float m_leafsize;
};

#define  EUCLIDEAN_TYPE   0
#define  DBSCAN_TYPE      1
#define  REGIONGROWING_TYPE      2

class Param
{
	public:
		//typedef std::pair<int, float> Distance_Pair;
		//typedef std::vector<Distance_Pair> Distance_Pair_Vector;
	public:
		Param();
		~Param();
	public:
		void setRadarNum(int num){m_radar_num = num;};
		void setLeafSize(double leafsize){m_leafsize = leafsize;};

		void setleftBox(skywell::Pbox box){leftbox = box;};
		void setrightBox(skywell::Pbox box){rightbox = box;};
		void setmidleBox(skywell::Pbox box){midlebox = box;};
		void setfrontBox(skywell::Pbox box){frontbox = box;};
		
		void setplaneBox(skywell::Pbox box){planebox = box;};
		void setSegParam(int maxiterations,double distance_threshold){
			m_distance_threshold = distance_threshold;
			m_maxiterations = maxiterations;
		};
		void setClusterType(int type){m_clustertype = type;}
		
		void setEuclideanClusterSize(int minsize,int maxsize){
			m_min_size = minsize;
			m_max_size = maxsize;
		};

		void setDistanceMap(int distance,double value);

		void setDBScanClusterSize(int n,int m,double r){
			m_around_num = n;
			m_min_num = m;
			m_radius = r;
		}

	public:
		double round(double f, int bits);
	public:

		int m_radar_num;
		//下采样体素叶大小
		double m_leafsize;
		//左边下采样 范围
		skywell::Pbox leftbox;
		//右边下采样 范围
		skywell::Pbox rightbox;
		//中间下采样 范围
		skywell::Pbox midlebox;
		//前方下采样 范围
		skywell::Pbox frontbox;

		//地面分割算法选择 0:平面分割，1：渐进式形态分割，2：渐进上形态分割
		int m_seqtype;
		//平面模型分割点云
		//地面范围
		skywell::Pbox planebox;
		double m_distance_threshold;
		int m_maxiterations;
		//渐进式形态学滤波地面
		float m_max_w_s;
		float m_slope;
		float m_max_d ;
		float m_initial_d;
		float m_cell_s;
		float m_base ;
		bool m_exponential;

		// 离群滤波
		float m_raduis;
		int m_minneighbor;

		//聚类类型
		int m_clustertype;
		//欧几里德聚类参数设置
		int m_min_size;
		int m_max_size;
		std::map<int,double>m_distance_map;

		//DBSCAN聚类参数设置
		int m_around_num; // 最小周围数量
		int m_min_num;//最小聚类数量
		double m_radius;  // 搜索半径


		//REGIONGROWING 区域增长，
		int m_rg_min_s;
		int m_rg_max_s;
		int m_rg_kn;
		float m_rg_smoothness;// 平滑度
		float m_rg_curvature;// 曲率
	public:
		int loadcfg(void);
	public:
		int m_lidar_num;
		// lidar topic
		map<int,string> m_lidar_topic_map;

	public:
		void loadparam(ros::NodeHandle node, ros::NodeHandle private_nh);
		string findkeyvalue(string source,string key);
	public:
		string front_rslidar_topic;
		string left_rslidar_topic;
		string right_rslidar_topic;
		int seg_model;
		float check_angle;
		string pub_ground;
		string pub_no_ground;
		//点云下采样大小
		float leaf_size;
		// 过滤点云
		// 1.根据车身高度，消减一定高度的点云
		float clip_height;
		// 2.根据车辆尺寸，过滤车身点云
		float clip_min_x;
		float clip_min_y;
		float clip_max_x;
		float clip_max_y;
		// 射线去地面参数
		float ray_angle;
		float he;
		float hg;
	    // 射线去地面参数 model 0
		float initial_distance;
		float max_distance;
		float height_threshold;
		float slope_threshold;
};



}


#endif
