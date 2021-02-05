#ifndef _MANAGER_H_
#define _MANAGER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <cluster/Cluster.h>
#include <cluster/ObstSet.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <boost/thread/mutex.hpp>
#include <sys/types.h>
#include <unistd.h>
#include <pcl/octree/octree.h>
#include <fstream>  
#include <iostream>  
#include <random>
#include <memory.h>
#include <set>
#include<math.h>


#include "param.h"
#include "filter.h"
#include "match.h"
#include "cluster.h"
#include "cluster3d.h"


typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef unsigned long long uint64;


using namespace std;
namespace skywell{

class Manager
{
	public:
		Manager(ros::NodeHandle &nh,skywell::Param *param);
		~Manager();
	private:
		void SubInit(ros::NodeHandle &nh,skywell::Param *param);
		void PubInit(ros::NodeHandle &nh,skywell::Param *param);
		void ModInit(skywell::Param *param);
	private:
		void doRslidarWork(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr,const std::string topicName);
		void ToObstSet(ClusterVectorPtr midsave,cluster::ObstSet &obstset,uint64_t stamp);
	private:
		void MyEuclideanClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,ClusterVectorPtr midsave, float clusterTolerance, int minSize,int maxSize);
	private:
		skywell::Param *m_param;
		ros::Subscriber sub_ground;
		ros::Subscriber sub_no_ground;

		ros::Publisher pub_obst_set;
};

}



#endif
