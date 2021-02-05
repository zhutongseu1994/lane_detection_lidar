#ifndef _MANAGER_H_
#define _MANAGER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <unordered_set>
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



#include "filter.h"
#include "cluster.h"
#include "param.h"
#include "decvcheck.h"
#include "grid_segment.h"

#define RADIAL_DIVIDER_ANGLE 1.0
#define concentric_divider_distance_ 0.3 //0.1 meters default
#define min_height_threshold_ 0.1
#define local_max_slope_ 8   //max slope of the ground between points, degree
#define general_max_slope_ 5 //max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;
typedef unsigned long long uint64;



using namespace std;
namespace skywell{


typedef struct st_Point
{
    double x;
    double y;
}st_Point;

struct PointXYZIRT
{
	PointT point;
	float radius;
	float theta;
	int ground_flag;
	size_t radial_div;
	size_t original_index;
};

typedef std::vector<PointXYZIRT> PointCloudXYZIRT;

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
		int ransacLiner(std::vector<st_Point> &pstData, int dataCnt, int minCnt, 
							double maxIterCnt, int consensusCntThreshold,double maxErrorThreshod,
							double& A, double& B, double& C,double& modelMeanError);
		void XYZI_to_RTZ(float ray_angle,const CloudT::Ptr in_cloud,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRT> &out_radial_ordered_clouds);
		void XYZI_to_RTZ(const CloudT::Ptr in_cloud,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<pcl::PointCloud<PointT>> &out_radial_ordered_clouds);
	
		void classify_pc(std::vector<PointCloudXYZIRT> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,float he,float hg,
                              pcl::PointIndices &out_no_ground_indices,pcl::PointIndices &out_road_boundary_indices);
		bool check_point_is_ground(std::vector<PointXYZIRT> &ground_points,int index,PointXYZIRT &pointxyzrt,double initial_distance,double max_distance,double height_threshold,double slope_threshold);
		bool reverse_check_point_is_ground(std::vector<PointXYZIRT> &ground_points,int index,int points_size,PointXYZIRT &pointxyzrt,double initial_distance,double max_distance,double height_threshold,double slope_threshold);
		void my_classify_pc(std::vector<PointCloudXYZIRT> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,double initial_distance,double  max_distance,double height_threshold,double slope_threshold,
                              pcl::PointIndices &out_no_ground_indices,pcl::PointIndices &out_road_boundary_indices);
		void publish_cloud(const ros::Publisher &in_publisher,
                                const CloudT::Ptr in_cloud_to_publish_ptr);
		void removeNan(CloudT::Ptr in, CloudT::Ptr out);
		void remove_close_pt(double min_distance, const CloudT::Ptr in,
                                  const CloudT::Ptr out);
		void remove_close_pt(double minx,double minyz,double maxx,double maxy,const CloudT::Ptr in,const CloudT::Ptr out);
		void clip_above(double clip_height, const CloudT::Ptr in,
                             const CloudT::Ptr out);
		void octreeVoxelGrid(float leafsize,CloudT::Ptr in, CloudT::Ptr out);
	private:
		void my_segment( CloudT::Ptr cloud);
		// 去地面算法
		void RayGroundSegment(float ray_angle,float he,float hg,const CloudT::Ptr in_cloud);
		void GroundSegment(float ray_angle,float initial_distance,float  max_distance,float height_threshold,float slope_threshold,const CloudT::Ptr in_cloud);
		void debugRayGround(const CloudT::Ptr in_cloud);
		void RansacSegmentPlane(const CloudT::Ptr in_cloud);
		void plane_segment(const CloudT::Ptr in_cloud);
		void grid_segment(const CloudT::Ptr in_cloud);
		void ProgressiveMorphologicalFilter(const CloudT::Ptr in_cloud);
	private:
		void doRslidarWork(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr,const std::string topicName);
	private:
		std::vector<CloudT> cache_livox_point_cloud;
		skywell::Param *m_param;
		//skywell::TcpServer *m_tcpserver;

	private:
		ros::Subscriber sub_front_rslidar;
		ros::Subscriber sub_left_rslidar;
		ros::Subscriber sub_right_rslidar;

		ros::Publisher pub_ground;
		ros::Publisher pub_no_ground;
		ros::Publisher marker_pub;

		ros::Publisher decv_state_pub;
	private:
		//ros::Publisher pub_2_track;
		ros::Publisher pub_point_1;
		ros::Publisher pub_point_2;
		ros::Publisher pub_point_3;
		ros::Publisher pub_point_4;

		unsigned long int last_point_could_stamp;
	private:
		boost::mutex mutex;
		ros::Time startTime;
	private:
		DecvCheck *m_decv_check;
		ofstream m_file;
	private:
		boost::shared_ptr<std::vector<skywell::Grid>> ground_grid_list = boost::make_shared<std::vector<Grid>>();
};

}



#endif
