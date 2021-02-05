#ifndef _LANE_DETECTION_MANAGER_H_
#define _LANE_DETECTION_MANAGER_H_

#include <algorithm>
#include <math.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <param.h>
#include <config.h>

#include <boost/bind.hpp>


typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

namespace skywell
{

    typedef struct st_Point
    {
        double x;
        double y;
    } st_Point;

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

    class LaneManager
    {
    public:
        LaneManager(ros::NodeHandle &nh, skywell::Param *param);
        ~LaneManager();

    private:
        ros::Subscriber sub_no_ground_;

        ros::Publisher pub_neareast_points_;
        ros::Publisher pub_lane_line_left_;
        ros::Publisher pub_lane_line_right_;
        ros::Publisher pub_lane_line_marker_left_;
        ros::Publisher pub_lane_line_marker_right_;

        skywell::Param* param_;

    private:
        void LaneManagerSubInit(ros::NodeHandle &nh, skywell::Param *param);
        void LaneManagerPubInit(ros::NodeHandle &nh, skywell::Param *param);

        void doRslidarWork(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr, const std::string topic_name);

        void XYZI_to_RTZ(float ray_angle, const CloudT::Ptr in_cloud_ptr,
                         std::vector<pcl::PointIndices> &out_radial_divided_indices,
                         std::vector<PointCloudXYZIRT> &out_radial_ordered_clouds);

        void detectLaneLine(float ray_angle, const CloudT::Ptr in_cloud_ptr);

        void publish_cloud(const ros::Publisher &in_publisher,
                           const CloudT::Ptr in_cloud_to_publish_ptr);

        void lookForLaneLine(float miny, float maxy, float size, CloudT::Ptr in_cloud_ptr);

        void clip_above(double clip_height, const CloudT::Ptr in,
                        const CloudT::Ptr out);

        int ransacLinear(std::vector<st_Point> &pstData, int dataCnt,
                         int minCnt, double maxIterCnt,
                         int consensusCntThreshold, double maxErrorThreshold,
                         double &A, double &B, double &C, double &modelMeanError,
                         set<unsigned int>& consensusIndexs);
    };
} // namespace skywell

#endif