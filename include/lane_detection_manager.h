#ifndef _LANE_DETECTION_MANAGER_H_
#define _LANE_DETECTION_MANAGER_H_

#include <algorithm>
#include <math.h>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

namespace skywell
{

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
        LaneManager();
        ~LaneManager();

    private:
        ros::Subscriber sub_no_ground_;

        ros::Publisher pub_neareast_points_;
        skywell::Param param_;

    private:
        void LaneManagerSubInit(ros::NodeHandle &nh, skywell::Param *param);

        void XYZI_to_RTZ(float ray_angle, const CloudT::Ptr in_cloud_ptr,
                         std::vector<pcl::PointIndices> &out_radial_divided_indices,
                         std::vector<PointCloudXYZIRT> &out_radial_ordered_clouds);

        void detectLaneLine(float ray_angle, const CloudT in_cloud_ptr);

        void publish_cloud(const ros::Publisher &in_publisher,
                           const CloudT::Ptr in_cloud_to_publish_ptr);

        void lookForLaneLine(float miny, float maxy, int size, CloudT in_cloud_ptr);
    };
} // namespace skywell