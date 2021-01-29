#include "lane_detection_manager.h"

namespace skywell
{
    LaneManager::LaneManager(ros::NodeHandle &nh, skywell::Param *param)
    {
        param_ = param;
        LaneManagerSubInit(nh, param);
    }

    LaneManager::~LaneManager(){};

    LaneManager::LaneManagerSubInit(ros::NodeHandle &nh, skywell::Param *param)
    {
        if (param->sub_no_ground != "")
        {
            sub_no_ground_ =
                nh.subscribe<sensor::PointCloud2>(param->sub_no_ground, 10,
                                                  boost::bind(&LaneManager::doRslidarWork,
                                                              this, -1, param->sub_no_ground));
        }
    }

    void LaneManager::LaneManagerPubInit(ros::NodeHandle &nh, skywell::Param *param)
    {
        if (param->pub_neareast_points != "")
        {
            pub_neareast_points_ = nh.advertise<sensor_msgs::PointCloud2>(param->pub_neareast_points, 10);
        }
    }

    void LaneManager::doRslidarWork(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr, const std::string topic_name)
    {
        param_->loadcfg();

        CloudT::Ptr _current_pointcloud_ptr = boost::make_shared<CloudT>();
        pcl::fromROSMsg(*in_cloud_ptr, *_current_pointcloud_ptr);

        detectLaneLine(param_->ray_angle_for_lane_detection, _current_pointcloud_ptr);
    }

    void LaneManager::XYZI_to_RTZ(float ray_angle, const CloudT::Ptr in_cloud_ptr,
                                  std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                  std::vector<PointCloudXYZIRT> &out_radial_ordered_clouds)
    {
        for (size_t i = 0; i < in_cloud_ptr->points.size(); ++i)
        {
            PointXYZIRT _new_point;
            auto _radius = (float)sqrt(in_cloud_ptr->points[i].x * in_cloud_ptr->points[i].x +
                                           in_cloud_ptr->points[i].y * in_cloud_ptr >
                                       points[i].y);

            auto _theta = (float)atan2(in_cloud_ptr->points[i].y, in_cloud_ptr->points[i].x) * 180 / 3.1415926;

            if (_theta < 0)
            {
                -theta += 360;
            }

            auto _radial_div = (size_t)floor(_theta / ray_angle);

            _new_point.point = in_cloud_ptr.points[i];
            _new_point.radius = _radius;
            _new_point.theta = _theta;
            _new_point.radial_div = _radial_div;
            _new_point.original_index = i;

            out_radial_divided_indices[_radial_div].indices.push_back(i);
            out_radial_ordered_clouds[_radial_div].push_back(_new_point);
        }

        for (size_t i = 0; i < out_radial_ordered_clouds.size(); ++i)
        {
            std::sort(out_radial_ordered_clouds.begin(), out_radial_ordered_clouds.end(),
                      [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.radius < b.radius; });
        }
    }

    void LaneManager::detectLaneLine(float ray_angle, const CloudT in_cloud_ptr)
    {
        std::vector<pcl::PointIndices> _radial_division_indices;
        std::vector<PointCloudXYZIRT> _radial_ordered_clouds;
        std::vector<pcl::PointIndices> _radial_nearest_points;

        int _radial_dividers_num = ceil(360 / ray_angle);
        _radial_division_indices.resize(_radial_dividers_num);
        _radial_ordered_clouds.resize(_radial_dividers_num);
        //_radial_nearest_points_indices.resize(_radial_dividers_num);
        pcl::PointIndices _radial_nearest_points_indices;

        XYZI_to_RTZ(ray_angle, in_cloud_ptr, _radial_division_indices, _radial_ordered_clouds);

        for (size_t i = 0; i < _radial_ordered_clouds.size(); ++i)
        {
            _radial_nearest_points_indices.indices.push_back(_radial_ordered_clouds[i][0].original_index);
        }

        pcl::ExtractIndices<PointT> _extract_neareast_points;
        CloudT::Ptr _neareast_points_cloud_ptr = boost::make_shared<CloudT>();

        _extract_neareast_points.setInputCloud(in_cloud_ptr);
        _extract_neareast_points.setIndices(boost::make_shared<pcl::PointIndices>(_radial_nearest_points_indices));
        _extract_neareast_points.setNegative(false);
        _extract_neareast_points.filter(*_neareast_points_cloud_ptr);
        publish_cloud(pub_neareast_points_, _neareast_points_cloud_ptr);
    }

    void LaneManager::publish_cloud(const ros::Publisher &in_publisher,
                                    const CloudT::Ptr in_cloud_to_publish_ptr)
    {
        sensor_msgs::PointCloud2 _cloud_msg;
        pcl::toROSMsg(*in_cloud_to_publish_ptr, _cloud_msg);
        _cloud_msg.header.frame_id = "livox_frame";
        in_publisher.publish(_cloud_msg);
        //ROS_INFO("segment points.size() = %ld publish_cloud %ld",in_cloud_to_publish_ptr->points.size(),in_cloud_to_publish_ptr->header.stamp);
    }

    void LaneManager::lookForLaneLine(float miny, float maxy, int size, CloudT in_cloud_ptr)
    {
        std::vector<CloudT> _points_for_counting_numbers;

        int _col = (int)(maxy - miny) / size;

        for (size_t i = 0; i < _col; ++i)
        {
            for (size_t j = 0; j < in_cloud_ptr->points.size(); ++j)
            {
                if (in_cloud_ptr->points[j].y < (miny + (i + 1) * _col))
                {
                    _points_for_counting_numbers[i].push_back(in_cloud_ptr->points[j]);
                }
            }
        }

        for (size_t i = 0; i < _points_for_counting_numbers.size(); ++i)
        {
            printf("the size of pointcloud %d is %d", i, _points_for_counting_numbers[i].size());
        }
    }

} // namespace skywell