#include "lane_detection_manager.h"

namespace skywell
{
    LaneManager::LaneManager(ros::NodeHandle &nh, skywell::Param *param)
    {
        param_ = param;
        LaneManagerPubInit(nh, param);
        LaneManagerSubInit(nh, param);
    }

    LaneManager::~LaneManager(){};

    void LaneManager::LaneManagerSubInit(ros::NodeHandle &nh, skywell::Param *param)
    {
        if (param->sub_no_ground != "")
        {
            sub_no_ground_ =
                nh.subscribe<sensor_msgs::PointCloud2>(param->sub_no_ground, 10,
                                                       boost::bind(&LaneManager::doRslidarWork,
                                                                   this, _1, param->sub_no_ground));
        }
    }

    void LaneManager::LaneManagerPubInit(ros::NodeHandle &nh, skywell::Param *param)
    {
        pub_lane_line_marker_left_ = nh.advertise<visualization_msgs::Marker>("lane_line_left_marker", 10);

        pub_lane_line_marker_right_ = nh.advertise<visualization_msgs::Marker>("lane_line_right_marker", 10);

        if (param->pub_neareast_points != "")
        {
            pub_neareast_points_ = nh.advertise<sensor_msgs::PointCloud2>(param->pub_neareast_points, 10);
        }

        pub_lane_line_right_ = nh.advertise<sensor_msgs::PointCloud2>("lane_line_right", 10);

        pub_lane_line_left_ = nh.advertise<sensor_msgs::PointCloud2>("lane_line_left", 10);
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
                                       in_cloud_ptr->points[i].y * in_cloud_ptr->points[i].y);

            auto _theta = (float)atan2(in_cloud_ptr->points[i].y, in_cloud_ptr->points[i].x) * 180 / 3.1415926;

            if (_theta < 0)
            {
                _theta += 360;
            }

            auto _radial_div = (size_t)floor(_theta / ray_angle);

            _new_point.point = in_cloud_ptr->points[i];
            _new_point.radius = _radius;
            _new_point.theta = _theta;
            _new_point.radial_div = _radial_div;
            _new_point.original_index = i;

            out_radial_divided_indices[_radial_div].indices.push_back(i);
            out_radial_ordered_clouds[_radial_div].push_back(_new_point);
        }

        for (size_t i = 0; i < out_radial_ordered_clouds.size(); ++i)
        {
            std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                      [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.radius < b.radius; });
        }
    }

    void LaneManager::detectLaneLine(float ray_angle, const CloudT::Ptr in_cloud_ptr)
    {
        std::vector<pcl::PointIndices> _radial_division_indices;
        std::vector<PointCloudXYZIRT> _radial_ordered_clouds;
        std::vector<pcl::PointIndices> _radial_nearest_points;

        int _radial_dividers_num = ceil(360 / ray_angle);
        _radial_division_indices.resize(_radial_dividers_num);
        _radial_ordered_clouds.resize(_radial_dividers_num);

        pcl::PointIndices _radial_nearest_points_indices;

        CloudT::Ptr _point_cloud_after_clip = boost::make_shared<CloudT>();
        clip_above(param_->lane_clip_height, in_cloud_ptr, _point_cloud_after_clip);

        XYZI_to_RTZ(ray_angle, _point_cloud_after_clip, _radial_division_indices, _radial_ordered_clouds);

        for (size_t i = 0; i < _radial_ordered_clouds.size(); ++i)
        {

            if (_radial_ordered_clouds[i].size() > 0)
            {
                _radial_nearest_points_indices.indices.push_back(_radial_ordered_clouds[i].front().original_index);
            }
        }

        pcl::ExtractIndices<PointT> _extract_neareast_points;
        CloudT::Ptr _neareast_points_cloud_ptr = boost::make_shared<CloudT>();

        _extract_neareast_points.setInputCloud(in_cloud_ptr);
        _extract_neareast_points.setIndices(boost::make_shared<pcl::PointIndices>(_radial_nearest_points_indices));
        _extract_neareast_points.setNegative(false);
        _extract_neareast_points.filter(*_neareast_points_cloud_ptr);
        publish_cloud(pub_neareast_points_, _neareast_points_cloud_ptr);

        lookForLaneLine(param_->lane_region_miny, param_->lane_region_maxy, param_->lane_region_size, _neareast_points_cloud_ptr);
    }

    void LaneManager::lookForLaneLine(float miny, float maxy, float size, CloudT::Ptr in_cloud_ptr)
    {
        //printf("miny = %f, maxy = %f, size = %f\n", miny, maxy, size);

        std::vector<PointCloudXYZIRT> _points_for_counting_numbers;
        std::vector<pcl::PointIndices> _points_indices_for_counting_numbers;

        int _col = (int)ceil((maxy - miny) / size);

        _points_for_counting_numbers.resize(_col);
        _points_indices_for_counting_numbers.resize(_col);

        //printf("_col is %d\n", _col);

        //printf("in_cloud_ptr->size() is : %d\n", in_cloud_ptr->size());

        for (size_t i = 0; i < _col; ++i)
        {
            for (size_t j = 0; j < in_cloud_ptr->points.size(); ++j)
            {
                if (in_cloud_ptr->points[j].y > (miny + i * size) &&
                    in_cloud_ptr->points[j].y < (miny + (i + 1) * size))
                {
                    PointXYZIRT _new_point;
                    _new_point.point = in_cloud_ptr->points[j];
                    _new_point.original_index = i;

                    _points_for_counting_numbers[i].push_back(_new_point);
                    _points_indices_for_counting_numbers[i].indices.push_back(j);
                }
            }
        }

        for (size_t i = 0; i < _points_for_counting_numbers.size(); ++i)
        {
            printf("the size of pointcloud %d is %d\n", i, _points_for_counting_numbers[i].size());
        }

        bool _flag_for_check_right_line = false;
        for (size_t i = 0; i < (int)ceil(_col / 2); ++i)
        {
            bool _flag = true;
            _flag_for_check_right_line = (!(bool)_points_for_counting_numbers[i].size()) && _flag;
        }

        if (!_flag_for_check_right_line)
        {
            std::sort(_points_for_counting_numbers.begin(), _points_for_counting_numbers.begin() + (int)ceil(_col / 2),
                      [](const PointCloudXYZIRT &a, const PointCloudXYZIRT &b) { return a.size() > b.size(); });

            size_t _lane_line_right_index = _points_for_counting_numbers[0].front().original_index;

            printf("_lane_line_right_index is %d\n", _lane_line_right_index);

            pcl::PointIndices::Ptr _lane_line_right_points_indices_ptr =
                boost::make_shared<pcl::PointIndices>(_points_indices_for_counting_numbers[_lane_line_right_index]);

            pcl::ExtractIndices<PointT> _extract_lane_line_right;
            CloudT::Ptr _lane_line_right_ptr = boost::make_shared<CloudT>();

            _extract_lane_line_right.setInputCloud(in_cloud_ptr);
            _extract_lane_line_right.setIndices(_lane_line_right_points_indices_ptr);
            _extract_lane_line_right.setNegative(false);
            _extract_lane_line_right.filter(*_lane_line_right_ptr);
            publish_cloud(pub_lane_line_right_, _lane_line_right_ptr);

            std::vector<st_Point> _tmp_data_points;
            for (size_t i = 0; i < _lane_line_right_ptr->points.size(); ++i)
            {
                st_Point _point;
                _point.x = _lane_line_right_ptr->points[i].x;
                _point.y = _lane_line_right_ptr->points[i].y;

                _tmp_data_points.push_back(_point);
                std::cout << "[x: " << _point.x << ", y: " << _point.y << "]\n";
            }
            double _A = 0;
            double _B = 0;
            double _C = 0;
            double _mean_error = 0;
            int _data_len = _tmp_data_points.size();
            set<unsigned int> _consensus_indexs;
            st_Point _point_a;
            st_Point _point_b;

            if (!ransacLinear(_tmp_data_points, _data_len, 2, _data_len, _data_len * 0.6, 0.2, _A, _B, _C, _mean_error, _consensus_indexs))
            {
                ROS_INFO("A = %f,B = %f,C= %f,Error = %f,K = %f, consensus_indexs.size = %d", _A, _B, _C, _mean_error, (-_A / _B), _consensus_indexs.size());
            }
            else
            {
                ROS_INFO("RANSAC Failed\n");
            }

            if (_consensus_indexs.size() > 1)
            {
                set<unsigned int>::iterator _consensus_iter = _consensus_indexs.begin();

                _point_a.x = _tmp_data_points[*_consensus_iter].x;
                _point_a.y = _tmp_data_points[*_consensus_iter].y;

                _point_b.x = _tmp_data_points[*(++_consensus_iter)].x;
                _point_b.y = _tmp_data_points[*(++_consensus_iter)].y;

                ROS_INFO("_point_a.x = %f, _point_a.y = %f, _point_b.x = %f, _point_b.y = %f", _point_a.x, _point_a.y, _point_b.x, _point_b.y);
                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "livox_frame";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = "lines";
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.pose.orientation.w = 1.0;
                line_list.id = 0;
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.scale.x = 0.1;
                // Line list is red
                line_list.color.r = 1.0;
                line_list.color.a = 1.0;
                // Create the vertices for the points and lines
                geometry_msgs::Point p;
                p.x = 0;
                p.y = (-_A / _B) * p.x - _C / _B;
                p.z = 0;
                // The line list needs two points for each line
                line_list.points.push_back(p);
                p.x = 20.0;
                p.y = (-_A / _B) * p.x - _C / _B;
                p.z = 0;
                line_list.points.push_back(p);
                pub_lane_line_marker_right_.publish(line_list);
            }
        }

        bool _flag_for_check_left_line = false;
        for (size_t i = (int)ceil(_col / 2); i < _col; ++i)
        {
            bool _flag = true;
            _flag_for_check_left_line = (!(bool)_points_for_counting_numbers[i].size()) && _flag;
        }

        if (!_flag_for_check_left_line)
        {
            std::sort(_points_for_counting_numbers.begin() + (int)ceil(_col / 2), _points_for_counting_numbers.end(),
                      [](const PointCloudXYZIRT &a, const PointCloudXYZIRT &b) { return a.size() > b.size(); });

            size_t _lane_line_left_index = _points_for_counting_numbers[ceil(_col / 2)].front().original_index;

            printf("_lane_line_left_index is %d\n", _lane_line_left_index);

            pcl::PointIndices::Ptr _lane_line_left_points_indices_ptr =
                boost::make_shared<pcl::PointIndices>(_points_indices_for_counting_numbers[_lane_line_left_index]);

            pcl::ExtractIndices<PointT> _extract_lane_line_left;
            CloudT::Ptr _lane_line_left_ptr = boost::make_shared<CloudT>();

            _extract_lane_line_left.setInputCloud(in_cloud_ptr);
            _extract_lane_line_left.setIndices(_lane_line_left_points_indices_ptr);
            _extract_lane_line_left.setNegative(false);
            _extract_lane_line_left.filter(*_lane_line_left_ptr);
            publish_cloud(pub_lane_line_left_, _lane_line_left_ptr);

            std::vector<st_Point> _tmp_data_points;
            for (size_t i = 0; i < _lane_line_left_ptr->points.size(); ++i)
            {
                st_Point _point;
                _point.x = _lane_line_left_ptr->points[i].x;
                _point.y = _lane_line_left_ptr->points[i].y;

                _tmp_data_points.push_back(_point);
                std::cout << "[x: " << _point.x << ", y: " << _point.y << "]\n";
            }
            double _A = 0;
            double _B = 0;
            double _C = 0;
            double _mean_error = 0;
            int _data_len = _tmp_data_points.size();
            set<unsigned int> _consensus_indexs;
            st_Point _point_a;
            st_Point _point_b;

            if (!ransacLinear(_tmp_data_points, _data_len, 2, _data_len, _data_len * 0.6, 0.2, _A, _B, _C, _mean_error, _consensus_indexs))
            {
                ROS_INFO("A = %f,B = %f,C= %f,Error = %f,K = %f, consensus_indexs.size = %d", _A, _B, _C, _mean_error, (-_A / _B), _consensus_indexs.size());
            }
            else
            {
                ROS_INFO("RANSAC Failed\n");
            }

            if (_consensus_indexs.size() > 1)
            {
                set<unsigned int>::iterator _consensus_iter = _consensus_indexs.begin();

                _point_a.x = _tmp_data_points[*_consensus_iter].x;
                _point_a.y = _tmp_data_points[*_consensus_iter].y;

                _point_b.x = _tmp_data_points[*(++_consensus_iter)].x;
                _point_b.y = _tmp_data_points[*(++_consensus_iter)].y;

                ROS_INFO("_point_a.x = %f, _point_a.y = %f, _point_b.x = %f, _point_b.y = %f", _point_a.x, _point_a.y, _point_b.x, _point_b.y);
                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "livox_frame";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = "lines";
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.pose.orientation.w = 1.0;
                line_list.id = 0;
                line_list.type = visualization_msgs::Marker::LINE_LIST;
                line_list.scale.x = 0.1;
                // Line list is red
                line_list.color.r = 1.0;
                line_list.color.a = 1.0;
                // Create the vertices for the points and lines
                geometry_msgs::Point p;
                p.x = 0;
                p.y = (-_A / _B) * p.x - _C / _B;
                p.z = 0;
                // The line list needs two points for each line
                line_list.points.push_back(p);
                p.x = 20.0;
                p.y = (-_A / _B) * p.x - _C / _B;
                p.z = 0;
                line_list.points.push_back(p);
                pub_lane_line_marker_left_.publish(line_list);
            }

            // visualization_msgs::Marker line_list;
            // line_list.header.frame_id = "livox_frame";
            // line_list.header.stamp = ros::Time::now();
            // line_list.ns = "lines";
            // line_list.action = visualization_msgs::Marker::ADD;
            // line_list.pose.orientation.w = 1.0;
            // line_list.id = 2;
            // line_list.type = visualization_msgs::Marker::LINE_LIST;
            // line_list.scale.x = 0.1;
            // // Line list is red
            // line_list.color.r = 1.0;
            // line_list.color.a = 1.0;
            // // Create the vertices for the points and lines
            // geometry_msgs::Point p;
            // p.x = _point_a.y;
            // p.y = _point_a.y;
            // p.z = 0;
            // // The line list needs two points for each line
            // line_list.points.push_back(p);
            // p.x = _point_b.x;
            // p.y = _point_b.y;
            // p.z = 0;
            // line_list.points.push_back(p);
            // pub_lane_line_marker_left_.publish(line_list);
        }
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

    // 消减 上面点
    void LaneManager::clip_above(double clip_height, const CloudT::Ptr in,
                                 const CloudT::Ptr out)
    {
        pcl::ExtractIndices<PointT> _cliper;

        _cliper.setInputCloud(in);
        pcl::PointIndices _indices;
#pragma omp parallel for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            if (in->points[i].z > clip_height)
            {
#pragma omp critical
                _indices.indices.push_back(i);
            }
        }
        _cliper.setIndices(boost::make_shared<pcl::PointIndices>(_indices));
        _cliper.setNegative(true); //ture to remove the indices
        _cliper.filter(*out);
    }

    int LaneManager::ransacLinear(std::vector<st_Point> &pstData, int dataCnt, int minCnt,
                                  double maxIterCnt, int consensusCntThreshold, double maxErrorThreshold,
                                  double &A, double &B, double &C, double &modelMeanError,
                                  set<unsigned int> &consensusIndexs)
    {
        // 基于RANSAC算法的直线拟合
        // pstData: 指向存储数据的引用
        // dataCnt: 数据点个数
        // lineParameterK: 直线的斜率
        // lineParameterB: 直线的截距
        // minCnt: 模型(直线)参数估计所需的数据点的个数
        // maxIterCnt: 最大迭代次数
        // maxErrorThreshold: 最大误差阈值
        // consensusCntThreshold: 模型一致性判断准则
        // modelMeanError: 模型误差
        // 返回值： 返回0表示获取最优模型， 否则表示未获取最优模型
        default_random_engine rng;
        uniform_int_distribution<unsigned> uniform(0, dataCnt - 1);
        rng.seed(10);                   // 固定随机数种子
        set<unsigned int> selectIndexs; // 选择的点的索引
        vector<st_Point> selectPoints;  // 选择的点
        //set<unsigned int> consensusIndexs; // 满足一致性的点的索引
        double line_A = 0;
        double line_B = 0;
        double line_C = 0;
        modelMeanError = 0;
        int isNonFind = 1;
        unsigned int bestConsensusCnt = 0; // 满足一致性估计的点的个数
        int iter = 0;
        while (iter < maxIterCnt)
        {
            selectIndexs.clear();
            selectPoints.clear();

            // Step1: 随机选择minCnt个点
            while (1)
            {
                unsigned int index = uniform(rng);
                selectIndexs.insert(index);
                if (selectIndexs.size() == minCnt)
                {
                    break;
                }
            }

            // Step2: 进行模型参数估计 (y2 - y1)*x - (x2 - x1)*y + (y2 - y1)x2 - (x2 - x1)y2= 0
            set<unsigned int>::iterator selectIter = selectIndexs.begin();
            while (selectIter != selectIndexs.end())
            {
                unsigned int index = *selectIter;
                selectPoints.push_back(pstData[index]);
                selectIter++;
            }
            double deltaY = (selectPoints[1]).y - (selectPoints[0]).y;
            double deltaX = (selectPoints[1]).x - (selectPoints[0]).x;
            line_A = deltaY;
            line_B = -deltaX;
            line_C = -deltaY * (selectPoints[0]).x + deltaX * (selectPoints[0]).y;

            // Step3: 进行模型评估: 点到直线的距离
            int dataIter = 0;
            double meanError = 0;
            set<unsigned int> tmpConsensusIndexs;
            while (dataIter < dataCnt)
            {
                double distance = (line_A * pstData[dataIter].x + line_B * pstData[dataIter].y + line_C) / sqrt(line_A * line_A + line_B * line_B);
                distance = distance > 0 ? distance : -distance;
                if (distance < maxErrorThreshold)
                {
                    tmpConsensusIndexs.insert(dataIter);
                }
                meanError += distance;
                dataIter++;
            }

            // Step4: 判断一致性: 满足一致性集合的最小元素个数条件 + 至少比上一次的好
            if (tmpConsensusIndexs.size() >= bestConsensusCnt && tmpConsensusIndexs.size() >= consensusCntThreshold)
            {
                bestConsensusCnt = consensusIndexs.size(); // 更新一致性索引集合元素个数
                modelMeanError = meanError / dataCnt;
                consensusIndexs.clear();
                consensusIndexs = tmpConsensusIndexs; // 更新一致性索引集合
                A = line_A;
                B = line_B;
                C = line_C;
                isNonFind = 0;

                //     set<unsigned int>::iterator consensus_iter = consensusIndexs.begin();
                //     point_a.x = pstData[*consensus_iter].x;
                //     point_a.y = pstData[*consensus_iter].y;

                //     point_b.x = pstData[*(consensus_iter++)].x;
                //     point_b.y = pstData[*(consensus_iter++)].y;
            }
            iter++;
        }
        return isNonFind;
    }

} // namespace skywell