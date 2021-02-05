#include "manager.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
namespace skywell
{

    static float ground_k = 0.0f;
    static float ground_base = 0.0f;

    Manager::Manager(ros::NodeHandle &nh, skywell::Param *param)
    {
        m_param = param;
        m_decv_check = NULL;

        SubInit(nh, param);
        PubInit(nh, param);
        ModInit(param);
    };

    Manager::~Manager(){

    };

    void Manager::SubInit(ros::NodeHandle &nh, skywell::Param *param)
    {
        /*if (param->front_rslidar_topic != "")
	{
		sub_front_rslidar = nh.subscribe<sensor_msgs::PointCloud2>(param->front_rslidar_topic,10,boost::bind(&Manager::doRslidarWork,this,_1,param->front_rslidar_topic));
	}*/
        if (param->left_rslidar_topic != "")
        {
            sub_left_rslidar = nh.subscribe<sensor_msgs::PointCloud2>(param->left_rslidar_topic, 10, boost::bind(&Manager::doRslidarWork, this, _1, param->left_rslidar_topic));
        }
        if (param->right_rslidar_topic != "")
        {
            sub_right_rslidar = nh.subscribe<sensor_msgs::PointCloud2>(param->right_rslidar_topic, 10, boost::bind(&Manager::doRslidarWork, this, _1, param->right_rslidar_topic));
        }
    };
    void Manager::PubInit(ros::NodeHandle &nh, skywell::Param *param)
    {
        if (param->pub_ground != "")
        {
            pub_ground = nh.advertise<sensor_msgs::PointCloud2>(param->pub_ground, 10);
        }
        if (param->pub_no_ground != "")
        {
            pub_no_ground = nh.advertise<sensor_msgs::PointCloud2>(param->pub_no_ground, 10);
        }
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        decv_state_pub = nh.advertise<segment::OnLineState>("lidar_oline_state", 10);
    };

    void Manager::ModInit(skywell::Param *param)
    {
        if (m_decv_check == NULL)
        {
            m_decv_check = new DecvCheck;
            m_decv_check->Init(decv_state_pub);
            if (param->front_rslidar_topic != "")
            {
                m_decv_check->addDecv(param->front_rslidar_topic, 0x01);
            }
            if (param->left_rslidar_topic != "")
            {
                m_decv_check->addDecv(param->left_rslidar_topic, 0x02);
            }
            if (param->right_rslidar_topic != "")
            {
                m_decv_check->addDecv(param->right_rslidar_topic, 0x03);
            }
        }
    };
    //64位 微妙时间戳格式化 %G-%m-%d %H:%M:%S.%N
    string gettimestamp(uint64_t timestamp)
    {
        char szTmp[50] = {0};
        time_t tmptime = timestamp / 1000000;
        struct tm _tm = {0};
        localtime_r(&tmptime, &_tm);
        strftime(szTmp, 50, "%G-%m-%d %H:%M:%S", &_tm);
        sprintf(&szTmp[strlen(szTmp)], ".%ld", (timestamp % 1000000) / 1000);
        string str(szTmp);
        return str;
    };

    //

    void Manager::doRslidarWork(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr, const std::string topicName)
    {
        m_param->loadcfg();
        CloudT::Ptr current_pc_ptr = boost::make_shared<CloudT>();
        pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
        //根据topicName 检查对应话题是否工作正常是否有效
        m_decv_check->aliveDecv(topicName);
        //ROS_INFO("topicName:%s,current time:%s\n",topicName.c_str(),gettimestamp(current_pc_ptr->header.stamp).c_str());
        //ROS_INFO("m_decv_check->getOnlineNum() = %d\n",m_decv_check->getOnlineNum());
        // 第一阶段，多雷达数据合并
        //缓存数据，根据雷达数目，考虑缓存几场数据
        cache_livox_point_cloud.push_back(*current_pc_ptr);
        if (cache_livox_point_cloud.size() < m_decv_check->getOnlineNum())
        {
            return;
        }
        //多场数据合并
        CloudT::Ptr cloud_sum_ptr = boost::make_shared<CloudT>();
        for (size_t i = 0; i < cache_livox_point_cloud.size(); i++)
        {
            CloudT::Ptr cloud_ptr = boost::make_shared<CloudT>();
            cloud_ptr = cache_livox_point_cloud[i].makeShared();
            *cloud_sum_ptr += *cloud_ptr;
        }
        cache_livox_point_cloud.clear();
        ros::Time startTime = ros::Time::now();
        struct timeval start;
        gettimeofday(&start, NULL);
        //ROS_INFO("startTime time:%s\n",gettimestamp(start.tv_sec*1000000+start.tv_usec).c_str());
        // 体素下采样
        CloudT::Ptr octree_voxel_grid_ptr = boost::make_shared<CloudT>();
        octreeVoxelGrid(m_param->leaf_size, cloud_sum_ptr, octree_voxel_grid_ptr);

        // 去除nan点
        CloudT::Ptr remove_nan_ptr = boost::make_shared<CloudT>();
        removeNan(octree_voxel_grid_ptr, remove_nan_ptr);

        // 消减一定高度
        CloudT::Ptr cliped_pc_ptr = boost::make_shared<CloudT>();
        clip_above(m_param->clip_height, remove_nan_ptr, cliped_pc_ptr);

        // 去除自身周围
        CloudT::Ptr remove_close_ptr = boost::make_shared<CloudT>();
        remove_close_pt(m_param->clip_min_x, m_param->clip_min_y, m_param->clip_max_x, m_param->clip_max_y, cliped_pc_ptr, remove_close_ptr);

        if (m_param->seg_model == 0)
        {
            // 射线去地面
            GroundSegment(m_param->ray_angle, m_param->initial_distance, m_param->max_distance, m_param->height_threshold, m_param->slope_threshold, remove_close_ptr);
        }
        else if (m_param->seg_model == 1)
        {
            plane_segment(remove_close_ptr);
        }
        else if (m_param->seg_model == 2)
        {
            // 栅格去地面
            grid_segment(remove_close_ptr);
        }
        else if (m_param->seg_model == 9)
        {
            debugRayGround(remove_close_ptr);
        }
        ros::Time endTime = ros::Time::now();
        double elapsedTime = (endTime - startTime).toSec() * 1000;
        struct timeval end;
        gettimeofday(&end, NULL);
        //ROS_INFO("current time:%s\n",gettimestamp(current_pc_ptr->header.stamp).c_str());
        //ROS_INFO("endTime time:%s\n",gettimestamp(end.tv_sec*1000000+end.tv_usec).c_str());
        ROS_INFO("segment_node---------------- use_time = %f mesc\n", elapsedTime);
    };

    void Manager::RayGroundSegment(float ray_angle, float he, float hg, const CloudT::Ptr in_cloud)
    {
        std::vector<pcl::PointIndices> radial_division_indices;
        std::vector<PointCloudXYZIRT> radial_ordered_clouds;

        int radial_dividers_num_ = ceil(360 / ray_angle);
        radial_division_indices.resize(radial_dividers_num_);
        radial_ordered_clouds.resize(radial_dividers_num_);

        XYZI_to_RTZ(ray_angle, in_cloud, radial_division_indices, radial_ordered_clouds);

        std::vector<st_Point> tempdataPoints;
        for (size_t j = 0; j < radial_ordered_clouds[0].size(); j++)
        {
            float distance = 0.0f;
            size_t len = tempdataPoints.size();
            if (len >= 1)
            {
                distance = tempdataPoints[len - 1].x - tempdataPoints[0].x;
                if ((distance <= 20.0) && (tempdataPoints.size() < 100))
                {
                    st_Point _point;
                    _point.x = radial_ordered_clouds[0][j].radius;
                    _point.y = radial_ordered_clouds[0][j].point.z;
                    tempdataPoints.push_back(_point);
                }
                else
                {
                    break;
                }
            }
            else
            {
                st_Point _point;
                _point.x = radial_ordered_clouds[0][j].radius;
                _point.y = radial_ordered_clouds[0][j].point.z;
                tempdataPoints.push_back(_point);
            }
        }
        double A = 0;
        double B = 0;
        double C = 0;
        double meanError = 0;
        int datalen = tempdataPoints.size();
        if (!ransacLiner(tempdataPoints, datalen, 2, datalen * 0.5, datalen * 0.6, 0.3, A, B, C, meanError))
        {
            ROS_DEBUG("A = %f,B = %f,C= %f,Error = %f,K = %f,angle = %f\n", A, B, C, meanError, (-A / B), atan(-A / B) * 180 / 3.1415926);
            if (meanError <= 0.3)
            {
                ground_k = (-A / B);
                ground_base = (-C / B);
            }
        }
        else
        {
            ROS_DEBUG("RANSAC Failed ");
        }

        pcl::PointIndices ground_indices, no_ground_indices, road_boundary_indices;
        classify_pc(radial_ordered_clouds, ground_indices, he, hg, no_ground_indices, road_boundary_indices);

        CloudT::Ptr ground_cloud_ptr(new CloudT);
        CloudT::Ptr no_ground_cloud_ptr(new CloudT);

        pcl::ExtractIndices<PointT> extract_ground;
        extract_ground.setInputCloud(in_cloud);
        extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(no_ground_indices));

        extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
        extract_ground.filter(*no_ground_cloud_ptr);

        extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
        extract_ground.filter(*ground_cloud_ptr);
        publish_cloud(pub_ground, ground_cloud_ptr);
        publish_cloud(pub_no_ground, no_ground_cloud_ptr);
    };

    void Manager::GroundSegment(float ray_angle, float initial_distance, float max_distance, float height_threshold, float slope_threshold, const CloudT::Ptr in_cloud)
    {
        std::vector<pcl::PointIndices> radial_division_indices;
        std::vector<PointCloudXYZIRT> radial_ordered_clouds;

        int radial_dividers_num_ = ceil(360 / ray_angle);
        radial_division_indices.resize(radial_dividers_num_);
        radial_ordered_clouds.resize(radial_dividers_num_);

        XYZI_to_RTZ(ray_angle, in_cloud, radial_division_indices, radial_ordered_clouds);

        pcl::PointIndices ground_indices, no_ground_indices, road_boundary_indices;
        my_classify_pc(radial_ordered_clouds, ground_indices, initial_distance, max_distance, height_threshold, slope_threshold, no_ground_indices, road_boundary_indices);

        CloudT::Ptr ground_cloud_ptr(new CloudT);
        CloudT::Ptr no_ground_cloud_ptr(new CloudT);

        //std::cout<<"****************************************"<<std::endl;

        pcl::ExtractIndices<PointT> extract_no_ground;
        extract_no_ground.setInputCloud(in_cloud);
        extract_no_ground.setIndices(boost::make_shared<pcl::PointIndices>(no_ground_indices));

        extract_no_ground.setNegative(false); //true removes the indices, false leaves only the indices
        extract_no_ground.filter(*no_ground_cloud_ptr);

        //publish_cloud(pub_no_ground, temp_no_ground_cloud_ptr);

        publish_cloud(pub_no_ground, no_ground_cloud_ptr);

        pcl::ExtractIndices<PointT> extract_ground;
        extract_ground.setInputCloud(in_cloud);
        extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

        extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
        extract_ground.filter(*ground_cloud_ptr);
        publish_cloud(pub_ground, ground_cloud_ptr);
    };
    /*
void Manager::Ray_ProgressiveMorphologicalFilter(const CloudT::Ptr in_cloud)
{
	std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<CloudT> radial_ordered_clouds;

	int radial_dividers_num_ = ceil(360/RADIAL_DIVIDER_ANGLE);
	radial_division_indices.resize(radial_dividers_num_);
    radial_ordered_clouds.resize(radial_dividers_num_);

	XYZI_to_RTZ(in_cloud,radial_division_indices,radial_ordered_clouds);

};
*/
    /*
void Manager::RansacSegmentPlane(const CloudT::Ptr in_cloud)
{
    int num_points = in_cloud->points.size();
    auto cloud_points = in_cloud->points;
    Ransac RansacSeg(10, 0.5, num_points);
	std::unordered_set<int> inliersResult = RansacSeg.Ransac3d(in_cloud);
    CloudT::Ptr ground_cloud_ptr(new CloudT);
    CloudT::Ptr no_ground_cloud_ptr(new CloudT);
    for (int i = 0; i < num_points; i++) {
        PointT pt = cloud_points[i];
        if (inliersResult.count(i)) {
            no_ground_cloud_ptr->points.push_back(pt);
        } else {
            ground_cloud_ptr->points.push_back(pt);
        }
    }
    publish_cloud(pub_ground, ground_cloud_ptr);
    publish_cloud(pub_no_ground, no_ground_cloud_ptr);
};
*/

    void Manager::debugRayGround(const CloudT::Ptr in_cloud)
    {
        std::vector<pcl::PointIndices> radial_division_indices;
        std::vector<PointCloudXYZIRT> radial_ordered_clouds;

        int radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);
        radial_division_indices.resize(radial_dividers_num_);
        radial_ordered_clouds.resize(radial_dividers_num_);
        XYZI_to_RTZ(RADIAL_DIVIDER_ANGLE, in_cloud, radial_division_indices, radial_ordered_clouds);
        int index = ceil(m_param->check_angle / RADIAL_DIVIDER_ANGLE);
        std::vector<st_Point> tempdataPoints;
        //printf("radial_ordered_clouds[0].size() = %ld\n",radial_ordered_clouds[0].size());
        for (size_t j = 0; j < radial_ordered_clouds[index].size(); j++)
        {
            st_Point _point;
            _point.x = radial_ordered_clouds[index][j].radius;
            _point.y = radial_ordered_clouds[index][j].point.z;
            tempdataPoints.push_back(_point);
            // 标定的时候，依据r,z值标定，要求r值增长，z值始终约等于 0.0，表示雷达水平；可以在不同方向上测试
            std::cout << "[r:" << radial_ordered_clouds[index][j].radius << ",z:" << radial_ordered_clouds[index][j].point.z << "],";
        }
        std::cout << std::endl;
        double A = 0;
        double B = 0;
        double C = 0;
        double meanError = 0;
        int datalen = tempdataPoints.size();
        if (!ransacLiner(tempdataPoints, datalen, 2, datalen * 0.5, datalen * 0.6, 0.2, A, B, C, meanError))
        {
            ROS_DEBUG("A = %f,B = %f,C= %f,Error = %f,K = %f,angle = %f\n", A, B, C, meanError, (-A / B), atan(-A / B) * 180 / 3.1415926);
        }
        else
        {
            ROS_DEBUG("RANSAC Failed\n");
        }
        CloudT::Ptr ground_cloud_ptr(new CloudT);
        pcl::ExtractIndices<PointT> extract_ground;
        extract_ground.setInputCloud(in_cloud);
        extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(radial_division_indices[index]));
        extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
        extract_ground.filter(*ground_cloud_ptr);
        if (ground_cloud_ptr->points.size() > 0)
        {
            publish_cloud(pub_ground, ground_cloud_ptr);
        }
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "livox_frame";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.1;
        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
        // Create the vertices for the points and lines
        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.x = 20.0;
        p.y = 0;
        p.z = (0 - (20 * A + C)) / B;
        line_list.points.push_back(p);
        marker_pub.publish(line_list);
    };

    int Manager::ransacLiner(std::vector<st_Point> &pstData, int dataCnt, int minCnt, double maxIterCnt, int consensusCntThreshold, double maxErrorThreshod, double &A, double &B, double &C, double &modelMeanError)
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
        rng.seed(10);                      // 固定随机数种子
        set<unsigned int> selectIndexs;    // 选择的点的索引
        vector<st_Point> selectPoints;     // 选择的点
        set<unsigned int> consensusIndexs; // 满足一致性的点的索引
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
                if (distance < maxErrorThreshod)
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
            }
            iter++;
        }
        return isNonFind;
    }

    void Manager::XYZI_to_RTZ(float ray_angle, const CloudT::Ptr in_cloud,
                              std::vector<pcl::PointIndices> &out_radial_divided_indices,
                              std::vector<PointCloudXYZIRT> &out_radial_ordered_clouds)
    {
        //out_radial_divided_indices.clear();
        //#pragma omp parallel for
        //printf("in_cloud->points.size() = %ld\n",in_cloud->points.size());
        for (size_t i = 0; i < in_cloud->points.size(); i++)
        {
            PointXYZIRT new_point;
            auto radius = (float)sqrt(
                in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
            auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / 3.1415926;
            if (theta < 0)
            {
                theta += 360;
            }
            //角度的微分
            auto radial_div = (size_t)floor(theta / ray_angle);
            //printf("x:%f,y:%f,z:%f\n",in_cloud->points[i].x,in_cloud->points[i].y,in_cloud->points[i].z);
            //auto radial_div = 0;
            new_point.point = in_cloud->points[i];
            new_point.radius = radius;
            new_point.theta = theta;
            new_point.radial_div = radial_div;
            new_point.ground_flag = -1;
            new_point.original_index = i;
            //#pragma omp critical
            {
                out_radial_divided_indices[radial_div].indices.push_back(i);
                out_radial_ordered_clouds[radial_div].push_back(new_point);
            }
        } //end for

        //将同一根射线上的点按照半径（距离）排序
        //#pragma omp for

        for (size_t i = 0; i < out_radial_ordered_clouds.size(); i++)
        {
            std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                      [](const PointXYZIRT &a, const PointXYZIRT &b) { return a.radius < b.radius; });
        }
    }

    void Manager::XYZI_to_RTZ(const CloudT::Ptr in_cloud,
                              std::vector<pcl::PointIndices> &out_radial_divided_indices,
                              std::vector<pcl::PointCloud<PointT>> &out_radial_ordered_clouds)
    {
        //out_radial_divided_indices.clear();
        //#pragma omp parallel for
        for (size_t i = 0; i < in_cloud->points.size(); i++)
        {
            auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / 3.1415926;
            if (theta < 0)
            {
                theta += 360;
            }
            //角度的微分
            auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
            //#pragma omp critical
            {
                out_radial_divided_indices[radial_div].indices.push_back(i);
                out_radial_ordered_clouds[radial_div].push_back(in_cloud->points[i]);
            }
        } //end for
    }

    void Manager::classify_pc(std::vector<PointCloudXYZIRT> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              float he, float hg,
                              pcl::PointIndices &out_no_ground_indices,
                              pcl::PointIndices &out_road_boundary_indices)
    {

        out_ground_indices.indices.clear();
        out_no_ground_indices.indices.clear();
        out_road_boundary_indices.indices.clear();
        //#pragma omp for
        float Hg = hg;
        float He = he;
        for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
        {
            // 一段点云集计算
            float min_z = std::numeric_limits<float>::max();
            float max_z = -std::numeric_limits<float>::max();
            float start_radius = 0.0f;
            float end_radius = 0.0f;
            std::vector<PointXYZIRT> tmpPointXYZIRT;
            bool first_no_ground_point = true;
            for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
            {
                float points_distance = 0.0f;
                if (j != 0)
                {
                    points_distance = in_radial_ordered_clouds[i][j].radius - start_radius;
                }
                else
                {
                    start_radius = in_radial_ordered_clouds[i][j].radius;
                }

                if ((points_distance <= 0.5))
                {
                    if (in_radial_ordered_clouds[i][j].point.z < min_z)
                    {
                        min_z = in_radial_ordered_clouds[i][j].point.z;
                    }
                    if (in_radial_ordered_clouds[i][j].point.z > max_z)
                    {
                        max_z = in_radial_ordered_clouds[i][j].point.z;
                    }
                    end_radius = in_radial_ordered_clouds[i][j].radius;
                    tmpPointXYZIRT.push_back(in_radial_ordered_clouds[i][j]);
                }
                else
                {
                    float _hg = max_z - min_z;
                    float _he = min_z - 0.0;
                    float _base_ground_height = ((start_radius + end_radius) / 2) * ground_k + ground_base;
                    if (((_hg <= Hg) && (_he <= (He + _base_ground_height))) || (max_z < _base_ground_height))
                    {
                        for (size_t k = 0; k < tmpPointXYZIRT.size(); k++)
                        {
                            out_ground_indices.indices.push_back(tmpPointXYZIRT[k].original_index);
                        }
                    }
                    else
                    {
                        if (tmpPointXYZIRT.size() > 1)
                        {
                            for (size_t k = 0; k < tmpPointXYZIRT.size(); k++)
                            {
                                out_no_ground_indices.indices.push_back(tmpPointXYZIRT[k].original_index);
                            }
                            if (first_no_ground_point) // 第一个非地面点
                            {
                                out_road_boundary_indices.indices.push_back(tmpPointXYZIRT[0].original_index);
                                first_no_ground_point = false;
                            }
                        }
                    }
                    tmpPointXYZIRT.clear();
                    min_z = std::numeric_limits<float>::max();
                    max_z = -std::numeric_limits<float>::max();
                    if (in_radial_ordered_clouds[i][j].point.z < min_z)
                    {
                        min_z = in_radial_ordered_clouds[i][j].point.z;
                    }
                    if (in_radial_ordered_clouds[i][j].point.z > max_z)
                    {
                        max_z = in_radial_ordered_clouds[i][j].point.z;
                    }
                    start_radius = in_radial_ordered_clouds[i][j].radius;
                    tmpPointXYZIRT.push_back(in_radial_ordered_clouds[i][j]);
                }
            }
            if (!tmpPointXYZIRT.empty())
            {
                float _hg = max_z - min_z;
                float _he = min_z - 0.0;
                float _base_ground_height = ((start_radius + end_radius) / 2) * ground_k + ground_base;
                if (((_hg <= Hg) && (_he <= (He + _base_ground_height))) || (max_z < _base_ground_height))
                {
                    for (size_t k = 0; k < tmpPointXYZIRT.size(); k++)
                    {
                        out_ground_indices.indices.push_back(tmpPointXYZIRT[k].original_index);
                    }
                }
                else
                {
                    if (tmpPointXYZIRT.size() > 1)
                    {
                        for (size_t k = 0; k < tmpPointXYZIRT.size(); k++)
                        {
                            out_no_ground_indices.indices.push_back(tmpPointXYZIRT[k].original_index);
                        }
                        if (first_no_ground_point) // 第一个非地面点
                        {
                            out_road_boundary_indices.indices.push_back(tmpPointXYZIRT[0].original_index);
                            first_no_ground_point = false;
                        }
                    }
                }
                tmpPointXYZIRT.clear();
            }
        }
    }

    double distance(PointT &first, PointT &second)
    {
        float x = first.x - second.x;
        float y = first.y - second.y;
        float z = first.z - second.z;
        return sqrt(x * x + y * y + z * z);
    }

    /** 
 * @brief 判断输入点是否是地面点
 * @param ground_points       一组需要判断的点云
 * @param index               判断地面点参考已知地面点集中的点序号
 * @param pointxyzrt          需要判断的点
 * @param initial_distance    初始地面高度
 * @param max_distance        参考点与判断最远距离
 * @param height_threshold    高度阀值：当判断点与参考点距离超过max_distance，按照height_threshold判断
 * @param slope_threshold     坡度阀值：输入的是角度
 * @return 返回说明
 *     -<em>false</em> 非地面点
 *     -<em>true</em> 地面点
 */
    bool Manager::check_point_is_ground(std::vector<PointXYZIRT> &ground_points, int index, PointXYZIRT &pointxyzrt, double initial_distance, double max_distance, double height_threshold, double slope_threshold)
    {
        if (pointxyzrt.point.z < initial_distance)
        {
            return true;
        }
        bool is_ground_point = false;
        if (index > 0)
        {
            double _height_threshold = tan(DEG2RAD(slope_threshold)) * max_distance;
            height_threshold = (height_threshold > _height_threshold) ? height_threshold : _height_threshold;
            if (ground_points[index - 1].ground_flag == 1) // 1标记为非地面点
            {
                is_ground_point = check_point_is_ground(ground_points, index - 1, pointxyzrt, initial_distance, max_distance, height_threshold, slope_threshold);
            }
            else // ground_flag = 0,不能为-1，因为从前往后按顺序判断，前面的点一定已经有了结果
            {
                double prev_ground_points_radius = pointxyzrt.radius - ground_points[index - 1].radius;
                double prev_ground_point_height = ground_points[index - 1].point.z;
                double prev_height_threshold = tan(DEG2RAD(slope_threshold)) * prev_ground_points_radius;
                double points_distacne = distance(ground_points[index - 1].point, pointxyzrt.point);
                double current_height = pointxyzrt.point.z;
                //printf("ground_points[index-1].radius = %f,pointxyzrt.radius = %f\n",ground_points[index-1].radius,pointxyzrt.radius);
                //printf("prev_ground_points_radius = %f\n",prev_ground_points_radius);
                //printf("prev_height_threshold = %f\n",prev_height_threshold);
                if (points_distacne < 0.05)
                {
                    is_ground_point = true;
                }
                else
                {
                    if (prev_ground_points_radius < max_distance)
                    {
                        if (current_height < (prev_ground_point_height + prev_height_threshold))
                        //if (current_height < (prev_ground_point_height + prev_height_threshold) && current_height > (prev_ground_point_height - prev_height_threshold)
                        {
                            is_ground_point = true;
                        }
                        else
                        {
                            is_ground_point = check_point_is_ground(ground_points, index - 1, pointxyzrt, initial_distance, max_distance, height_threshold, slope_threshold);
                        }
                    }
                    else
                    {
                        // 与前一个地面点比较
                        /*if ( current_height < prev_ground_height + height_threshold )
					{
						is_ground_point = true;
					}else
					{*/
                        is_ground_point = false;
                        //}
                    }
                }
            }
        }
        else
        {
            /*if (flag)
		{
			is_ground_point = false;
		}else
		{*/
            double prev_ground_points_radius = pointxyzrt.radius - 0.0f;
            double prev_ground_point_height = 0.0f;
            double prev_height_threshold = tan(DEG2RAD(slope_threshold)) * prev_ground_points_radius;
            double current_height = pointxyzrt.point.z;
            //if ((current_height < (prev_ground_point_height + prev_height_threshold) && current_height > (prev_ground_point_height - prev_height_threshold))||(current_height < initial_distance))
            if (current_height < (prev_ground_point_height + prev_height_threshold))
            {
                //printf("66\n");
                is_ground_point = true;
            }
            else
            {
                //printf("77\n");
                is_ground_point = false;
            }
            //}
        }
        return is_ground_point;
    };

    bool Manager::reverse_check_point_is_ground(std::vector<PointXYZIRT> &ground_points, int index, int points_size, PointXYZIRT &pointxyzrt, double initial_distance, double max_distance, double height_threshold, double slope_threshold)
    {
        if ((pointxyzrt.point.z < initial_distance) || (pointxyzrt.ground_flag == 0))
        {
            return true;
        }
        bool is_ground_point = false;
        if (index < points_size)
        {
            double _height_threshold = tan(DEG2RAD(slope_threshold)) * max_distance;
            height_threshold = (height_threshold > _height_threshold) ? height_threshold : _height_threshold;
            if (ground_points[index + 1].ground_flag == 1) // 1标记为非地面点
            {
                is_ground_point = reverse_check_point_is_ground(ground_points, index + 1, points_size, pointxyzrt, initial_distance, max_distance, height_threshold, slope_threshold);
            }
            else // ground_flag = 0,不能为-1，因为已经经过了一轮判断，所有点一定已经有了结果
            {
                double prev_ground_points_radius = ground_points[index + 1].radius - pointxyzrt.radius;
                double prev_ground_point_height = ground_points[index + 1].point.z;
                double prev_height_threshold = tan(DEG2RAD(slope_threshold)) * prev_ground_points_radius;
                double points_distacne = distance(ground_points[index + 1].point, pointxyzrt.point);
                double current_height = pointxyzrt.point.z;
                if (points_distacne < 0.05)
                {
                    is_ground_point = true;
                }
                else
                {
                    if (prev_ground_points_radius < max_distance)
                    {
                        if (current_height < (prev_ground_point_height + prev_height_threshold))
                        //if (current_height < (prev_ground_point_height + prev_height_threshold) && current_height > (prev_ground_point_height - prev_height_threshold))
                        {
                            is_ground_point = true;
                        }
                        else
                        {
                            is_ground_point = reverse_check_point_is_ground(ground_points, index + 1, points_size, pointxyzrt, initial_distance, max_distance, height_threshold, slope_threshold);
                        }
                    }
                    else
                    {
                        is_ground_point = false;
                    }
                }
            }
        }
        else
        {
            if (pointxyzrt.ground_flag == 0)
            {
                is_ground_point = true;
            }
            else
            {
                is_ground_point = false;
            }
        }
        return is_ground_point;
    }

    void Manager::my_classify_pc(std::vector<PointCloudXYZIRT> &in_radial_ordered_clouds,
                                 pcl::PointIndices &out_ground_indices,
                                 double initial_distance, double max_distance, double height_threshold, double slope_threshold,
                                 pcl::PointIndices &out_no_ground_indices,
                                 pcl::PointIndices &out_road_boundary_indices)
    {

        out_ground_indices.indices.clear();
        out_no_ground_indices.indices.clear();
        out_road_boundary_indices.indices.clear();
        for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
        {
            bool current_ground = false;

            // 从前往后判断一次
            size_t points_size = in_radial_ordered_clouds[i].size();
            for (size_t j = 0; j < points_size; j++)
            {
                current_ground = check_point_is_ground(in_radial_ordered_clouds[i], j, in_radial_ordered_clouds[i][j], initial_distance, max_distance, height_threshold, slope_threshold);
                if (current_ground)
                {
                    in_radial_ordered_clouds[i][j].ground_flag = 0;
                }
                else
                {
                    in_radial_ordered_clouds[i][j].ground_flag = 1;
                }
            }
            // 从后向前验证一次
            for (size_t j = points_size; j > 0; j--)
            {
                current_ground = reverse_check_point_is_ground(in_radial_ordered_clouds[i], j - 1, points_size - 1, in_radial_ordered_clouds[i][j - 1], initial_distance, max_distance, height_threshold, slope_threshold);
                if (current_ground)
                {
                    //in_radial_ordered_clouds[i][j-1].ground_flag = 0;
                    out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j - 1].original_index);
                    //std::cout<<"radius:"<<in_radial_ordered_clouds[i][j-1].radius<<",x:"<<in_radial_ordered_clouds[i][j-1].point.x<<",y:"<<in_radial_ordered_clouds[i][j-1].point.y<<",z:"<<in_radial_ordered_clouds[i][j-1].point.z<<"------ ground"<<std::endl;
                }
                else
                {
                    //in_radial_ordered_clouds[i][j-1].ground_flag = 1;
                    out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j - 1].original_index);
                    //std::cout<<"radius:"<<in_radial_ordered_clouds[i][j-1].radius<<",x:"<<in_radial_ordered_clouds[i][j-1].point.x<<",y:"<<in_radial_ordered_clouds[i][j-1].point.y<<",z:"<<in_radial_ordered_clouds[i][j-1].point.z<<"------ no ground"<<std::endl;
                }
            }
        }
    }

    void Manager::publish_cloud(const ros::Publisher &in_publisher,
                                const CloudT::Ptr in_cloud_to_publish_ptr)
    {
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
        cloud_msg.header.frame_id = "livox_frame";
        in_publisher.publish(cloud_msg);
        //ROS_INFO("segment points.size() = %ld publish_cloud %ld",in_cloud_to_publish_ptr->points.size(),in_cloud_to_publish_ptr->header.stamp);
    }
    // 去除nan点
    void Manager::removeNan(CloudT::Ptr in, CloudT::Ptr out)
    {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*in, *out, indices);
    }

    // 八叉树下采样
    void Manager::octreeVoxelGrid(float leafsize, CloudT::Ptr in, CloudT::Ptr out)
    {
        pcl::octree::OctreePointCloudSearch<PointT> octree(10.0);
        octree.setInputCloud(in);
        octree.defineBoundingBox();
        //octree.enableDynamicDepth (leafAggSize);
        octree.addPointsFromInputCloud();
        pcl::octree::OctreePointCloudPointVector<PointT>::Iterator it;
        pcl::octree::OctreePointCloudPointVector<PointT>::Iterator it_end = octree.leaf_end();
        pcl::IndicesPtr indexVector(new vector<int>);
        for (it = octree.leaf_begin(); it != it_end; ++it)
        {
            const pcl::octree::OctreeNode *node = it.getCurrentOctreeNode();
            if (node->getNodeType() == pcl::octree::LEAF_NODE)
            {
                CloudT::Ptr grid_voxel_cloud = boost::make_shared<CloudT>();
                const pcl::octree::OctreeContainerPointIndices &container = it.getLeafContainer();
                std::vector<int> points_indices;
                container.getPointIndices(points_indices);
                pcl::IndicesPtr indexVector = boost::make_shared<std::vector<int>>(points_indices);
                pcl::VoxelGrid<PointT> pGrid;
                pGrid.setLeafSize(leafsize, leafsize, leafsize);
                pGrid.setInputCloud(in);
                pGrid.setIndices(indexVector);
                pGrid.filter(*grid_voxel_cloud);
                *out += *grid_voxel_cloud;
            }
        }
    };

    // 去掉 近原点的点
    void Manager::remove_close_pt(double min_distance, const CloudT::Ptr in,
                                  const CloudT::Ptr out)
    {
        pcl::ExtractIndices<PointT> cliper;

        cliper.setInputCloud(in);
        pcl::PointIndices indices;
#pragma omp parallel for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

            if (distance < min_distance)
            {
#pragma omp critical
                indices.indices.push_back(i);
            }
        }
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        cliper.setNegative(true); //ture to remove the indices
        cliper.filter(*out);
    }

    // 去掉车身范围点
    void Manager::remove_close_pt(double minx, double miny, double maxx, double maxy, const CloudT::Ptr in, const CloudT::Ptr out)
    {
        skywell::CropBoxFilter crop;
        crop.setMinP(minx, miny, -100.0);
        crop.setMaxP(maxx, maxy, 100.0);
        crop.setNegative(true);
        crop.filter(in, out);
    }

    // 消减 上面点
    void Manager::clip_above(double clip_height, const CloudT::Ptr in,
                             const CloudT::Ptr out)
    {
        pcl::ExtractIndices<PointT> cliper;

        cliper.setInputCloud(in);
        pcl::PointIndices indices;
#pragma omp parallel for
        for (size_t i = 0; i < in->points.size(); i++)
        {
            if (in->points[i].z > clip_height)
            {
#pragma omp critical
                indices.indices.push_back(i);
            }
        }
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        cliper.setNegative(true); //ture to remove the indices
        cliper.filter(*out);
    }

    /*
bool estimateGroundPlane(PointCloudXYZI::Ptr &in_cloud, PointCloudXYZI::Ptr &out_cloud,
                                   visualization_msgs::MarkerPtr &plane_marker, const float in_distance_thre)
{
    //plane segmentation
    pcl::SACSegmentation<PointT> plane_seg;
    pcl::PointIndices::Ptr plane_inliers ( new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr plane_coefficients ( new pcl::ModelCoefficients );
    plane_seg.setOptimizeCoefficients (true);
    plane_seg.setModelType ( pcl::SACMODEL_PLANE );
    plane_seg.setMethodType ( pcl::SAC_RANSAC );
    plane_seg.setDistanceThreshold ( in_distance_thre );
    plane_seg.setInputCloud ( in_cloud );
    plane_seg.segment ( *plane_inliers, *plane_coefficients );
    return true;
}*/

#define PAI 3.141592653
    void Manager::my_segment(CloudT::Ptr cloud)
    {
        float resolution = 1.0f;
        float Hg = 0.2f;
        float He = 0.8f;
        float Angle = 2.0f;
        pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        //octree.enableDynamicDepth (leafAggSize);
        octree.addPointsFromInputCloud();
        pcl::octree::OctreePointCloudPointVector<PointT>::Iterator it;
        pcl::octree::OctreePointCloudPointVector<PointT>::Iterator it_end = octree.leaf_end();
        unsigned int leaf_count = 0;
        double minx, miny, minz, maxx, maxy, maxz;
        octree.getBoundingBox(minx, miny, minz, maxx, maxy, maxz);
        //std::vector<int> indexVector;
        //pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        std::cout << "minx:" << minx << ",miny:" << miny << ",minz:" << minz << std::endl;
        std::cout << "maxx:" << maxx << ",maxy:" << maxy << ",maxy:" << maxy << std::endl;
        std::cout << "GetBranchCount() = " << octree.getBranchCount() << endl;
        std::cout << "GetLeafCount() = " << octree.getLeafCount() << endl;

        for (it = octree.leaf_begin(); it != it_end; ++it)
        {
            Eigen::Vector3f min_pt;
            Eigen::Vector3f max_pt;
            octree.getVoxelBounds(it, min_pt, max_pt);
            //std::cout<<"min_pt:"<<min_pt<<",max_pit:"<<max_pt<<endl;
            //std::cout<<"GetCurrentOctreeDepth() = "<<it.getCurrentOctreeDepth()<<endl;
            //std::cout<<"getNodeID() = "<<it.getNodeID()<<endl;

            const pcl::octree::OctreeNode *node = it.getCurrentOctreeNode();
            if (node->getNodeType() == pcl::octree::LEAF_NODE)
            {
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                const pcl::octree::OctreeContainerPointIndices &container = it.getLeafContainer();
                std::vector<int> tmpVector;
                container.getPointIndices(tmpVector);
                for (int i = 0; i < tmpVector.size(); i++)
                {
                    inliers->indices.push_back(tmpVector[i]);
                }

                if (inliers->indices.size() > 0)
                {
                    // 提取剩余点云
                    CloudT::Ptr myCloud(new CloudT);
                    pcl::ExtractIndices<PointT> extract;
                    // Extract the inliers
                    extract.setInputCloud(cloud);
                    extract.setIndices(inliers);
                    extract.setNegative(true);
                    extract.filter(*myCloud);

                    sensor_msgs::PointCloud2 plane_cloud;
                    pcl::toROSMsg(*myCloud, plane_cloud);
                    plane_cloud.header.frame_id = "livox_frame";
                    pub_ground.publish(plane_cloud);
                }

                float min_z = std::numeric_limits<float>::max();
                float max_z = -std::numeric_limits<float>::max();
                int min_p_i = 0;
                int max_p_i = 0;
                /*for (int i = 0; i < tmpVector.size() ; i++)
			{
				int index = tmpVector[i];
				if (cloud->points[index].z  < min_z)
				{
					min_z = cloud->points[index].z;
					min_p_i = index;
				}
				if (cloud->points[index].z  > max_z)
				{
					max_z = cloud->points[index].z;
					max_p_i = index;
				}
			}
			float _hg = max_z - min_z;
			float _he = max_z - 0.0;
			if (((_hg > Hg )||(_he > He))) 
			{
					for (int i = 0;i < tmpVector.size() ; i++)
					{
						inliers->indices.push_back(tmpVector[i]);
					}
			}*/
            }
            else
            {
            }
            //sleep(1);
        }
        //printf("\n");
        /*
	if (inliers->indices.size() > 0)
	{
		// 提取剩余点云
		CloudT::Ptr myCloud (new CloudT);
		pcl::ExtractIndices<PointT> extract;
		// Extract the inliers
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*myCloud);

		sensor_msgs::PointCloud2 plane_cloud;
		pcl::toROSMsg(*myCloud, plane_cloud);
		plane_cloud.header.frame_id = "livox_frame";
		pub_ground.publish(plane_cloud);
	}*/
        //printf("leaf_count = %d\n",leaf_count);
    };

    /*
void Manager::ObjectToClusterMsg(boost::shared_ptr<std::vector<skywell::Object>> objs,perception::Cluster &cluster)
{
	for (auto& obj : *objs) {
		perception::Object  _object;
		_object.center.pos[0] = obj.transform(0);
		_object.center.pos[1] = obj.transform(1);
		_object.center.pos[2] = obj.transform(2);
		_object.width = obj.width;
		_object.height = obj.height;
		_object.depth = obj.depth;
		_object.rotate.x = obj.rotate.x();
		_object.rotate.y = obj.rotate.y();
		_object.rotate.z = obj.rotate.z();
		_object.rotate.w = obj.rotate.w();
		cluster.object.push_back(_object);
	}
};


void Manager::ToObstSet(ClusterVectorPtr midsave,perception::ObstSet &obstset,uint64_t stamp)
{
	for (size_t i = 0;i < midsave->size() ;i++ )
	{
		perception::ObstPointCloud _obstpointcloud;
		for (int index = 0; index < (*(*midsave)[i]).size() ;index++ )
		{
			perception::PointXYZI pointt;
			pointt.x = ((*(*midsave)[i])[index]).x;
			pointt.y = ((*(*midsave)[i])[index]).y;
			pointt.z = ((*(*midsave)[i])[index]).z;
			pointt.intensity = ((*(*midsave)[i])[index]).intensity;
			_obstpointcloud.points.push_back(pointt);
		}
		ros::Time ros_stamp((stamp/1000000),(stamp%1000000));
		_obstpointcloud.header.stamp = ros_stamp;
		obstset.obsts.push_back(_obstpointcloud);
	}
		printf("障碍物数目：%ld\n",midsave->size());
};
*/

    void Manager::plane_segment(const CloudT::Ptr in_cloud)
    {
        //平面模型分割点云
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);

        seg.setDistanceThreshold(0.2);
        seg.setInputCloud(in_cloud);
        seg.setMaxIterations(50);
        seg.setProbability(0.95);
        seg.segment(*inliers, *coefficients);
        if (0 == inliers->indices.size())
        {
            std::cout << "could not estimate a planar model for the given dataset." << std::endl;
        }
        CloudT::Ptr no_ground_cloud_ptr = boost::make_shared<CloudT>();
        CloudT::Ptr ground_cloud_ptr = boost::make_shared<CloudT>();

        // 提取点云地面
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(in_cloud);
        extract.setIndices(boost::make_shared<const pcl::PointIndices>(*inliers)); //设置索引
        extract.filter(*ground_cloud_ptr);
        // 障碍物点云提取
        extract.setNegative(true);
        extract.filter(*no_ground_cloud_ptr);

        publish_cloud(pub_ground, ground_cloud_ptr);
        publish_cloud(pub_no_ground, no_ground_cloud_ptr);
    };

    void Manager::grid_segment(const CloudT::Ptr in_cloud)
    {
        CloudT::Ptr no_ground_cloud_ptr = boost::make_shared<CloudT>();
        CloudT::Ptr ground_cloud_ptr = boost::make_shared<CloudT>();
        pcl::PointIndices out_indices;

        skywell::GridSegment _grid_segment;
        _grid_segment.setGridRange(-30, -30, 30, 30);
        _grid_segment.setGridSize(1.0);
        _grid_segment.initGrid();

        _grid_segment.setInputCloud(in_cloud);
        _grid_segment.processGrid();
        _grid_segment.setHeightThreshold(0.2);
        _grid_segment.segment(ground_grid_list, out_indices);

        // 提取点云地面
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(in_cloud);
        extract.setIndices(boost::make_shared<const pcl::PointIndices>(out_indices)); //设置索引
        extract.filter(*ground_cloud_ptr);
        // 障碍物点云提取
        extract.setNegative(true);
        extract.filter(*no_ground_cloud_ptr);

        /*CloudT::Ptr tmp_no_ground_cloud_ptr = boost::make_shared<CloudT>();

	//提取 
	skywell::CropBoxFilter crop;
	crop.setMinP(1.0, -1.0, -1.0);
	crop.setMaxP(10.0, 5.0, 1.0);
	crop.setNegative(false);
	crop.filter(no_ground_cloud_ptr,tmp_no_ground_cloud_ptr);

	
	printf("points.size() == %ld\n",tmp_no_ground_cloud_ptr->points.size());
	for (int i = 0; i < tmp_no_ground_cloud_ptr->points.size() ;i++ )
	{
		printf("x:%f,y:%f,z:%f\n",tmp_no_ground_cloud_ptr->points[i].x,
			tmp_no_ground_cloud_ptr->points[i].y,tmp_no_ground_cloud_ptr->points[i].z);
	}
		printf("\n\n");*/
        publish_cloud(pub_ground, ground_cloud_ptr);
        publish_cloud(pub_no_ground, no_ground_cloud_ptr);
    };

    void Manager::ProgressiveMorphologicalFilter(const CloudT::Ptr in_cloud)
    {
        float max_w_s = 2.0f;
        float slope = 0.3f;
        float initial_d = 0.2f;
        float max_d = 0.2f;
        float cell_size = 0.1f;
        CloudT::Ptr no_ground_cloud_ptr = boost::make_shared<CloudT>();
        CloudT::Ptr ground_cloud_ptr = boost::make_shared<CloudT>();
        pcl::PointIndicesPtr ground(new pcl::PointIndices);

        // Create the filtering object
        pcl::ProgressiveMorphologicalFilter<PointT> pmf;
        pmf.setInputCloud(in_cloud);
        pmf.setMaxWindowSize(max_w_s);
        pmf.setSlope(slope);
        pmf.setCellSize(cell_size);
        //pmf.setBase(2.0f);
        pmf.setInitialDistance(initial_d);
        pmf.setMaxDistance(max_d);
        pmf.extract(ground->indices);
        //Create the filtering object
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(in_cloud);
        extract.setIndices(ground);
        extract.filter(*ground_cloud_ptr);

        extract.setNegative(true);
        extract.filter(*no_ground_cloud_ptr);

        publish_cloud(pub_ground, ground_cloud_ptr);
        publish_cloud(pub_no_ground, no_ground_cloud_ptr);
    }

    /*
void::Manager gridFilter(const CloudT::Ptr in_cloud);
{
	float grid_size = 0.2f;
	float height_threshold = 0.5f;

}*/

    /*


template <typename PointT> void
pcl::applyMorphologicalOperator (const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
                                 float resolution, const int morphological_operator,
                                 pcl::PointCloud<PointT> &cloud_out)
{
  if (cloud_in->empty ())
    return;

  pcl::copyPointCloud<PointT, PointT> (*cloud_in, cloud_out);

  pcl::octree::OctreePointCloudSearch<PointT> tree (resolution);

  tree.setInputCloud (cloud_in);
  tree.addPointsFromInputCloud ();

  float half_res = resolution / 2.0f;

  switch (morphological_operator)
  {
    case MORPH_DILATE:
    case MORPH_ERODE:
    {
      for (size_t p_idx = 0; p_idx < cloud_in->points.size (); ++p_idx)
      {
        Eigen::Vector3f bbox_min, bbox_max;
        std::vector<int> pt_indices;
        float minx = cloud_in->points[p_idx].x - half_res;
        float miny = cloud_in->points[p_idx].y - half_res;
        float minz = -std::numeric_limits<float>::max ();
        float maxx = cloud_in->points[p_idx].x + half_res;
        float maxy = cloud_in->points[p_idx].y + half_res;
        float maxz = std::numeric_limits<float>::max ();
        bbox_min = Eigen::Vector3f (minx, miny, minz);
        bbox_max = Eigen::Vector3f (maxx, maxy, maxz);
        tree.boxSearch (bbox_min, bbox_max, pt_indices);

        if (pt_indices.size () > 0)
        {
          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D<PointT> (*cloud_in, pt_indices, min_pt, max_pt);

          switch (morphological_operator)
          {
            case MORPH_DILATE:
            {
              cloud_out.points[p_idx].z = max_pt.z ();
              break;
            }
            case MORPH_ERODE:
            {
              cloud_out.points[p_idx].z = min_pt.z ();
              break;
            }
          }
        }
      }
      break;
    }
    case MORPH_OPEN:
    case MORPH_CLOSE:
    {
      pcl::PointCloud<PointT> cloud_temp;

      pcl::copyPointCloud<PointT, PointT> (*cloud_in, cloud_temp);

      for (size_t p_idx = 0; p_idx < cloud_temp.points.size (); ++p_idx)
      {
        Eigen::Vector3f bbox_min, bbox_max;
        std::vector<int> pt_indices;
        float minx = cloud_temp.points[p_idx].x - half_res;
        float miny = cloud_temp.points[p_idx].y - half_res;
        float minz = -std::numeric_limits<float>::max ();
        float maxx = cloud_temp.points[p_idx].x + half_res;
        float maxy = cloud_temp.points[p_idx].y + half_res;
        float maxz = std::numeric_limits<float>::max ();
        bbox_min = Eigen::Vector3f (minx, miny, minz);
        bbox_max = Eigen::Vector3f (maxx, maxy, maxz);
        tree.boxSearch (bbox_min, bbox_max, pt_indices);

        if (pt_indices.size () > 0)
        {
          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D<PointT> (cloud_temp, pt_indices, min_pt, max_pt);

          switch (morphological_operator)
          {
            case MORPH_OPEN:
            {
              cloud_out.points[p_idx].z = min_pt.z ();
              break;
            }
            case MORPH_CLOSE:
            {
              cloud_out.points[p_idx].z = max_pt.z ();
              break;
            }
          }
        }
      }

      cloud_temp.swap (cloud_out);

      for (size_t p_idx = 0; p_idx < cloud_temp.points.size (); ++p_idx)
      {
        Eigen::Vector3f bbox_min, bbox_max;
        std::vector<int> pt_indices;
        float minx = cloud_temp.points[p_idx].x - half_res;
        float miny = cloud_temp.points[p_idx].y - half_res;
        float minz = -std::numeric_limits<float>::max ();
        float maxx = cloud_temp.points[p_idx].x + half_res;
        float maxy = cloud_temp.points[p_idx].y + half_res;
        float maxz = std::numeric_limits<float>::max ();
        bbox_min = Eigen::Vector3f (minx, miny, minz);
        bbox_max = Eigen::Vector3f (maxx, maxy, maxz);
        tree.boxSearch (bbox_min, bbox_max, pt_indices);

        if (pt_indices.size () > 0)
        {
          Eigen::Vector4f min_pt, max_pt;
          pcl::getMinMax3D<PointT> (cloud_temp, pt_indices, min_pt, max_pt);

          switch (morphological_operator)
          {
            case MORPH_OPEN:
            default:
            {
              cloud_out.points[p_idx].z = max_pt.z ();
              break;
            }
            case MORPH_CLOSE:
            {
              cloud_out.points[p_idx].z = min_pt.z ();
              break;
            }
          }
        }
      }
      break;
    }
    default:
    {
      PCL_ERROR ("Morphological operator is not supported!\n");
      break;
    }
  }

  return;
}*/

} // namespace skywell
