#include "param.h"

namespace skywell
{
    Param::Param()
    {
        m_radar_num = 1;
        m_leafsize = 0.1;
        m_clustertype = 0;
    }

    Param::~Param()
    {
    }

    void Param::setDistanceMap(int distance, double value)
    {
        std::map<int, double>::iterator iter;
        iter = m_distance_map.find(distance);
        if (iter != m_distance_map.end())
        {
            iter->second = value;
            return;
        }
        m_distance_map.insert(std::make_pair(distance, value));
    };

    //数字转字符串
    std::string i2s(int number)
    {
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
    static bool DEBUG_PRINT = true;
    int Param::loadcfg(void)
    {
        int ret = -1;
        TCfgFileParser cfg;
        ret = cfg.parseFile("./conf/perception.cfg");
        if (ret != 0)
        {
            //配置文件加载失败
            ROS_INFO("pcluster_seg loadcfg error ,exit(1)!\n");
            return -1;
        }
        sub_ground = cfg.getValue("CLUSTER_NODE", "sub_ground");
        sub_no_ground = cfg.getValue("CLUSTER_NODE", "sub_no_ground");
        pub_obst_set = cfg.getValue("CLUSTER_NODE", "pub_obst");
        cluster_model = atoi(cfg.getValue("CLUSTER_NODE", "cluster_model").c_str());

        eucliden_param = cfg.getValue("CLUSTER_NODE", "eucliden");
        string value = "";
        value = findkeyvalue(eucliden_param, "minsize");
        if (value != "")
        {
            m_min_size = atoi(value.c_str());
        }
        value = findkeyvalue(eucliden_param, "maxsize");
        if (value != "")
        {
            m_max_size = atoi(value.c_str());
        }
        value = findkeyvalue(eucliden_param, "rounds");
        if (value != "")
        {
            int rounds = atoi(value.c_str());
            string tmpsource = eucliden_param;
            for (int i = 0; i < rounds; i++)
            {
                int radius = 0;
                float distacne = 0.0f;
                string::size_type loc1 = tmpsource.find("radius");
                tmpsource = tmpsource.substr(loc1);
                value = findkeyvalue(tmpsource, "radius");
                if (value != "")
                {
                    radius = atoi(value.c_str());
                }
                tmpsource = tmpsource.substr(string("radius").size() + value.size() + 1);
                value = findkeyvalue(tmpsource, "distance");
                if (value != "")
                {
                    distacne = atof(value.c_str());
                }
                tmpsource = tmpsource.substr(string("distance").size() + value.size() + 1);
                setDistanceMap(radius, distacne);
            }
        }
        dbscan_param = cfg.getValue("CLUSTER_NODE", "dbscan");
        value = findkeyvalue(dbscan_param, "minnum");
        if (value != "")
        {
            m_min_num = atoi(value.c_str());
        }
        value = findkeyvalue(dbscan_param, "aroundnum");
        if (value != "")
        {
            m_around_num = atoi(value.c_str());
        }
        value = findkeyvalue(dbscan_param, "radius");
        if (value != "")
        {
            m_radius = atof(value.c_str());
        }

        pub_neareast_points = cfg.getValue("LANE_DETECTION_NODE", "pub_neareast_points");
        ray_angle_for_lane_detection = atof(cfg.getValue("LANE_DETECTION_NODE","ray_angle_for_lane_detection").c_str());
        lane_region_miny = atof(cfg.getValue("LANE_DETECTION_NODE", "lane_region_miny").c_str());
        lane_region_maxy = atof(cfg.getValue("LANE_DETECTION_NODE", "lane_region_maxy").c_str());
        lane_region_size = atof(cfg.getValue("LANE_DETECTION_NODE", "lane_region_size").c_str());
        lane_clip_height = atof(cfg.getValue("LANE_DETECTION_NODE", "lane_clip_height").c_str());


        if (DEBUG_PRINT)
        {
            ROS_INFO("########################LANE_DETECTION_NODE#########################\n");
            ROS_INFO("ray_angle_for_lane_detection:%f\n", ray_angle_for_lane_detection);
            ROS_INFO("lane_region_miny:%f\n", lane_region_miny);
            ROS_INFO("lane_region_maxy:%f\n", lane_region_maxy);
            ROS_INFO("lane_region_size:%f\n", lane_region_size);
            ROS_INFO("lane_clip_height:%f\n", lane_clip_height);

            DEBUG_PRINT = false;
        }
        return 0;
    };

    void Param::loadparam(ros::NodeHandle node, ros::NodeHandle private_nh)
    {
        private_nh.param("cluster_model", cluster_model, int(0));
        private_nh.param("sub_ground", sub_ground, std::string(""));
        private_nh.param("sub_no_ground", sub_no_ground, std::string(""));
        private_nh.param("pub_obst_set", pub_obst_set, std::string(""));
        private_nh.param("eucliden", eucliden_param, std::string(""));

        string value = "";
        value = findkeyvalue(eucliden_param, "minsize");
        if (value != "")
        {
            m_min_size = atoi(value.c_str());
        }
        value = findkeyvalue(eucliden_param, "maxsize");
        if (value != "")
        {
            m_max_size = atoi(value.c_str());
        }
        value = findkeyvalue(eucliden_param, "rounds");
        if (value != "")
        {
            int rounds = atoi(value.c_str());
            string tmpsource = eucliden_param;
            for (int i = 0; i < rounds; i++)
            {
                int radius = 0;
                float distacne = 0.0f;
                string::size_type loc1 = tmpsource.find("radius");
                tmpsource = tmpsource.substr(loc1);
                value = findkeyvalue(tmpsource, "radius");
                if (value != "")
                {
                    radius = atoi(value.c_str());
                }
                tmpsource = tmpsource.substr(string("radius").size() + value.size() + 1);
                value = findkeyvalue(tmpsource, "distance");
                if (value != "")
                {
                    distacne = atof(value.c_str());
                }
                tmpsource = tmpsource.substr(string("distance").size() + value.size() + 1);
                setDistanceMap(radius, distacne);
            }
        }
        private_nh.param("dbscan", dbscan_param, std::string(""));
        value = findkeyvalue(dbscan_param, "minnum");
        if (value != "")
        {
            m_min_num = atoi(value.c_str());
        }
        value = findkeyvalue(dbscan_param, "aroundnum");
        if (value != "")
        {
            m_around_num = atoi(value.c_str());
        }
        value = findkeyvalue(dbscan_param, "radius");
        if (value != "")
        {
            m_radius = atof(value.c_str());
        }
        if (DEBUG_PRINT)
        {
            ROS_INFO("########################CLUSTER_NODE#########################\n");
            ROS_INFO("cluster:sub_ground:%s\n", sub_ground.c_str());
            ROS_INFO("cluster::sub_no_ground:%s\n", sub_no_ground.c_str());
            ROS_INFO("cluster::pub_obst_set:%s\n", pub_obst_set.c_str());
            ROS_INFO("cluster::clusetr_model:%d\n", cluster_model);
            ROS_INFO("cluster::eucliden_param:%s\n", eucliden_param.c_str());
            ROS_INFO("cluster::dbscan_param:%s\n", dbscan_param.c_str());
            DEBUG_PRINT = false;
        }
    };
    //从字符串中提取关键字值
    string Param::findkeyvalue(string source, string key)
    {
        string value = "";
        string::size_type loc1 = source.find(key);
        if (loc1 != string::npos)
        {
            string::size_type pos = loc1 + key.size() + 1;
            for (int i = pos; i < source.size(); i++)
            {
                if ((source.at(i) >= '0' && source.at(i) <= '9') || (source.at(i) == '.') || (source.at(i) == '-'))
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
} // namespace skywell
