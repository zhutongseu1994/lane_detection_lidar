#include "manager.h"

namespace skywell {



Manager::Manager(ros::NodeHandle &nh,skywell::Param *param)
{
	m_param = param;
	SubInit(nh,param);
	PubInit(nh,param);
	ModInit(param);
};

Manager::~Manager()
{
};

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
void Manager::SubInit(ros::NodeHandle &nh,skywell::Param *param)
{
	if (param->sub_no_ground != "")
	{
		sub_no_ground = nh.subscribe<sensor_msgs::PointCloud2>(param->sub_no_ground,10,boost::bind(&Manager::doRslidarWork,this,_1,param->sub_no_ground));
	}
	if (param->sub_ground != "")
	{
		//sub_ground = nh.subscribe<sensor_msgs::PointCloud2>(param->sub_ground,10,boost::bind(&Manager::doRslidarWork,this,_1,param->sub_ground));
	}
};
void Manager::PubInit(ros::NodeHandle &nh,skywell::Param *param)
{
	
	if (param->pub_obst_set != "")
	{
		pub_obst_set = nh.advertise<cluster::ObstSet>(param->pub_obst_set, 10);
	}

};
void Manager::ModInit(skywell::Param *param)
{

};

void Manager::doRslidarWork(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr,const std::string topicName)
{
	m_param->loadcfg();
	CloudT::Ptr current_pc_ptr = boost::make_shared<CloudT>();
	pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
	//ROS_INFO("cluster heard %ld",current_pc_ptr->header.stamp);
	ros::Time startTime = ros::Time::now();
	/*CloudT::Ptr filter_pc_ptr = boost::make_shared<CloudT>();
	skywell::RadiusFilter radiusfilter;
	radiusfilter.setRadius(m_param->m_raduis);
	radiusfilter.setMinNeighbor(m_param->m_minneighbor);
	radiusfilter.filter(_block_set->obst_pc_ptr,filter_pc_ptr);

	sensor_msgs::PointCloud2 filter_cloud;
	pcl::toROSMsg(*filter_pc_ptr, filter_cloud);
	filter_cloud.header.frame_id = "livox_frame";
	m_pub_point_2.publish(filter_cloud);
	*/
	struct timeval start;
	gettimeofday(&start, NULL);
	//ROS_INFO("startTime time:%s\n",gettimestamp(start.tv_sec*1000000+start.tv_usec).c_str());
	//ROS_INFO("current time:%s\n",gettimestamp(current_pc_ptr->header.stamp).c_str());
	skywell::ClusterVectorPtr midsave = boost::make_shared<skywell::ClusterVector>();
	if (m_param->cluster_model == EUCLIDEAN_TYPE)
	{
		// 方法一 欧几里德聚类
		skywell::EuclideanClustering	Cluster;
		Cluster.setDistanceMap(m_param->m_distance_map);
		Cluster.setClusterSize(m_param->m_min_size,m_param->m_max_size);
		Cluster.cluster(current_pc_ptr, midsave);
	}else if (m_param->cluster_model == DBSCAN_TYPE)
	{
		// 方法二 DBSCAN聚类 聚类
		skywell::DBSCANClustering Cluster;
		Cluster.setSize(m_param->m_around_num,m_param->m_min_num,m_param->m_radius);
		Cluster.cluster(current_pc_ptr, midsave);
	}else if (m_param->cluster_model == MY_DBSCAN_TYPE)
	{
		// 方法三 DBSCAN聚类 MY欧几里德聚类
		MyEuclideanClustering(current_pc_ptr,midsave,(*m_param->m_distance_map.begin()).second,m_param->m_min_size,m_param->m_max_size);
	}
	cluster::ObstSet obstset;
	ToObstSet(midsave,obstset,current_pc_ptr->header.stamp);
	pub_obst_set.publish(obstset);
	ros::Time endTime = ros::Time::now();
	double elapsedTime = (endTime - startTime).toSec()*1000;

	struct timeval end;
	gettimeofday(&end, NULL);
	//ROS_INFO("endTime time:%s\n",gettimestamp(end.tv_sec*1000000+end.tv_usec).c_str());
	ROS_INFO("cluster_node---------------- use_time = %f mesc\n", elapsedTime);
};

void Manager::ToObstSet(ClusterVectorPtr midsave,cluster::ObstSet &obstset,uint64_t stamp)
{
	ros::Time ros_stamp((stamp/1000000),(stamp%1000000)*1000);
	for (size_t i = 0;i < midsave->size() ;i++ )
	{
		cluster::ObstPointCloud _obstpointcloud;
		for (int index = 0; index < (*(*midsave)[i]).size() ;index++ )
		{
			cluster::PointXYZI pointt;
			pointt.x = ((*(*midsave)[i])[index]).x;
			pointt.y = ((*(*midsave)[i])[index]).y;
			pointt.z = ((*(*midsave)[i])[index]).z;
			pointt.intensity = ((*(*midsave)[i])[index]).intensity;
			_obstpointcloud.points.push_back(pointt);
		}
		_obstpointcloud.header.stamp = ros_stamp;
		obstset.obsts.push_back(_obstpointcloud);
	}
	obstset.header.stamp = ros_stamp;
	//printf("障碍物数目：%ld\n",midsave->size());
};



void Manager::MyEuclideanClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,ClusterVectorPtr midsave,float clusterTolerance, int minSize,int maxSize)
{

    skywell::ClusterPts clusterPoints(cloud->points.size(), clusterTolerance, minSize, maxSize);
    clusterPoints.EuclidCluster(cloud,midsave);
    //return clusters;
}


}
