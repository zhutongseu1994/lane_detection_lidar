#include"cluster.h"
#include<stack>
#include<boost/make_shared.hpp>
#include<pcl/kdtree/kdtree_flann.h>

namespace skywell {
	DBSCANClustering::DBSCANClustering() {
		radius = 0;
		num = 0;
		adapt = true;
	}
	void DBSCANClustering::setSize(int n,int m,double r){
		num = n;
		maxsize = m;
		radius = r;
	}

	void DBSCANClustering::isAdaptive(bool flag) {
		adapt = flag;
	}

	void DBSCANClustering::cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr source, ClusterVectorPtr midsave) {
		//使用k-d tree构造点云
		boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZI>> kdtree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
		//创建索引
		std::vector<bool> pcindex(source->size(), false);
		//使用点云创建kd树
		kdtree->setInputCloud(source);
		//聚类
		for (int i = 0; i < source->size(); i++) 
		{
			//被标记过,则跳过
			if (pcindex[i])
			{
				continue;
			}
			//int min = 0;
			//int max = 0;
			//double r = 0;
			//用于存储每一个聚类的点云
			boost::shared_ptr<std::vector<int>> category = boost::make_shared<std::vector<int>>();
			//栈用于迭代寻找邻居
			std::stack<int> stc;
			//先压入自己
			stc.push(i);
			//从未标记的某一点开始聚类
			while (true)
			{
				//栈空则跳出
				if (stc.empty()) 
				{
					break;
				}
				//定义索引和聚类
				std::vector<int> indices;
				std::vector<float> distance;
				//使用kd树查找半径内的，所有点，时间复杂度O(1)
				int index = stc.top();
				//optimizationNum(source->points[index], r, min,max);
				kdtree->radiusSearch(source->points[index], radius, indices, distance);
				//printf("index :%d ,indices = %d\n",index,indices.size());
				stc.pop();
				//周围点数量大于num
				if (indices.size() > num)
				{
					//将周围未聚类的点加入
					for (int i = 0; i < indices.size(); i++)
					{
						//printf("i = %d  distance = %f\n",i,sqrt(distance[i]));
						
						if (!pcindex[indices[i]]) 
						{				//标记
							pcindex[indices[i]] = true;//标记
							category->push_back(indices[i]);//存入索引
							//对于距离较近的点不在重复计算
							if (indices.size() > num) 
							{
								if (distance[i] > radius * radius * 0.7) 
								{
									stc.push(indices[i]);//压栈
								}
							}
						}
					}
				}
			}
			if (category->size() > maxsize) 
			{
				boost::shared_ptr<ClusterSave> cs = boost::make_shared<ClusterSave>(source, category);
				midsave->push_back(cs);
			}
		}
	}
	void DBSCANClustering::optimizationNum(const pcl::PointXYZI& p, double &r, int &min,int &max) {
		if (adapt) {
			double distance = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);

			double pinterval = (3.1415926*distance)/1800;
			double linterval = 3.1415926*distance/32/2;
			int minsize = 0;
			int maxsize = 0;
			if (pinterval < 0.1)
			{
				minsize = (int)(2*1.414*linterval/0.1);
				maxsize = (int)((2*1.414*linterval + 2*linterval)/0.1);
			}else
			{
				minsize = (int)(2*1.414*linterval/pinterval);
				maxsize = (int)((2*1.414*linterval + 2*linterval)/pinterval);
			}
			r = 1.414*linterval;
			min = minsize;
			max = maxsize;
		}
	}

	EuclideanClustering::EuclideanClustering()
	{
		setDistanceMap(80,0.08);
	};
	
	void EuclideanClustering::setDistanceMap(std::map<int,double> distancemap)
	{
		m_distance_map = distancemap;
		/*while (!m_distance_map.empty())
		{
			m_distance_map.erase(m_distance_map.begin());
		}
		std::map<int,double>::iterator iter;
		for (iter = distancemap.bgein();iter != distancemap.end() ;iter++)
		{
			m_distance_map.insert(std::make_pair(iter->first,iter->second));
		}*/
	};

	void EuclideanClustering::setDistanceMap(int distance,double value)
	{
		std::map<int,double>::iterator iter;
		iter = m_distance_map.find(distance);
		if (iter != m_distance_map.end())
		{
			iter->second = value;
			return;
		}
		m_distance_map.insert(std::make_pair(distance,value));
	};

	void EuclideanClustering::cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr source, ClusterVectorPtr midsave)
	{
		//printf("m_distance_map.size() == %ld\n",m_distance_map.size());
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_pc_array(m_distance_map.size());
		for (size_t i = 0; i < segment_pc_array.size() ; i++)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
			segment_pc_array[i] = tmp;
		}
		for (size_t i = 0; i < source->points.size() ;i++ )
		{
			pcl::PointXYZI current_point;
			current_point.x = source->points[i].x;
			current_point.y = source->points[i].y;
			current_point.z = source->points[i].z;
			current_point.intensity = source->points[i].intensity;
			float origin_distance = sqrt(
			pow(current_point.x, 2) + pow(current_point.y, 2));
			if (origin_distance >= 120) 
			{
				continue;
			}
			std::map<int,double>::iterator iter;
			int n = 0;
			for (iter = m_distance_map.begin(); iter != m_distance_map.end();iter++)
			{
				if (origin_distance <= iter->first )
				{
					segment_pc_array[n]->points.push_back(current_point);
					break;
				}
				n++;
			}
		}
		std::map<int,double>::iterator iter;
		int i = 0;
		for (iter = m_distance_map.begin(); iter != m_distance_map.end();iter++)
		{
			cluster_segment(segment_pc_array[i],iter->second,midsave);
			i++;
		}
	};

	void EuclideanClustering::cluster_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr source,double in_max_cluster_distance,ClusterVectorPtr midsave)
	{
		if (source->points.size() == 0)
		{
			return;
		}
		pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
		//create 2d pc
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::copyPointCloud(*source, *cloud_2d);

		//make it flat
		for (size_t i = 0; i < cloud_2d->points.size(); i++)
		{
			cloud_2d->points[i].z = 0;
		}
		if (cloud_2d->points.size() > 0)
		{
			tree->setInputCloud(cloud_2d);
		}
		std::vector<pcl::PointIndices> local_indices;

		pcl::EuclideanClusterExtraction<pcl::PointXYZI> euclid;
		euclid.setInputCloud(cloud_2d);
		euclid.setClusterTolerance(in_max_cluster_distance); //设置近邻搜索的搜索半径为8cm
		euclid.setMinClusterSize(m_min_size);// 设置一个聚类需要的最少点数目为10
		euclid.setMaxClusterSize(m_max_size); //设置一个聚类需要的最大点数目为10000
		euclid.setSearchMethod(tree);
		euclid.extract(local_indices);
		for (size_t i = 0; i < local_indices.size() ;i++ )
		{
			boost::shared_ptr<std::vector<int>> category = boost::make_shared<std::vector<int>>();
			for (auto pit = local_indices[i].indices.begin();
			 pit != local_indices[i].indices.end(); ++pit)
			{
				category->push_back(*pit);
			}
			boost::shared_ptr<ClusterSave> cs = boost::make_shared<ClusterSave>(source, category);
			midsave->push_back(cs);
		}
	};

RegionGrowCluster::RegionGrowCluster()
{
	m_rg_min_s = 100;
	m_rg_max_s = 10000;
	m_rg_kn = 20;
	m_rg_smoothness = 30.0;// 平滑度
	m_rg_curvature = 0.05;// 曲率
}
RegionGrowCluster::~RegionGrowCluster()
{

}

void RegionGrowCluster::setParam(int min_s,int max_s,int kn,float smoothness,float curvature)
{
	m_rg_min_s = min_s;
	m_rg_max_s = max_s;
	m_rg_kn = kn;
	m_rg_smoothness = smoothness;// 平滑度
	m_rg_curvature = curvature;// 曲率
};

pcl::PointCloud<pcl::PointXYZI>::Ptr RegionGrowCluster::cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr source, ClusterVectorPtr midsave)
{

	bool Bool_Cuting=false;//设置默认输入参数
	float far_cuting=10; //设置默认输入参数
	float near_cuting=0;//设置默认输入参数
	pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI> > (new pcl::search::KdTree<pcl::PointXYZI>);//创建一个指向kd树搜索对象的共享指针
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;//创建法线估计对象
	normal_estimator.setSearchMethod (tree);//设置搜索方法
	normal_estimator.setInputCloud (source);//设置法线估计对象输入点集
	normal_estimator.setKSearch (m_rg_kn);// 设置用于法向量估计的k近邻数目
	normal_estimator.compute (*normals);//计算并输出法向量
	
	pcl::IndicesPtr indices (new std::vector <int>);//创建一组索引
	if(Bool_Cuting)//判断是否需要直通滤波
	{

		pcl::PassThrough<pcl::PointXYZI> pass;//设置直通滤波器对象
		pass.setInputCloud (source);//设置输入点云
		pass.setFilterFieldName ("z");//设置指定过滤的维度
		pass.setFilterLimits (near_cuting, far_cuting);//设置指定纬度过滤的范围
		pass.filter (*indices);//执行滤波，保存滤波结果在上述索引中
	}
	// 区域生长算法的5个参数
	pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;//创建区域生长分割对象
	reg.setMinClusterSize (m_rg_min_s);//设置一个聚类需要的最小点数
	reg.setMaxClusterSize (m_rg_max_s);//设置一个聚类需要的最大点数
	reg.setSearchMethod (tree);//设置搜索方法
	reg.setNumberOfNeighbours (50);//设置搜索的临近点数目
	reg.setInputCloud (source);//设置输入点云
	if(Bool_Cuting)reg.setIndices (indices);//通过输入参数设置，确定是否输入点云索引
	reg.setInputNormals (normals);//设置输入点云的法向量
	reg.setSmoothnessThreshold (m_rg_smoothness / 180.0 * M_PI);//设置平滑阈值
	reg.setCurvatureThreshold (m_rg_curvature);//设置曲率阈值

	std::vector <pcl::PointIndices> clusters;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	reg.extract (clusters);//获取聚类的结果，分割结果保存在点云索引的向量中。
	//std::vector<int> index;
	for (size_t i = 0; i < clusters.size() ;i++ )
	{
		boost::shared_ptr<std::vector<int>> category = boost::make_shared<std::vector<int>>();
		for (auto pit = clusters[i].indices.begin();
		 pit != clusters[i].indices.end(); ++pit)
		{
			category->push_back(*pit);
			inliers->indices.push_back(*pit);
		}
		boost::shared_ptr<ClusterSave> cs = boost::make_shared<ClusterSave>(source, category);
		midsave->push_back(cs);
	}

	// 提取剩余点云
		pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud (new pcl::PointCloud<pcl::PointXYZI>);
	 	pcl::ExtractIndices<pcl::PointXYZI> extract;
	 	// Extract the inliers
	 	extract.setInputCloud(source);
	 	extract.setIndices(inliers);
	 	extract.setNegative(true);
	 	extract.filter(*obstCloud);
		return obstCloud;
};

}






