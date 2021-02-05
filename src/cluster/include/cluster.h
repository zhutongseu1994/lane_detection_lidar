#ifndef _CLUSTER_H_
#define _CLUSTER_H_

#include<vector>
#include<string>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include<map>

namespace skywell{
	class ClusterSave
	{
		public:
			ClusterSave(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,boost::shared_ptr <std::vector<int>> indexPtr):m_avDensity(0), m_source(pc), m_index(indexPtr){}
		public:
			size_t size() const {
				return m_index->size();
			}
			pcl::PointXYZI& operator [] (int i) {
				return (*m_source)[(*m_index)[i]];
			}
			const std::vector<int>& inliner() {
				return *m_index;
			}
		private:
			ClusterSave(ClusterSave&) {}
			ClusterSave& operator =(const ClusterSave&) {}
			ClusterSave& operator =(const ClusterSave&&) {}
		public:
			double m_avDensity;//平均密度
		private:
			boost::shared_ptr<std::vector<int>> m_index;	//点云索引
			pcl::PointCloud<pcl::PointXYZI>::Ptr m_source;	//点云
	};


	typedef std::vector<boost::shared_ptr<ClusterSave>> ClusterVector;
	typedef boost::shared_ptr<std::vector<boost::shared_ptr<ClusterSave>>> ClusterVectorPtr;
	//DBSCAN聚类
	class DBSCANClustering {
		public:
			DBSCANClustering();
		public:
			void setSize(int n,int m,double r);
			void isAdaptive(bool flag);
			void cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr source, ClusterVectorPtr midsave);
		private:
			void optimizationNum(const pcl::PointXYZI& p, double &r, int &min,int &max);
		private:
			double adapt;
			int num;
			int maxsize;
			double radius;
	};


	// 欧几里德聚类
	class EuclideanClustering
	{
		public:
			EuclideanClustering();
			void setClusterSize(int minsize,int maxsize){
			m_min_size = minsize;
			m_max_size = maxsize;
		};
		public:
			void setDistanceMap(std::map<int,double> distancemap);
			void setDistanceMap(int distance,double value);
		public:
			void cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr source, ClusterVectorPtr midsave);
			void cluster_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr source, double in_max_cluster_distance,ClusterVectorPtr midsave);
		private:
			std::map<int ,double> m_distance_map;
			int m_min_size;
			int m_max_size;
	};

	class RegionGrowCluster
	{
		public:
			RegionGrowCluster();
			~RegionGrowCluster();
		public:
			void setParam(int min_s,int max_s,int kn,float smoothness,float curvature);
		public:
			pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr source, ClusterVectorPtr midsave);
		private:
			int m_rg_min_s;
			int m_rg_max_s;
			int m_rg_kn;
			float m_rg_smoothness;// 平滑度
			float m_rg_curvature;// 曲率
	};








	


}



#endif
