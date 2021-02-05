	#ifndef _VFH_FEATURE_H_
	#define _VFH_FEATURE_H_

	#include<pcl/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/features/vfh.h>                     //VFH特征估计类头文件
	#include <pcl/features/normal_3d.h>
	#include <pcl/features/integral_image_normal.h>
	typedef pcl::PointXYZI PointT;
	typedef pcl::PointCloud<PointT> CloudT;

	using namespace std;

	class VFHFeature
	{
		public:
			VFHFeature()
			{
				cloud = CloudT::Ptr (new CloudT);
				vfhs = pcl::PointCloud<pcl::VFHSignature308>::Ptr (new pcl::PointCloud<pcl::VFHSignature308> ());
			};
		public:
			void setInputCloud(CloudT::Ptr source_cloud)
			{
				cloud = source_cloud;
				processInput();
			};
			pcl::PointCloud<pcl::VFHSignature308>::Ptr getVFHs()const
			{
				return (vfhs);
			}
		private:
			void processInput(void)
			{
				// 估计法线
				pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
				pcl::NormalEstimation<PointT, pcl::Normal> ne;
				ne.setInputCloud (cloud);
				pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
				ne.setSearchMethod (tree);
				ne.setRadiusSearch (0.5);
				//计算特征值
				ne.compute (*normals);
				//创建VFH估计对象vfh，并把输入数据集cloud和法线normal传递给它
				pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
				vfh.setInputCloud (cloud);
				vfh.setInputNormals (normals);
				//如果点云是PointNormal类型，则执行vfh.setInputNormals (cloud);
				//基于已知的输入数据集，建立kdtree
				vfh.setSearchMethod (tree);
				//计算特征值
				vfh.compute (*vfhs);
			};
		private:
			CloudT::Ptr cloud;
			pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs;
	};






	#endif
