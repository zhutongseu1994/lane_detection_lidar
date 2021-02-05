#ifndef _FILTER_H_
#define _FILTER_H_


#include<string>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/conditional_removal.h>
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/crop_box.h>
#include<pcl/filters/extract_indices.h>
#define LIMIT_ZERO 0.000001

namespace skywell{

// 条件滤波
	class ConditionFilter
	{
		public:
			ConditionFilter():m_negative(true){};
		public:
			// 设置滤波参数
			void setRange(std::string type,double min,double max);
			// 设置保持点云结构
			void setKeepOrganized(bool negative = true);
			// 过滤
			void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr source, pcl::PointCloud<pcl::PointXYZI>::Ptr destination);
			void init();
		private:
			pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond;
			pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
			double x_min, y_min, z_min;
			double x_max, y_max, z_max;
			bool m_negative;
	};

	//周围滤波
	class RadiusFilter
	{
		public:
			// 设置滤波半径
			void setRadius(double radius);
			// 设置领近点集数小于N的删除条件
			void setMinNeighbor(int min);
		public:
			void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr source, pcl::PointCloud<pcl::PointXYZI>::Ptr destination);
		private:
			double m_radius;
			int m_min;
	};


	//离群滤波
	class StatisticsFilter
	{
		public:
			StatisticsFilter():m_negative(false){};
		public:
			void setMeanK(int k);
			void setStddevMulThresh(double thresh);
			void setNegative(bool negative = false);
		public:
			void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr source, pcl::PointCloud<pcl::PointXYZI>::Ptr destination);
		private:
			int m_k;
			double m_thresh;
			bool m_negative;
	};

	// 扇形滤波
	class SectorFilter
	{
		public:
			bool setAngle(double angle);
			void setXMin(double min);
			void setZMax(double max);
			void setZMin(double min);
			void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr source, pcl::PointCloud<pcl::PointXYZI>::Ptr destination);
		private:
			bool conform(const double &x,const double &y);
		private:
			double coefficient;// 系数
			double x_min;
			double z_max,z_min;
	};


	//体素下采样滤波
	class VoxelFilter
	{
		public:
			void setVox(double x, double y, double z);
			void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr source, pcl::PointCloud<pcl::PointXYZI>::Ptr destination);
		private:
		pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	};

	//过滤指定立方体内的点
	class CropBoxFilter
	{
		public:
			CropBoxFilter():m_negative(false){};
		public:
			void setMinP(double xmin,double ymin,double zmin);
			void setMaxP(double xmax,double ymax,double zmax);
			void setNegative(bool negative = false);
			void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr source, pcl::PointCloud<pcl::PointXYZI>::Ptr destination);
			void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr source, std::vector<int> &indice);
		private:
			pcl::CropBox<pcl::PointXYZI> m_clipper;
			Eigen::Vector4f m_minPoint;
			Eigen::Vector4f m_maxPoint;
			bool m_negative;
	};

	// 直通滤波


	// 提取点云子集
	class ExtractIndices
	{
		public:
			ExtractIndices():m_negative(false){}; //默认false提取索引点云
			void setNegative(bool negative = false);
			void filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr source,boost::shared_ptr <std::vector<int>> indexPtr,pcl::PointCloud<pcl::PointXYZI>::Ptr destination);
		private:
			pcl::ExtractIndices<pcl::PointXYZI> extract;
			bool m_negative;
	};



}







#endif


