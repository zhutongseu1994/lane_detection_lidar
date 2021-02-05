#include"filter.h"


namespace skywell {

	const static double PAI = 3.14159265358979;
	void ConditionFilter::setRange(std::string type,double min,double max)
	{
		if ("x" == type)
		{
			x_min = min;
			x_max = max;
		}else if ("y" == type)
		{
			x_min = min;
			x_max = max;
		}else if ("z" == type)
		{
			x_min = min;
			x_max = max;
		}
		return;
	};

	void ConditionFilter::setKeepOrganized(bool negative)
	{
		m_negative = negative;
	};

	void ConditionFilter::filter(const CloudT::Ptr source, CloudT::Ptr destination)
	{
		if (!range_cond)
		{
			init();
		}
		condrem.setInputCloud(source);
		condrem.setCondition(range_cond);
		condrem.setKeepOrganized(m_negative);
		condrem.filter(*destination);
	};


	void ConditionFilter::init()
	{
		range_cond = boost::make_shared<pcl::ConditionAnd<PointT>>();
		range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, x_min)));
		range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, x_max)));

		range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, y_min)));
		range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, y_max)));

		range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, z_min)));
		range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, z_max)));
	}


	void RadiusFilter::setRadius(double radius)
	{
		m_radius = radius;
	};

	void RadiusFilter::setMinNeighbor(int min)
	{
		m_min = min;
	};

	void RadiusFilter::filter(const CloudT::Ptr source, CloudT::Ptr destination)
	{
		pcl::RadiusOutlierRemoval<PointT> outrem;
		// 创建滤波器
		outrem.setInputCloud(source);
		outrem.setRadiusSearch(m_radius);
		outrem.setMinNeighborsInRadius (m_min);
		// 应用滤波器
		outrem.filter (*destination);
	};



	void StatisticsFilter::setMeanK(int k)
	{
		m_k = k;
	};
	void StatisticsFilter::setStddevMulThresh(double thresh)
	{
		m_thresh = thresh;
	};
	void StatisticsFilter::setNegative(bool negative)
	{
		m_negative = negative;
	};


	void StatisticsFilter::filter(const CloudT::Ptr source, CloudT::Ptr destination)
	{
		pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud (source);
		sor.setMeanK (m_k);
		sor.setStddevMulThresh (m_thresh);
		sor.setNegative (m_negative);
		sor.filter (*destination);
	};


	bool SectorFilter::setAngle(double angle){
		if (angle <= 0 || angle >= 90)
		{
			std::cout << "角度范围错误，应为0~90" << std::endl;
			return false;
		}
		coefficient = tan(angle *PAI /180.0);
		return true;
	}
	void SectorFilter::setXMin(double min) {
			this->x_min = min;
	}

	void SectorFilter::setZMax(double max) {
			this->z_max = max;
	}

	void SectorFilter::setZMin(double min) {
			this->z_min = min;
	}
	bool SectorFilter::conform(const double& x, const double& y) {
		if (x > 0 || x < x_min) {
			return false;
		}
		if (y > 0) {
			if (abs(x) > coefficient * y) {
				return true;
			}
		}
		if (y < 0) {
			if (abs(x) > -coefficient * y) {
				return true;
			}
		}
		return false;
	}

	void SectorFilter::filter(const CloudT::Ptr source, CloudT::Ptr destination) {
			for (const auto & p : *source) {
			if (p.z > z_max || p.z < z_min || (p.z > -LIMIT_ZERO && p.z < -LIMIT_ZERO)) {
				continue;
			}
			if (conform(p.x - 3, p.y)) {
				destination->push_back(p);
			}
		}
	}

	void VoxelFilter::setVox(double x, double y, double z) {
		sor.setLeafSize(x, y, z);
	}


	void VoxelFilter::filter(const CloudT::Ptr source, CloudT::Ptr destination)
	{
		pcl::PCLPointCloud2::Ptr pc2 = boost::make_shared<pcl::PCLPointCloud2>();
		pcl::PCLPointCloud2::Ptr pc3 = boost::make_shared<pcl::PCLPointCloud2>();
		pcl::toPCLPointCloud2(*source, *pc2);
		sor.setInputCloud(pc2);
		sor.filter(*pc3);
		pcl::fromPCLPointCloud2(*pc3, *destination);
	};



	void CropBoxFilter::setMinP(double xmin,double ymin,double zmin)
	{
		m_minPoint = Eigen::Vector4f(xmin, ymin, zmin, 1);
	};
	void CropBoxFilter::setMaxP(double xmax,double ymax,double zmax)
	{
		m_maxPoint = Eigen::Vector4f(xmax, ymax, zmax, 1);
	};
	void CropBoxFilter::setNegative(bool negative)
	{
		m_negative = negative;
	};
	void CropBoxFilter::filter(const CloudT::Ptr source, CloudT::Ptr destination)
	{
		m_clipper.setInputCloud(source); 
		m_clipper.setMin(m_minPoint);
		m_clipper.setMax(m_maxPoint);
		m_clipper.setNegative(m_negative);
		m_clipper.filter(*destination); 
	};

	void CropBoxFilter::filter(const CloudT::Ptr source, std::vector<int> &indice)
	{
		m_clipper.setInputCloud(source); 
		m_clipper.setMin(m_minPoint);
		m_clipper.setMax(m_maxPoint);
		m_clipper.setNegative(m_negative);
		m_clipper.filter(indice); 
	};

	void ExtractIndices::setNegative(bool negative)
	{
		m_negative = negative;
	};
	void ExtractIndices::filter(const CloudT::Ptr source,boost::shared_ptr <std::vector<int>> indexPtr,CloudT::Ptr destination)
	{
		extract.setInputCloud (source);
		extract.setIndices (indexPtr);
		extract.setNegative (m_negative);
		extract.filter (*destination);
	};
}
