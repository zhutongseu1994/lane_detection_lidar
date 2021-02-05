//计算包围盒
#ifndef _FEATURE_H_
#define _FEATURE_H_
#include"cluster.h"
#include"type.h"

namespace skywell {
	class Feature {
	public:
		void makeCubes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &Obst, boost::shared_ptr<std::vector<skywell::Object>> cubes);
		void makeCubes(ClusterVectorPtr cvp, boost::shared_ptr<std::vector<skywell::Object>> cubes);
		void selffusion(boost::shared_ptr<std::vector<skywell::Object>> source,boost::shared_ptr<std::vector<skywell::Object>> cubes);
	private:
		void findCubeWithOpenCV( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, skywell::Object & object);
		void findCubeWithOpenCV(ClusterSave& csave, skywell::Object& Object);
	private://nouse
		void classify(skywell::Object& Object);
		void findCube(ClusterSave& csave, skywell::Object& Object);
		//以z轴为轴旋转slope斜率
		void rotate(double& x, double& y, const double& slope);
		void rerotate(double& x, double& y, const double& slope);
		//double findSlopeWithRANSAC(ClusterSave& csave);
		double findSlopeWithPCA(ClusterSave& csave);
	};
}

#endif
