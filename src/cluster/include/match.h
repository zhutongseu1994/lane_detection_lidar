#ifndef _MATCH_H_
#define _MATCH_H_
#include"type.hpp"
#include<list>
#include<vector>
#include<pcl/point_types.h>  

#define DELTA_TIME 0.1

namespace skywell {
	//存储物体列表
	class Match {
	public:
		//汽车自身移动,对物体进行修正
		void transformCorrection(Eigen::Vector3f transfer);
		//汽车自身旋转,没z轴
		void rotateSelf(Eigen::Matrix2f rotate);
		void rotateSelf(double angle);
		//匹配
		void match(boost::shared_ptr<std::vector<skywell::Object>> cubes);
		boost::shared_ptr<std::vector<skywell::Object>> getlast();
	private:
		double distance(Eigen::Vector3f& first, Eigen::Vector3f& second);
		//根据位置计算概率
		double positionCheck(Object& first, Object& second);
		//根据旋转角计算概率
		double rotationCheck(Object& first, Object& second);
		//根据体积计算概率
		double volumeCheck(Object& first, Object& second);
	private:
		double calculatepredictedvalue(skywell::Object first, skywell::Object second);
	private:
		boost::shared_ptr<std::vector<skywell::Object>> lastFrameObject;
	};
}

#endif