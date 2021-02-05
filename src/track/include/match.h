#ifndef _MATCH_H_
#define _MATCH_H_
#include"type.h"
#include <list>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <sys/time.h>
#include <flann/flann.h>
#define DELTA_TIME 0.1

typedef std::pair<int,std::vector<float> > vfh_model;
namespace skywell
{
//存储物体列表
class Match
{
public:
	//汽车自身移动,对物体进行修正
	void transformCorrection(Eigen::Vector3f transfer);
	//汽车自身旋转,没z轴
	void rotateSelf(Eigen::Matrix2f rotate);
	void rotateSelf(double angle);
	//匹配
	double distance(Eigen::Vector3f &v);
	void match(boost::shared_ptr<std::vector<skywell::Object>> cubes, boost::shared_ptr<std::vector<skywell::Object>> target);
	void vfh_match(boost::shared_ptr<std::vector<skywell::Object>> cubes,boost::shared_ptr<std::vector<skywell::Object>> target);
	boost::shared_ptr<std::vector<skywell::Object>> getlast();

private:
	bool isvaild(pcl::PointXYZ &point);
	double distance(Eigen::Vector3f &first, Eigen::Vector3f &second);
	//根据位置计算概率
	double positionCheck(Object &first, Object &second);
	//根据旋转角计算概率
	double rotationCheck(Object &first, Object &second);
	//根据体积计算概率
	double volumeCheck(Object &first, Object &second);
	double areaCheck(Object &first, Object &second);
	//实际与预测误差
	double forecastPosCheck(skywell::Object first, skywell::Object second);

private:
	double calculatepredictedvalue(skywell::Object first, skywell::Object second);
	void nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

private:
	boost::shared_ptr<std::vector<skywell::Object>> lastFrameObject;
};

} // namespace skywell

#endif
