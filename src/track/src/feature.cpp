#include "feature.h"

#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <boost/make_shared.hpp>
#include <stdio.h>
#include <stdlib.h>

#define PAI 3.141592653

namespace skywell
{

//数字转字符串
static std::string int2str(int number)
{
	static char c[20];
	snprintf(c, sizeof(c), "%d", number);
	return c;
}

//计算两点间距离
static float distance(cv::Point2f &first, cv::Point2f &second)
{
	float dx = first.x - second.x;
	float dy = first.y - second.y;
	return sqrt(dx * dx + dy * dy);
}


void Feature::selffusion(boost::shared_ptr<std::vector<skywell::Object>> source,boost::shared_ptr<std::vector<skywell::Object>> cubes)
{
	boost::shared_ptr<std::vector<skywell::Object>> tempobjects = boost::make_shared<std::vector<skywell::Object>>();
	Object lastobj;
	for (int i = 0;i < source->size() ; i++)
	{
		if (i == 0)
		{
			lastobj =  (*source)[i];
		}else
		{
			Object thisobj;
			thisobj = (*source)[i];
			bool iscontain = false;
			if ((lastobj.length * lastobj.width) > (thisobj.length * thisobj.width))
			{

				Point p;
				p.setP(thisobj.transform.x(),thisobj.transform.y());
				if (IsPointInMatrix(p, lastobj))
				{
					iscontain = true;
				};
			}else
			{
				Point p;
				p.setP(lastobj.transform.x(),lastobj.transform.y());
				if (IsPointInMatrix(p, thisobj))
				{
					iscontain = true;
					lastobj = thisobj;
				};
			}
			if (!iscontain)
			{
				tempobjects->push_back(thisobj);
			}
		}
	}
	cubes->push_back(std::move(lastobj));
	if (tempobjects->size() != 0)
	{
		selffusion(tempobjects,cubes);
	}
};



void Feature::makeCubes(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &Obst, boost::shared_ptr<std::vector<skywell::Object>> cubes)
{
	boost::shared_ptr<std::vector<skywell::Object>> tempobjects = boost::make_shared<std::vector<skywell::Object>>();
	for (size_t i = 0; i < Obst.size(); i++)
	{
		Object Object;
		findCubeWithOpenCV(Obst[i], Object);
		if (Object.height > 0.1)
		{
			tempobjects->push_back(std::move(Object));
		}
		
	}
	if (tempobjects->size() != 0)
	{
		selffusion(tempobjects, cubes);
	}
	//selffusion(tempobjects,cubes);
	return;
};

//求所有物体包围盒
void Feature::makeCubes(ClusterVectorPtr cvp, boost::shared_ptr<std::vector<skywell::Object>> cubes)
{
	for (size_t i = 0; i < cvp->size(); i++)
	{
		Object Object;
		findCubeWithOpenCV(*(*cvp)[i], Object);
		//findCube(*(*cvp)[i], Object);
		cubes->push_back(std::move(Object));
	}
	return;
}

//使用opencv找包围盒
void Feature::findCubeWithOpenCV(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, skywell::Object &object)
{
	object.timestamp = cloud->header.stamp;
	object.pointsize = (int)cloud->points.size();
	std::vector<cv::Point> vec;
	//double avgintensity = 0.0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		cv::Point p;
		p.x = cloud->points[i].x * 10000.0;
		p.y = cloud->points[i].y * 10000.0;
		//avgintensity += cloud->points[i].intensity;
		vec.push_back(std::move(p));
	}
	pcl::copyPointCloud(*cloud,*object.cloud);
	object.vfh.setInputCloud(object.cloud);
	//object.avgintensity = avgintensity / (double)object.pointsize;
	//计算包围盒
	cv::RotatedRect rect = cv::minAreaRect(vec);  // 最小斜矩阵
	cv::Rect rectb = cv::boundingRect(vec);// 正矩阵
	cv::Point2f P[4];
	rect.points(P);
	//printf("angle = %f\n",rect.angle);
	
	for (size_t i = 0; i < 4; i++)
	{
		P[i].x /= 10000.0;
		P[i].y /= 10000.0;
		object.P[i].x = P[i].x;
		object.P[i].y = P[i].y;
		//printf("P[%d].x : %f, P[%d].y : %f\n",i,P[i].x,i,P[i].y);
	}
	object.width = rectb.height/ 10000.0;
	object.length = rectb.width/ 10000.0;
	//旋转角为最长边旋转角
	double zmin, zmax;
	zmin = zmax = cloud->points[0].z;
	for (size_t i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].z < zmin)
			zmin = cloud->points[i].z;
		else if (cloud->points[i].z > zmax)
			zmax = cloud->points[i].z;
	}

	//偏移
	object.transform(0) = (rectb.x + cvRound(rectb.width/2.0))/10000.0;
	object.transform(1) = (rectb.y + cvRound(rectb.height/2.0))/10000.0;

	//object.transform(0) = rect.center.x / 10000.0;
	//object.transform(1) = rect.center.y / 10000.0;
	object.transform(2) = (zmin + zmax) / 2;

	//旋转角，弧度制
	double theta = 0;
	double slope = 0;
	double _theta = 0;

	//计算最长边，以最长边为主要方向
	//float one = distance(P[0], P[1]);
	//float another = distance(P[1], P[2]);
	//printf("rect:width: %f   heigth:%f\n",rect.size.width,rect.size.height);
	//printf("rect0:width: %f   heigth:%f\n",rect_0.width,rect_0.height);
	
	if (rect.size.height > rect.size.width)
	{
		_theta = ((rect.angle + 90 )/180)*PAI;
		//object.length = rect.size.height / 10000.0;
		//object.width = rect.size.width / 10000.0;
	}else
	{
		_theta = ((rect.angle)/180)*PAI;
		//object.length = rect.size.width / 10000.0;
		//object.width = rect.size.height / 10000.0;
	}
	//printf("object.transform(0):%f,object.transform(1):%f\n",object.transform(0),object.transform(1));
	//printf("rectb.x:%f,rectb.y:%f,rectb.height:%f,rectb.width:%f -- rect.center.x:%f,rect.center.y:%f,P[0].x:%f,P[0].y:%f,rect.height:%f,rect.width:%f\n",rectb.x/10000.0,rectb.y/10000.0,rectb.height/10000.0,rectb.width/10000.0,rect.center.x/10000.0,rect.center.y/10000.0,P[0].x,P[0].y,rect.size.height/10000.0,rect.size.width/10000.0);


	//printf("one = %f   another = %f\n",one,another);

	//printf("_theta = %f\n",_theta);


	/*if (one > another)
	{
		printf("===============================\n");
		//斜率为正负无穷
		if ((P[0].x - P[1].x) > -0.01 and (P[0].x - P[1].x) < 0.01)
		{
			theta = PAI / 2;
		}
		else
		{
			slope = (P[0].y - P[1].y) / (P[0].x - P[1].x);
			theta = atan(slope);
			printf("theta = %f\n",theta);
		}
		//object.length = one;
		//object.width = another;
	}
	else
	{
		printf("------------------------------\n");
		if ((P[1].x - P[2].x) > -1 and (P[1].x - P[2].x) < 1)
		{
			theta = PAI / 2;
		}
		else
		{
			slope = (P[1].y - P[2].y) / (P[1].x - P[2].x);
			theta = atan(slope);
			printf("theta = %f\n",theta);
		}
		//object.length = another;
		//object.width = one;
	}*/
	//object.height_0 =  zmax - zmin;
	object.height = zmax - zmin;
	object.avgpointsize = (float)object.pointsize/(object.height *(object.width + object.length));


	object.rotate.w() = cos(_theta / 2);
	object.rotate.x() = 0;
	object.rotate.y() = 0;
	object.rotate.z() = sin(_theta / 2);
}
void Feature::findCubeWithOpenCV(ClusterSave &csave, skywell::Object &object)
{
	std::vector<cv::Point> vec;
	for (int i = 0; i < csave.size(); i++)
	{
		cv::Point p;
		p.x = csave[i].x * 10000.0;
		p.y = csave[i].y * 10000.0;
		vec.push_back(std::move(p));
	}
	//计算包围盒
	cv::RotatedRect rect = cv::minAreaRect(vec);
	cv::Point2f P[4];
	rect.points(P);
	for (size_t i = 0; i < 4; i++)
	{
		P[i].x /= 10000.0;
		P[i].y /= 10000.0;
	}
	//旋转角为最长边旋转角
	double zmin, zmax;
	zmin = zmax = csave[0].z;
	for (size_t i = 0; i < csave.size(); i++)
	{
		if (csave[i].z < zmin)
			zmin = csave[i].z;
		else if (csave[i].z > zmax)
			zmax = csave[i].z;
	}

	//偏移
	object.transform(0) = rect.center.x / 10000.0;
	object.transform(1) = rect.center.y / 10000.0;
	object.transform(2) = (zmin + zmax) / 2;

	//旋转角，弧度制
	double theta = 0;

	//计算最长边，以最长边为主要方向
	float one = distance(P[0], P[1]);
	float another = distance(P[1], P[2]);
	if (one > another)
	{
		//斜率为正负无穷
		if ((P[0].x - P[1].x) > -0.01 and (P[0].x - P[1].x) < 0.01)
		{
			theta = PAI / 2;
		}
		else
		{
			double slope = (P[0].y - P[1].y) / (P[0].x - P[1].x);
			theta = atan(slope);
		}
		object.length = one;
		object.width = another;
	}
	else
	{
		if ((P[1].x - P[2].x) > -1 and (P[1].x - P[2].x) < 1)
		{
			theta = PAI / 2;
		}
		else
		{
			double slope = (P[1].y - P[2].y) / (P[1].x - P[2].x);
			theta = atan(slope);
		}
		object.length = another;
		object.width = one;
	}
	object.height = zmax - zmin;

	object.rotate.w() = cos(theta / 2);
	object.rotate.x() = 0;
	object.rotate.y() = 0;
	object.rotate.z() = sin(theta / 2);
}

//根据RANSAC找
/*double Feature::findSlopeWithRANSAC(ClusterSave& csave)
	{
		//分桶找最多点的那条线
		std::vector<int> bucket[32];
		for (size_t i = 0; i < csave.size(); i++) {
			bucket[csave[i]].push_back(i);  //疑问
		}
		int max(0);
		int first(0), second(0);
		for (size_t i = 0; i < 32; i++) {
			if (bucket[i].size() > max) {
				second = first;
				first = i;
				max = bucket[i].size();
			}
		}
		//存储其中的数据
		std::vector<int> number;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		for (size_t i = 0; i < bucket[first].size(); i++) {
			cloud->push_back(csave[bucket[first][i]]);
			number.push_back(bucket[first][i]);
			//csave[bucket[first][i]].rgb = 100;
		}
		//RANSAC拟合直线
		std::vector<int> inliers;
		pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l = boost::make_shared<pcl::SampleConsensusModelLine<pcl::PointXYZ>>(cloud);
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
		ransac.setDistanceThreshold(.01);
		ransac.computeModel();
		ransac.setMaxIterations(100);
		ransac.getInliers(inliers);
		Eigen::VectorXf vec;
		ransac.getModelCoefficients(vec);

		for (size_t i = 0; i < inliers.size(); i++)
		{
			csave[number[inliers[i]]].rgb = 100;
		}

		pcl::PointXYZ& p1 = (*cloud)[inliers[0]];
		pcl::PointXYZ& p2 = (*cloud)[inliers[inliers.size() - 1]];
		double slope = (p2.y - p1.y) / (p2.x - p1.x);

		return slope;
	}*/

//根据PCA找
double Feature::findSlopeWithPCA(ClusterSave &csave)
{
	static int counter = 1;
	//计算x,y分量上的期望
	double meanx = 0, meany = 0;
	for (size_t i = 0; i < csave.size(); i++)
	{
		meanx += csave[i].x;
		meany += csave[i].y;
	}
	meanx = meanx / csave.size();
	meany = meany / csave.size();
	//计算x,y分量的协方差矩阵
	Eigen::Matrix<double, 2, 2> covariances; //协方差矩阵
	covariances << 0, 0, 0, 0;
	for (size_t i = 0; i < csave.size(); i++)
	{
		covariances(0, 0) += (csave[i].x - meanx) * (csave[i].x - meanx);
		covariances(0, 1) += (csave[i].x - meanx) * (csave[i].y - meany);
		covariances(1, 1) += (csave[i].y - meany) * (csave[i].y - meany);
	}
	covariances(0, 0) /= csave.size();
	covariances(0, 1) /= csave.size();
	covariances(1, 1) /= csave.size();
	covariances(1, 0) = covariances(0, 1);
	//求特征值和特征向量
	Eigen::Matrix<double, 2, 2> eigenvalues;  //特征值
	Eigen::Matrix<double, 2, 2> eigenvectors; //特征向量
	Eigen::EigenSolver<Eigen::Matrix2d> es(covariances);
	eigenvalues = es.pseudoEigenvalueMatrix();
	eigenvectors = es.pseudoEigenvectors();
	//计算斜率，两条线垂直，计算一条就够
	double slope = eigenvectors(0, 0) / eigenvectors(1, 0);

	return slope;
}

//剔除一些不必要的
void Feature::classify(skywell::Object &Object)
{
	double v = Object.height * Object.width * Object.width;
	if (v > 3 && v < 6)
	{
	}
}

//求包围盒
void Feature::findCube(ClusterSave &csave, skywell::Object &Object)
{

	double slope;

	slope = findSlopeWithPCA(csave);
	//slope = findSlopeWithRANSAC(csave);

	//x,y,z三个分量上的最大值和最小值
	double tx, ty;
	tx = csave[0].x;
	ty = csave[0].y;
	rotate(tx, ty, slope);
	double xmax, xmin, ymax, ymin, zmax, zmin;
	xmax = tx;
	xmin = tx;
	ymax = ty;
	ymin = ty;
	zmax = csave[0].z;
	zmin = csave[0].z;
	for (size_t i = 0; i < csave.size(); i++)
	{
		tx = csave[i].x;
		ty = csave[i].y;
		rotate(tx, ty, slope);
		if (tx > xmax)
		{
			xmax = tx;
		}
		if (tx < xmin)
		{
			xmin = tx;
		}
		if (ty > ymax)
		{
			ymax = ty;
		}
		if (ty < ymin)
		{
			ymin = ty;
		}
		if (csave[i].z > zmax)
		{
			zmax = csave[i].z;
		}
		if (csave[i].z < zmin)
		{
			zmin = csave[i].z;
		}
	}
	//长宽高
	Object.length = xmax - xmin;
	//Object.height = ymax - ymin;
	//Object.width = zmax - zmin;
	Object.width = ymax - ymin;
	Object.height = zmax - zmin;

	//中心位置
	tx = (xmax + xmin) / 2.0;
	ty = (ymax + ymin) / 2.0;
	rerotate(tx, ty, slope);
	Object.transform(0, 0) = tx;
	Object.transform(1, 0) = ty;
	Object.transform(2, 0) = (zmax + zmin) / 2;
	//旋转量
	Object.rotate.w() = cos(atan(-1.0 / slope) / 2.0);
	Object.rotate.x() = 0;
	Object.rotate.y() = 0;
	Object.rotate.z() = tan(atan(-1.0 / slope) / 2.0);
}

//旋转
void Feature::rotate(double &x, double &y, const double &slope)
{
	//计算角度
	double theta = atan(slope);
	//旋转
	double t = x;
	x = x * cos(theta) - y * sin(theta);
	y = t * sin(theta) + y * cos(theta);
}

//反向旋转
void Feature::rerotate(double &x, double &y, const double &slope)
{
	rotate(x, y, -slope);
}

} // namespace skywell




