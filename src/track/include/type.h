//数据结构
#ifndef _TYPE_H_
#define _TYPE_H_

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include "vfhfeature.h"

#define TIME_FRAME 0.1

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;

namespace skywell
{


// 一维卡尔曼滤波，主要是对计算后速度的进一步卡尔曼滤波
typedef  struct{
	double filterValue;  //k-1时刻的滤波值，即是k-1时刻的值
	double kalmanGain;   //   Kalamn增益
	double A;   // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
	double H;   // z(n)=H*x(n)+w(n),w(n)~N(0,R)
	double Q;   //预测过程噪声偏差的方差
	double R;   //测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
	double P;   //估计误差协方差
}  KalmanInfo;

void Init_KalmanInfo(KalmanInfo* info, double Q, double R);
double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement);


// x,y点kalman滤波计算速度
class Kalman
{
public:
	void init(double x, double y)
	{
		//初始速度为零
		status << x, y, 0, 0;
		//初始预测矩阵为
		stateCovariance = Eigen::Matrix4d::Identity();
		stateTransfer << 1, 0, TIME_FRAME, 0, 0, 1, 0, TIME_FRAME, 0, 0, 1, 0, 0, 0, 0, 1;
		stateTransCovariance = 0.001 * Eigen::Matrix4d::Identity();
		mapping << 1, 0, 0, 0, 0, 1, 0, 0;
		noise.fill(0.01);
	}

public:
	Eigen::Vector4d filter(double x, double y)
	{
		Eigen::Vector2d obsever;
		obsever << x, y;
		Eigen::Vector4d _status = stateTransfer * status;
		Eigen::Matrix4d _stateCovariance = stateTransfer * stateCovariance*stateTransfer.transpose() + stateTransCovariance;
		auto kelmanCoefficient = _stateCovariance * mapping.transpose() * (mapping*_stateCovariance*mapping.transpose() + noise).inverse();
		status = _status + kelmanCoefficient * (obsever - mapping * _status);
		stateCovariance = (Eigen::Matrix4d::Identity() - kelmanCoefficient * mapping)*_stateCovariance;
		return status;
	}

private:
	Eigen::Vector4d status;				  //状态
	Eigen::Matrix4d stateCovariance;	  //状态协方差
	Eigen::Matrix4d stateTransfer;		  //状态转移
	Eigen::Matrix4d stateTransCovariance; //状态转移协方差
	Eigen::Matrix<double, 2, 4> mapping;  //真实值到观测值的映射
	Eigen::Matrix2d noise;				  //观测噪声的方差
};


typedef struct 
{
	float speed_arry[10];
	uint64_t timestamp_arry[10];
	int arry_num;
} SpeedArry;


float avgspeed(SpeedArry * speedarry,float speed,uint64_t timestamp);

class Point
{
	public:
	float x;
	float y;
	void setP(float x,float y)
	{
		this->x = x;
		this->y = y;
	}
};






class Object
{

public:
	enum Type
	{
		car,
		people,
		tree,
		shrub,
		bicycle,
		other
	};
	//物体类型
	Type type;
	//原始点云
	CloudT::Ptr cloud;
	//编号，由匹配规定，第一帧按顺序排
	int number;
	int cut_in_out;
	int life;
	//物体位置
	Eigen::Vector3f transform;
	//预测下时刻物体位置
	Eigen::Vector3f forecast_next_transform;
	//物体四元数
	Eigen::Quaternionf rotate;

	//物体长宽高
	float length; //长
	float width;  //宽
	float height; //高

	//物体长宽高
	float length_0;
	float width_0;
	float height_0;

	int pointsize;
	float avgpointsize;
	int confidence; //置信度
	//物体速度
	float speedx;
	float speedy;


	//Absolute velocity
	//relative velocity

	float abs_speedx;
	float abs_speedy;


	SpeedArry speedx_arry;
	SpeedArry speedy_arry;


	float accelerationx; // 加速度
	float accelerationy; //// 加速度

	Point P[4];
	//点云聚类物体时间戳
	uint64_t timestamp;
	time_t datatime;
	unsigned int matchtag;
	unsigned int continued;
	bool real;
	float weights; //权重
	//kalman
	Kalman kal;
	//VFHFeature
	VFHFeature vfh;
public:
	Object()
	{
		number = -1;
		type = Type::other;
		abs_speedx = 0;
		abs_speedy = 0;
		speedx = 0;
		speedy = 0;
		accelerationx = 0;
		accelerationy = 0;
		matchtag = 0;
		continued = 0;
		timestamp = time(NULL);
		real = true;
		cut_in_out=0;
		length = 0;
		width = 0;
		memset(&speedx_arry,0,sizeof(speedx_arry));
		memset(&speedy_arry,0,sizeof(speedy_arry));
		cloud = CloudT::Ptr (new CloudT);
	};
};

float GetCross(Point& p1, Point& p2,Point& p);
bool IsPointInMatrix(Point& p,Object &obj);






} // namespace skywell

#endif
