//数据结构
#ifndef _TYPE_H_
#define _TYPE_H_


#include <Eigen/Dense>
#include <Eigen/Eigenvalues>




#define TIME_FRAME 0.1
namespace skywell {
	class Kelman {
	public:
		void init(double x, double y) {
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
		Eigen::Vector4d status;//状态
		Eigen::Matrix4d stateCovariance;//状态协方差
		Eigen::Matrix4d stateTransfer;//状态转移
		Eigen::Matrix4d stateTransCovariance;//状态转移协方差
		Eigen::Matrix<double, 2, 4> mapping;//真实值到观测值的映射
		Eigen::Matrix2d noise;//观测噪声的方差
	};

	class Object {
	public:
		enum Type {
			car, people, tree, shrub, other
		};
		//物体类型
		Type type;


		//编号，由匹配规定，第一帧按顺序排
		int number;
		//物体位置
		Eigen::Vector3f transform;
		//物体四元数
		Eigen::Quaternionf rotate;
		//物体长宽高
		double width;//长
		double height;//宽
		double depth;//高


		//物体速度
		double speedx;
		double speedy;

		//kelman
		Kelman kel;

	public:
		Object() {
			number = -1;
			type = Type::other;
			speedx = 0;
			speedy = 0;
		};
	};

}

#endif
