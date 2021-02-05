#include "match.h"
#include<boost/make_shared.hpp>
#include<pcl/kdtree/kdtree_flann.h>


namespace skywell {

	static double max(const double& first, const double& second) {
		if (first > second)
			return first;
		else
			second;
	}

	void Match::transformCorrection(Eigen::Vector3f transfer) {
		if (lastFrameObject) {
			for (auto& val : *lastFrameObject) {
				val.transform -= transfer;
			}
		}
	}

	void Match::rotateSelf(Eigen::Matrix2f rotate) {
		if (lastFrameObject) {
			for (auto &val : *lastFrameObject) {

			}
		}
	}

	void Match::rotateSelf(double angle) {


	}

	double Match::distance(Eigen::Vector3f& first, Eigen::Vector3f& second) {
		float x = first(0) - second(0);
		float y = first(1) - second(1);
		float z = first(2) - second(2);
		return sqrt(x * x + y * y + z * z);
	}


	static int iii = 0;
	//计算两个物体匹配概率
	double Match::calculatepredictedvalue(skywell::Object first, skywell::Object second) {
		double pos = positionCheck(first, second);
		if (iii == 1)
			return pos;
		double rot = rotationCheck(first, second);
		if (iii == 2)
			return rot;
		double vol = volumeCheck(first, second);
		if (iii == 3)
			return vol;
		return (pos + rot + vol) / 3.0;
	}

	void directCalculation(Object& first, Object& second) {
		second.speedx = (second.transform.x() - first.transform.x()) / TIME_FRAME;
		second.speedy = (second.transform.y() - first.transform.y()) / TIME_FRAME;
	}

	void Match::match(boost::shared_ptr<std::vector<skywell::Object>> cubes) {
		//汽车移动,每帧向前运动10厘米
		//transformSelf({ -0.1f,0,0 });

		static int counter = 1;

		//第一帧时,记录位置
		if (!lastFrameObject) {
			for (auto& obj : *cubes) {
				//给每个模型编号
				obj.number = counter++;
				//初始化每个模型
				obj.kel.init(obj.transform.x(), obj.transform.y());
			}
		}
		else {
			const int radius = 5;
			//std::cin >> iii;
			int lastSize = lastFrameObject->size();
			int thisSize = cubes->size();
			std::vector<double> matchTable(lastSize*thisSize);
			boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> allData = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

			//存入数据
			for (int i = 0; i < lastSize; i++) {
				pcl::PointXYZ t;
				t.x = (*lastFrameObject)[i].transform.x();
				t.y = (*lastFrameObject)[i].transform.y();
				t.z = (*lastFrameObject)[i].transform.z();
				allData->push_back(t);
			}
			for (int i = 0; i < thisSize; i++) {
				pcl::PointXYZ t;
				t.x = (*cubes)[i].transform.x();
				t.y = (*cubes)[i].transform.y();
				t.z = (*cubes)[i].transform.z();
				allData->push_back(t);
			}

			//建立kd数范围搜索
			boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> kdtree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
			kdtree->setInputCloud(allData);

			//使用kd树进行范围搜索
			for (size_t i = 0; i < lastFrameObject->size(); i++) {
				std::vector<int> index;//索引
				std::vector<float> value;//值
				kdtree->radiusSearch((*allData)[i], radius, index, value);
				//对于上一帧中每一个模型需要搜索
				for (auto j : index) {
					if (j > lastSize - 1) // this序号一定大于lastSize-1，因为上面两部分合并的时候顺序是先插入last，后插入this。在半径搜索的结果中存在last和this的两个点，只有当index序号大于lastSize-1 才表示搜索到的点是在this中，匹配才有意义
					{
						//计算概率并匹配,存入矩阵表
						double probability = calculatepredictedvalue((*lastFrameObject)[i], (*cubes)[j - lastSize]);
						matchTable[i*thisSize + j - lastSize] = probability;
					}
				}
			}
			//寻找最优概率,处理表
			for (size_t i = 0; i < thisSize; i++) {
				//最优位置与最优值
				int maxi = -1;
				double max = 0.01;
				//寻找最优值
				for (size_t j = 0; j < lastSize; j++) {
					if (matchTable[j*thisSize + i] > max) {
						max = matchTable[j*thisSize + i];
						maxi = j;
					}
				}
				if (maxi > -1) {
					//判断列最优值是否为行最优值
					for (size_t k = 0; k < thisSize; k++) {
						if (matchTable[maxi*thisSize + k] > max) {
							goto label1;
						}
					}
					//继承以前模型
					(*cubes)[i].number = (*lastFrameObject)[maxi].number;

					//更新速度
					//test((*lastFrameObject)[i], (*cubes)[maxi]);
					//卡尔曼滤波
					Eigen::Vector4d status = (*lastFrameObject)[maxi].kel.filter((*cubes)[i].transform.x(), (*cubes)[i].transform.y());
					(*cubes)[i].transform.x() = status(0);
					(*cubes)[i].transform.y() = status(1);
					(*cubes)[i].speedx = status(2);
					(*cubes)[i].speedy = status(3);
					(*cubes)[i].kel = (*lastFrameObject)[maxi].kel;

				}
				else
				{
				label1:
					(*cubes)[i].number = counter++;
					(*cubes)[i].kel.init((*cubes)[i].transform.x(), (*cubes)[i].transform.y());
				}

			}
		}
		lastFrameObject = cubes;
	}

	//获取上一个的模型
	boost::shared_ptr<std::vector<skywell::Object>> Match::getlast() {
		if (lastFrameObject) {
			return lastFrameObject;
		}
		return boost::make_shared<std::vector<skywell::Object>>();
	}

	//位置算概率
	double Match::positionCheck(Object & first, Object & second) {

		double probability = 0;
		static double radius = 5;
		double disan = distance(first.transform, second.transform);

		probability = (radius - disan) / radius;

		return probability;
	}

	//角度算概率
	double Match::rotationCheck(Object & first, Object & second) {

		double probability = 0;
		double theta1 = acos(first.rotate.w()) * 2.0;
		double theta2 = acos(second.rotate.w()) * 2.0;
		//double maxt = max(theta1, theta2);

		probability = 1.0 / (1.0 + abs(theta1 - theta2));

		return probability;
	}

	//体积算概率
	double Match::volumeCheck(Object & first, Object & second) {

		double probability = 0;
		//double d = abs(first.depth - second.depth) / max(first.depth, first.depth);
		//double h = abs(first.height - second.height) / max(first.height, first.height);
		//double w = abs(first.width - second.width) / max(first.width, first.width);
		double d = 1.0 / (1.0 + abs(first.depth - second.depth));
		double h = 1.0 / (1.0 + abs(first.height - second.height));
		double w = 1.0 / (1.0 + abs(first.width - second.width));

		probability = (d + h + w) / 3.0;

		return probability;
	}

}