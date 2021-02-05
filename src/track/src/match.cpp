#include "match.h"
#include <boost/make_shared.hpp>
#include <pcl/kdtree/kdtree_flann.h>

namespace skywell
{

static double max(const double &first, const double &second)
{
	if (first > second)
		return first;
	else
		second;
}

void Match::transformCorrection(Eigen::Vector3f transfer)
{
	if (lastFrameObject)
	{
		for (auto &val : *lastFrameObject)
		{
			val.transform -= transfer;
		}
	}
}

void Match::rotateSelf(Eigen::Matrix2f rotate)
{
	if (lastFrameObject)
	{
		for (auto &val : *lastFrameObject)
		{
		}
	}
}

void Match::rotateSelf(double angle)
{
}

double Match::distance(Eigen::Vector3f &v)
{
	float x = v(0);
	float y = v(1);
	float z = v(2);
	return sqrt(x * x + y * y + z * z);	
};

double Match::distance(Eigen::Vector3f &first, Eigen::Vector3f &second)
{
	float x = first(0) - second(0);
	float y = first(1) - second(1);
	float z = first(2) - second(2);
	return sqrt(x * x + y * y + z * z);
}


double Match::forecastPosCheck(skywell::Object first, skywell::Object second)
{
	double probability = 0;
	double radius = 10.0;
	//double distanc = distance(first.transform, second.transform);
	//double error_distanc = distance(first.forecast_next_transform, second.transform);
	//probability = 1.0 /(1.0 + fabs(distanc - error_distanc)/(distanc + error_distanc));
	double error_distanc = distance(first.forecast_next_transform, second.transform);
	probability = (radius - error_distanc) / radius;

	return probability;
};

static int iii = 0;
//计算两个物体匹配概率
double Match::calculatepredictedvalue(skywell::Object first, skywell::Object second)
{
	// 位置概率
	double pos = positionCheck(first, second);
	if (iii == 1)
		return pos;
	// 俯视面积
	double area = areaCheck(first, second);
	if (iii == 2)
		return area;
	// 体积
	double vol = volumeCheck(first, second);
	if (iii == 3)
		return vol;

	return (pos + area + vol)/3.0;
}

void directCalculation(Object &first, Object &second)
{
	//second.speedx = (second.transform.x() - first.transform.x()) / TIME_FRAME;
	//second.speedy = (second.transform.y() - first.transform.y()) / TIME_FRAME;
}

bool Match::isvaild(pcl::PointXYZ &point)
{
	if ((std::isnan(point.x))||(std::isnan(point.y))||(std::isnan(point.z))||(std::isinf(point.x))||(std::isinf(point.y))||(std::isinf(point.z)))
	{
		return false;
	}
	return true;
};
 void
Match::nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}


void Match::vfh_match(boost::shared_ptr<std::vector<skywell::Object>> cubes, boost::shared_ptr<std::vector<skywell::Object>> target)
{
	if (cubes->size() == 0)
	{
		return ;
	}
	//汽车移动,每帧向前运动10厘米
	//transformSelf({ -0.1f,0,0 });
	// 存储两场数据都存在的物体
	static unsigned int counter = 1;
	static unsigned int matchtag = 1;
	std::vector<vfh_model> models;
	matchtag++;
	//第一帧时,记录位置
	if (!lastFrameObject)
	{
		for (auto &obj : *cubes)
		{
			//给每个模型编号
			obj.number = (counter++)%1000;
			obj.life = 1;
			//初始化每个模型
			obj.kal.init(obj.transform.x(), obj.transform.y());
		}
	}
	else
	{
		int lastSize = lastFrameObject->size();
		int thisSize = cubes->size();
		//存入数据
		 std::vector<vfh_model> models;
		for (int i = 0; i < lastSize; i++)
		{
			vfh_model vfh_m;
			pcl::PointCloud <pcl::VFHSignature308> point = *((*lastFrameObject)[i].vfh.getVFHs());
			vfh_m.second.resize (308);
			for (int index = 0;index < 308 ; ++ index)
			{
				vfh_m.second[index] = point.points[0].histogram[index];
			}
			vfh_m.first = (*lastFrameObject)[i].number;
			models.push_back(vfh_m);
		}
		flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
		for (size_t i = 0; i < data.rows; ++i)
			for (size_t j = 0; j < data.cols; ++j)
			  data[i][j] = models[i].second[j];
		flann::Index<flann::ChiSquareDistance<float> > findex (data, flann::LinearIndexParams ());
		findex.buildIndex ();

		for (size_t i = 0; i < thisSize; i++)
		{
			int k = 6;
			if (lastSize < 6)
			{
				k = lastSize;
			}
			flann::Matrix<int> k_indices;
			flann::Matrix<float> k_distances;
			vfh_model histogram;
			pcl::PointCloud <pcl::VFHSignature308> point = *((*cubes)[i].vfh.getVFHs());
			histogram.second.resize (308);
			for (int index = 0;index < 308 ; ++ index)
			{
				histogram.second[index] = point.points[0].histogram[index];
			}
			histogram.first = i;
			nearestKSearch (findex, histogram, k, k_indices, k_distances);
			double tempdistance = 100.0;
			int _maxi = -1;
			for (int _k = 0;_k < k ; ++_k)
			{
				printf("%d -- %d (%d) with a distance of:%f\n",_k,models.at (k_indices[0][_k]).first,k_indices[0][_k],k_distances[0][_k]);
				if (k_distances[0][_k] < tempdistance)
				{
					tempdistance = k_distances[0][_k];
					_maxi = k_indices[0][_k];
				}
			}

			bool ismatch = false;
			if (_maxi >= 0)
			{
				printf("_maxi = %d\n",_maxi);
				printf("(lastFrameObject).number = %d\n",(*lastFrameObject)[_maxi].number);
				ismatch = true;
				//继承以前模型
				(*lastFrameObject)[_maxi].matchtag = matchtag;
				(*cubes)[i].number = (*lastFrameObject)[_maxi].number;
				(*cubes)[i].life = (*lastFrameObject)[_maxi].life < 100 ? (*lastFrameObject)[_maxi].life + 1 : 100;
				(*cubes)[i].datatime = time(NULL);
				//更新速度
				Eigen::Vector4d status = (*lastFrameObject)[_maxi].kal.filter((*cubes)[i].transform.x(), (*cubes)[i].transform.y());
				(*cubes)[i].speedx = status(2);
				(*cubes)[i].speedy = status(3);
				(*cubes)[i].forecast_next_transform.x() = (*cubes)[i].transform.x() + (*cubes)[i].speedx * TIME_FRAME; // 忽略加速度
				(*cubes)[i].forecast_next_transform.y() = (*cubes)[i].transform.y() + (*cubes)[i].speedy * TIME_FRAME;
				(*cubes)[i].forecast_next_transform.z() = (*cubes)[i].transform.z();
				(*cubes)[i].continued = (*lastFrameObject)[_maxi].continued < 5 ? (*lastFrameObject)[_maxi].continued + 1:5;
				(*cubes)[i].kal = (*lastFrameObject)[_maxi].kal;
				target->push_back((*cubes)[i]);
			}
			if (!ismatch)
			{
				(*cubes)[i].life = 1;
				(*cubes)[i].datatime = time(NULL);
				(*cubes)[i].number = (counter++)%1000;
				(*cubes)[i].kal.init((*cubes)[i].transform.x(), (*cubes)[i].transform.y());
				(*cubes)[i].forecast_next_transform.x() = (*cubes)[i].transform.x();
				(*cubes)[i].forecast_next_transform.y() = (*cubes)[i].transform.y();
				(*cubes)[i].forecast_next_transform.z() = (*cubes)[i].transform.z();
			}
		}
		for (size_t i = 0; i < lastFrameObject->size(); i++)
		{
			// 如果上一场数据没有匹配上，将上一场聚类结果保存到下一场匹配，保存时间2秒
			if ((*lastFrameObject)[i].matchtag != matchtag)
			{
				if (((*lastFrameObject)[i].continued > 0)&&((*lastFrameObject)[i].life > 5))
				{
					// 按照上一次速度做一个偏移，根据上一场数据速度，按照100毫秒间隔进行偏移
					(*lastFrameObject)[i].transform.x() = (*lastFrameObject)[i].transform.x() + (*lastFrameObject)[i].speedx * TIME_FRAME; // 忽略加速度
					(*lastFrameObject)[i].transform.y() = (*lastFrameObject)[i].transform.y() + (*lastFrameObject)[i].speedy * TIME_FRAME; // 忽略加速度
					//与本场数据进行合并
					(*lastFrameObject)[i].continued = (*lastFrameObject)[i].continued - 1;
					cubes->push_back((*lastFrameObject)[i]);
					target->push_back((*lastFrameObject)[i]);
				}
			}
		}
		delete[] data.ptr ();
	}
	lastFrameObject = cubes;
}

void Match::match(boost::shared_ptr<std::vector<skywell::Object>> cubes, boost::shared_ptr<std::vector<skywell::Object>> target)
{
	if (cubes->size() == 0)
	{
		return ;
	}
	//汽车移动,每帧向前运动10厘米
	//transformSelf({ -0.1f,0,0 });
	// 存储两场数据都存在的物体
	static unsigned int counter = 1;
	static unsigned int matchtag = 1;
	std::vector<vfh_model> models;
	matchtag++;
	//第一帧时,记录位置
	if (!lastFrameObject)
	{
		for (auto &obj : *cubes)
		{
			//给每个模型编号
			obj.number = (counter++)%1000;
			obj.life = 1;
			//初始化每个模型
			obj.kal.init(obj.transform.x(), obj.transform.y());
		}
	}
	else
	{
		const int radius = 10;
		int lastSize = lastFrameObject->size();
		int thisSize = cubes->size();
		std::vector<double> matchTable(lastSize * thisSize);
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> allData = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		allData->width = lastSize + thisSize;
		allData->height = 1;
		allData->points.resize(allData->width * allData->height);
		//存入数据
		 std::vector<vfh_model> models;
		for (int i = 0; i < lastSize; i++)
		{
			allData->points[i].x = (*lastFrameObject)[i].transform.x();
			allData->points[i].y = (*lastFrameObject)[i].transform.y();
			allData->points[i].z = (*lastFrameObject)[i].transform.z();
			/*vfh_model vfh_m;
			pcl::PointCloud <pcl::VFHSignature308> point = *((*lastFrameObject)[i].vfh.getVFHs());
			vfh_m.second.resize (308);
			for (int index = 0;index < 308 ; ++ index)
			{
				vfh_m.second[index] = point.points[0].histogram[index];
			}
			vfh_m.first = (*lastFrameObject)[i].number;
			models.push_back(vfh_m);*/
		}

		/*
		flann::Matrix<float> data (new float[models.size () * models[0].second.size ()], models.size (), models[0].second.size ());
		for (size_t i = 0; i < data.rows; ++i)
			for (size_t j = 0; j < data.cols; ++j)
			  data[i][j] = models[i].second[j];
		flann::Index<flann::ChiSquareDistance<float> > findex (data, flann::LinearIndexParams ());
		findex.buildIndex ();
		*/


		for (int i = 0; i < thisSize; i++)
		{
			allData->points[lastSize+i].x = (*cubes)[i].transform.x();
			allData->points[lastSize+i].y = (*cubes)[i].transform.y();
			allData->points[lastSize+i].z = (*cubes)[i].transform.z();
		}

		//建立kd数范围搜索
		boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> kdtree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
		kdtree->setInputCloud(allData);

		//使用kd树进行范围搜索
		for (size_t i = 0; i < lastFrameObject->size(); i++)
		{
			std::vector<int> index;	  //索引
			std::vector<float> value; //值
			if (!isvaild((*allData)[i]))
			{
				continue;
			}
			kdtree->radiusSearch((*allData)[i], radius, index, value);
			//对于上一帧中每一个模型需要搜索
			for (auto j : index)
			{
				if (j > lastSize - 1) // this序号一定大于lastSize-1，因为上面两部分合并的时候顺序是先插入last，后插入this。在半径搜索的结果中存在last和this的两个点，只有当index序号大于lastSize-1 才表示搜索到的点是在this中，匹配才有意义
				{
					//计算概率并匹配,存入矩阵表
					double probability = calculatepredictedvalue((*lastFrameObject)[i], (*cubes)[j - lastSize]);
					//printf("probability:%f\n",probability);
					matchTable[i * thisSize + j - lastSize] = probability;
				}
			}
		}
		//printf("=======================================\n");
		//寻找最优概率,处理表
		for (size_t i = 0; i < thisSize; i++)
		{
			//最优位置与最优值
			int maxi[3] = {-1,-1,-1};
			double max[3] = {0.0,0.0,0.0};
			double tempmax = 0.01;
			int index = 3;
			//寻找最优值
			for (size_t j = 0; j < lastSize; j++)
			{
				tempmax = matchTable[j * thisSize + i];
				for (int _i = 0; _i< index; _i++)
				{
					if (max[_i] < tempmax)
					{
						for (int k = index - 1; k > _i ;k-- )
						{
							max[k] = max[k -1];
							maxi[k] = maxi[k - 1];
						}
						max[_i] = tempmax;
						maxi[_i] = j;
						break;
					}
				}
			}
			bool ismatch = false;
			for (int _ii = 0; _ii < index ; _ii++)
			{
				if (maxi[_ii] > -1)
				{
					int _maxi = maxi[_ii];
					double _max = max[_ii];
					bool is_optimal = true;
					for (size_t k = 0; k < thisSize ;k++ )
					{
						if (matchTable[_maxi * thisSize + k] > _max)
						{
							// 列最优不是行最优
							is_optimal = false;
							break;
						}
					}
					if (is_optimal)
					{

						//这里可以根据位移变化、速度变化、俯视面积变化程度做一次滤波，一个障碍物，无论是静止还是运动的，这三个值得变化都应该是平滑的，有上下限的
						//double disan = distance((*lastFrameObject)[_maxi].transform, (*cubes)[i].transform);
						//double last_radius = distance((*lastFrameObject)[_maxi].transform);
						//double this_radius = distance((*cubes)[i].transform);
						double forecast_probability = forecastPosCheck((*lastFrameObject)[_maxi],(*cubes)[i]);
						//double probability = calculatepredictedvalue((*lastFrameObject)[_maxi],(*cubes)[i]);
						//double pos_probability =  positionCheck((*lastFrameObject)[_maxi],(*cubes)[i]);
						double area_probability = areaCheck((*lastFrameObject)[_maxi],(*cubes)[i]);
						//double vol_probability = volumeCheck((*lastFrameObject)[_maxi],(*cubes)[i]);
						// 面积匹配概率大于0.7，位置预测概率大于0.9
						if ((area_probability > 0.7)&&(forecast_probability > 0.9))
						{
							/*
							int k = 6;
							if (lastSize < 6)
							{
								k = lastSize;
							}
							flann::Matrix<int> k_indices;
							flann::Matrix<float> k_distances;
							vfh_model histogram;
							pcl::PointCloud <pcl::VFHSignature308> point = *((*cubes)[i].vfh.getVFHs());
							histogram.second.resize (308);
							for (int index = 0;index < 308 ; ++ index)
							{
								histogram.second[index] = point.points[0].histogram[index];
							}
							histogram.first = i;
							nearestKSearch (findex, histogram, k, k_indices, k_distances);
							printf("k = %d\n",k);
							double tempdistance = 100.0;
							int __maxi = 0;
							for (int _k = 0;_k < k ; ++_k)
							{
								if (k_distances[0][_k] < tempdistance)
								{
									tempdistance = k_distances[0][_k];
									__maxi = k_indices[0][_k];
								}
								printf("%d -- %d (%d) with a distance of:%f\n",_k,models.at (k_indices[0][_k]).first,k_indices[0][_k],k_distances[0][_k]);
							}
							printf("__maxi = %d\n",__maxi);
							printf("(lastFrameObject).number = %d\n",(*lastFrameObject)[_maxi].number);
							if (__maxi == _maxi)
							{
								printf("=============OK=============\n");
							}else
							{
								printf("=============No OK=============\n");
							}*/

							ismatch = true;
							//继承以前模型
							(*lastFrameObject)[_maxi].matchtag = matchtag;
							(*cubes)[i].number = (*lastFrameObject)[_maxi].number;
							(*cubes)[i].life = (*lastFrameObject)[_maxi].life < 100 ? (*lastFrameObject)[_maxi].life + 1 : 100;
							(*cubes)[i].datatime = time(NULL);
							//更新速度
							Eigen::Vector4d status = (*lastFrameObject)[_maxi].kal.filter((*cubes)[i].transform.x(), (*cubes)[i].transform.y());
							(*cubes)[i].speedx = status(2);
							(*cubes)[i].speedy = status(3);
							(*cubes)[i].forecast_next_transform.x() = (*cubes)[i].transform.x() + (*cubes)[i].speedx * TIME_FRAME; // 忽略加速度
							(*cubes)[i].forecast_next_transform.y() = (*cubes)[i].transform.y() + (*cubes)[i].speedy * TIME_FRAME;
							(*cubes)[i].forecast_next_transform.z() = (*cubes)[i].transform.z();
							(*cubes)[i].continued = (*lastFrameObject)[_maxi].continued < 5 ? (*lastFrameObject)[_maxi].continued + 1:5;
							(*cubes)[i].kal = (*lastFrameObject)[_maxi].kal;
							if ((*cubes)[i].life > 5)
							{
								//printf("==================================:%d\n",(*cubes)[i].number);
								target->push_back((*cubes)[i]);
							}
							break;
						}
					}
				}else
				{
					break;
				}
			}
			if (!ismatch)
			{
				(*cubes)[i].life = 1;
				(*cubes)[i].datatime = time(NULL);
				(*cubes)[i].number = (counter++)%1000;
				(*cubes)[i].kal.init((*cubes)[i].transform.x(), (*cubes)[i].transform.y());
				(*cubes)[i].forecast_next_transform.x() = (*cubes)[i].transform.x();
				(*cubes)[i].forecast_next_transform.y() = (*cubes)[i].transform.y();
				(*cubes)[i].forecast_next_transform.z() = (*cubes)[i].transform.z();
			}
		}
		for (size_t i = 0; i < lastFrameObject->size(); i++)
		{
			// 如果上一场数据没有匹配上，将上一场聚类结果保存到下一场匹配，保存时间2秒
			if ((*lastFrameObject)[i].matchtag != matchtag)
			{
				if (((*lastFrameObject)[i].continued > 0)&&((*lastFrameObject)[i].life > 5))
				{
					// 按照上一次速度做一个偏移，根据上一场数据速度，按照100毫秒间隔进行偏移
					(*lastFrameObject)[i].transform.x() = (*lastFrameObject)[i].transform.x() + (*lastFrameObject)[i].speedx * TIME_FRAME; // 忽略加速度
					(*lastFrameObject)[i].transform.y() = (*lastFrameObject)[i].transform.y() + (*lastFrameObject)[i].speedy * TIME_FRAME; // 忽略加速度
					//与本场数据进行合并
					(*lastFrameObject)[i].continued = (*lastFrameObject)[i].continued - 1;
					cubes->push_back((*lastFrameObject)[i]);
					target->push_back((*lastFrameObject)[i]);
				}
			}
		}
		//delete[] data.ptr ();
	}
	lastFrameObject = cubes;
}

//获取上一个的模型
boost::shared_ptr<std::vector<skywell::Object>> Match::getlast()
{
	if (lastFrameObject)
	{
		return lastFrameObject;
	}
	return boost::make_shared<std::vector<skywell::Object>>();
}
//位置算概率
double Match::positionCheck(Object &first, Object &second)
{

	double probability = 0;
	double radius = 10;
	double disan = distance(first.transform, second.transform);
	probability = (radius - disan) / radius;
	return probability;
}

//角度算概率
double Match::rotationCheck(Object &first, Object &second)
{

	double probability = 0;
	double theta1 = acos(first.rotate.w()) * 2.0;
	double theta2 = acos(second.rotate.w()) * 2.0;
	probability = 1.0 / (1.0 + fabs(theta1 - theta2));
	return probability;
}

//体积算概率
double Match::volumeCheck(Object &first, Object &second)
{
	double probability = 0;
	double first_vol = first.width * first.length * first.height;
	double second_vol = second.width * second.length * second.height;
	probability = 1.0 /(1.0 + fabs(first_vol - second_vol)/(first_vol + second_vol));
	return probability;
}

//俯视面积算概率
double Match::areaCheck(Object &first, Object &second)
{
	double probability = 0;
	double first_vol = first.width * first.length ;
	double second_vol = second.width * second.length;
	probability = 1.0 /(1.0 + fabs(first_vol - second_vol)/(first_vol + second_vol));
	return probability;
}

} // namespace skywell