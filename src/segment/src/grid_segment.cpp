/************************************************************ 
 Copyright (C), 2019-, skywell-mobility Tech. Co., Ltd. 
 Author:hl
 Version:v1.0
 Date: 2021/1/12
 Description:栅格点云分割，基于Grid类，进行点云地面分割。
*******************************************************/
#include "grid_segment.h"
namespace skywell {


/************************************************* 
 Description:     设置待处理的原始点云 
 Input:          cloud_ptr 点云
 Output:         无 
 Return:          无 
 Author:         hl
 Date:           2021/1/12
*********************************************/
void GridSegment::setInputCloud(pcl::PointCloud<PointT>::Ptr cloud_ptr)
{
	cloud_ptr_ = cloud_ptr;
};


/************************************************* 
 Description:     设置栅格点云的范围 
 Input:          minx,miny,maxx,maxy
 Output:         无 
 Return:          无 
 Author:         hl
 Date:           2021/1/12
*********************************************/
void GridSegment::setGridRange(float minx,float miny,float maxx,float maxy)
{
	minx_ = minx;
	miny_ = miny;
	maxx_ = maxx;
	maxy_ = maxy;
}


/************************************************* 
 Description:     设置栅格尺寸 
 Input:          size
 Output:         无 
 Return:          无 
 Author:         hl
 Date:           2021/1/12
*********************************************/
void GridSegment::setGridSize(float size)
{
	size_ = size;
};


/************************************************* 
 Description:     设置栅格分割方法中高度阀值 
 Input:          height_threshold 高度阀值
 Output:         无 
 Return:          无 
 Author:         hl
 Date:           2021/1/12
*********************************************/
void GridSegment::setHeightThreshold(float height_threshold)
{
	height_threshold_ = height_threshold;
};


/************************************************* 
 Description:     类初始化 
 Input:          无
 Output:         无 
 Return:          无 
 Author:         hl
 Date:           2021/1/12
*********************************************/
void GridSegment::initGrid(void)
{
	col_ = (maxx_ - minx_) / size_;
	row_ = (maxy_ - miny_) / size_;
	int _grid_size = col_ * row_;
	grid_list_->resize(_grid_size);
	for (int i = 0; i < _grid_size; ++i)
	{
		(*grid_list_)[i].is_vaild_ = false;
	}
	
	// 根据到原点距离进行排序，作为栅格判断的顺序，即从原点向四周扩散

	int _origin_col = (maxx_ - 0.0) / size_;
	int _origin_row = (maxy_ - 0.0) / size_;

	for (int i = 0; i < col_ ; i++)
	{
		int _tf_origin_col = i - _origin_col;
		for (int j = 0;j < row_ ;j++ )
		{
			int _tf_origin_row = j - _origin_row;

			int _origin_radius = sqrt(_tf_origin_col * _tf_origin_col + _tf_origin_row * _tf_origin_row);
			int _index = i * row_ + j;
			
			GridIndex _grid_index;
			_grid_index.radius_ = _origin_radius;
			_grid_index.index_ = _index;
			_grid_index.col_ = i;
			_grid_index.row_ = j;

			(*grid_list_)[_index].col_ = i;
			(*grid_list_)[_index].row_ = j;
			(*grid_list_)[_index].index_ = 0;

			grid_index_list_.push_back(_grid_index);
		}
	}

	std::sort(grid_index_list_.begin(), grid_index_list_.end(),
	[](const GridIndex &a, const GridIndex &b) { return a.radius_ < b.radius_; });
	for (int i = 0; i < _grid_size ; ++i)
	{
		int _index = grid_index_list_[i].index_;
		(*grid_list_)[_index].index_ = i;
	}

};

/************************************************* 
 Description:     点云栅格处理 
 Input:          无
 Output:         无 
 Return:          无 
 Author:         hl
 Date:           2021/1/12
*********************************************/
void GridSegment::processGrid(void)
{
	size_t _point_size = cloud_ptr_->points.size();
	for (size_t i = 0; i < _point_size ; ++i)
	{
		if ((cloud_ptr_->points[i].x > minx_) 
			&& (cloud_ptr_->points[i].x < maxx_)
			&& (cloud_ptr_->points[i].y > miny_) 
			&& (cloud_ptr_->points[i].y < maxy_))
		{
			int _col = (maxx_ - cloud_ptr_->points[i].x) / size_;
			int _row = (maxy_ - cloud_ptr_->points[i].y) / size_;
			int _index = _col * row_ + _row;

			if ((*grid_list_)[_index].is_vaild_ == false)
			{
				(*grid_list_)[_index].is_vaild_ = true;
			}
			(*grid_list_)[_index].addPoint(cloud_ptr_->points[i],i);
		}
	}

};

/************************************************* 
 Description:     一次栅格校验，对index序号的栅格进行校验
 Input:          index 栅格序号，提供的参考地面栅格集
 Output:         无 
 Return:          无 
 Author:         hl
 Date:           2021/1/19
*********************************************/
void GridSegment::firstCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list)
{
	// check_flag_  校验状态，-1 栅格未进行校验， 1 正在进行校验， 0 校验结束
	if ((*grid_list_)[index].check_flag_ == -1)
	{
		(*grid_list_)[index].check_flag_ = 1;

		// 栅格高度差小于阀值 且 栅格内点云平均高度小于阀值，认定栅格为地面栅格
		if (((*grid_list_)[index].current_feature_.abs_height_ < height_threshold_)
			&&((*grid_list_)[index].current_feature_.avg_z_ < height_threshold_)
			&&((*grid_list_)[index].current_feature_.avg_z_ > - height_threshold_))
		{
			
			(*grid_list_)[index].check_flag_ = 0;
			(*grid_list_)[index].ground_flag_ = 0;

			// 判断为地面栅格，需要提取地面栅格特征,以供周边栅格参考
			CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);

			// 判断为地面栅格，需要保存地面栅格,以供后续栅格校验参考
			(*ground_grid_list)[index].UpdateGroundFeature((*grid_list_)[index].ground_feature_);
		}
		else
		{
			// 不能满足设定的阀值，
			// 通过获得栅格周围栅格，如果周围栅格是地面栅格，
			// 则找到一个相对优的栅格快作为参考，计算栅格与参考栅格的相似度，
			// 满足80%的相似度，任务当前栅格也是地面栅格

			std::vector<int> _range_indexs;
			getRangeIndex((*grid_list_)[index].col_,(*grid_list_)[index].row_,_range_indexs);

			int _ground_points_size = 0;

			float _min_z = std::numeric_limits<float>::max();
			float _max_z = -std::numeric_limits<float>::max();
		
			float _optimal_probability = 0.0;
			int _optimal_index_ = 0;

			for (auto num : _range_indexs)
			{
				if ((*grid_list_)[num].is_vaild_ == false)
				{
					continue;
				}else
				{
					if ((*grid_list_)[num].check_flag_ == -1)
					{
						firstCheckGrid(num,ground_grid_list);
					}
					
					if (((*grid_list_)[num].ground_flag_ == 0)&&((*grid_list_)[num].current_feature_.point_size_ > 2))
					{
						float _probability = calProbability((*grid_list_)[index].current_feature_,(*grid_list_)[num].ground_feature_);
						if ((*grid_list_)[num].current_feature_.point_size_ > _ground_points_size)
						{
							_optimal_index_ = num;
							_optimal_probability = _probability;
							_ground_points_size = (*grid_list_)[num].current_feature_.point_size_;
						}
					}
				}
			}
			if (_optimal_index_ > 0)
			{
				if (_optimal_probability > 0.80)
				{
					(*grid_list_)[index].check_flag_ = 0;
					(*grid_list_)[index].ground_flag_ = 0;
					CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
					(*ground_grid_list)[index].UpdateGroundFeature((*grid_list_)[index].ground_feature_);
				}
			}else
			{
				// 第一次栅格结束后设置，栅格状态，ground_flag_ = -1 是为判断出结果栅格
				(*grid_list_)[index].check_flag_ = 0;
				(*grid_list_)[index].ground_flag_ = -1;
			}
		}
	}
};

/************************************************* 
 Description:     二次栅格校验，对index序号的栅格进行校验
 Input:          index 栅格序号，ground_grid_list 提供的参考地面栅格集
 Output:         无 
 Return:          无 
 Author:         hl
 Date:           2021/1/19
*********************************************/
void GridSegment::secondCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list)
{
	
	// ground_flag_ 栅格地面状态，-1 为判断出结果
	if ((*grid_list_)[index].ground_flag_ == -1)
	{

		// 通过获得栅格周围栅格，如果周围栅格是地面栅格，
		// 则找到一个相对优的栅格快作为参考，
		// 并对栅格进行二次栅格，以更小的尺寸进行栅格，这里设定 0.2米
		// 二次栅格判断将以周围地面块作为参考

		std::vector<int> _range_indexs;
		getRangeIndex((*grid_list_)[index].col_,(*grid_list_)[index].row_,_range_indexs);
		int _ground_points_size = 0;

		float _min_z = std::numeric_limits<float>::max();
		float _max_z = -std::numeric_limits<float>::max();
	
		float _optimal_probability = 0.0;
		int _optimal_index_ = 0;

		for (auto num : _range_indexs)
		{
			if ((*grid_list_)[num].is_vaild_ == false)
			{
				continue;
			}else
			{
				if (((*grid_list_)[num].ground_flag_ == 0)&&((*grid_list_)[num].current_feature_.point_size_ > 2))
				{
					if ((*grid_list_)[num].current_feature_.point_size_ > _ground_points_size)
					{
						_optimal_index_ = num;
						_ground_points_size = (*grid_list_)[num].current_feature_.point_size_;
					}
				}
			}
		}
		if (_optimal_index_ > 0)
		{

			// 进行二次栅格，栅格尺寸0.2米
			SubGrid _sub_grid;
			_sub_grid.segment((*grid_list_)[index],0.2,height_threshold_,
				(*grid_list_)[_optimal_index_].current_feature_,
				(*ground_grid_list)[index],
				(*grid_list_)[index].grid_ground_inliers_);
				(*grid_list_)[index].check_flag_ = 0;
				(*grid_list_)[index].ground_flag_ = 1;
		}
		else
		{
			// 没有在栅格周围的八个栅格中找到参考，继续向四周扩散，找到最近的一个作为参考
			//
			int _ground_points_size = 0;
			int _optimal_index_ = 0;
			std::vector<int> _range_indexs;
			searchNearestGroundGrid(index,3.0,_range_indexs);

				// 根据栅格到原点的距离计算一个阀值
			float _radius = grid_index_list_[(*grid_list_)[index].index_].radius_;
			float _ground_threshold = (_radius / 5.0) * height_threshold_;

			float _avg_z = - std::numeric_limits<float>::max();
			for (auto num : _range_indexs)
			{
				if ((*grid_list_)[num].ground_feature_.point_size_ > 2)
				{
					if ((*grid_list_)[num].ground_feature_.avg_z_ > _avg_z)
					{
						_optimal_index_ = num;
						_avg_z = (*grid_list_)[num].ground_feature_.avg_z_;
					}
				}
				//printf("_optimal_index_ =%d\n",_optimal_index_);
			}
			if (_optimal_index_ > 0)
			{
				// 计算找到的栅格与当前栅格的距离
				// 根据距离计算点云去除的阀值
				float current_col = (*grid_list_)[index].col_;
				float cuurent_row = (*grid_list_)[index].row_;

				float optimal_col = (*grid_list_)[_optimal_index_].col_;
				float optimal_row = (*grid_list_)[_optimal_index_].row_;

				float _distance = sqrt((current_col - optimal_col)*(current_col - optimal_col) + (cuurent_row - optimal_row)*(cuurent_row - optimal_row));
				for (int i = 0 ; i < (*grid_list_)[index].current_feature_.point_size_ ; i++)
				{
					if (((*grid_list_)[index].grid_cloud_[i].z < _avg_z + (_distance * 0.1) )
						|| ((*grid_list_)[index].grid_cloud_[i].z < _ground_threshold))
					{
						(*grid_list_)[index].grid_ground_inliers_.indices.push_back((*grid_list_)[index].grid_inliers_.indices[i]);
					}
				}
				(*grid_list_)[index].check_flag_ = 0;
				(*grid_list_)[index].ground_flag_ = 1;
			}else
			{
				if ((*grid_list_)[index].current_feature_.abs_height_ < height_threshold_)
				{
					if ((*grid_list_)[index].current_feature_.avg_z_ < _ground_threshold)
					{
						(*grid_list_)[index].check_flag_ = 0;
						(*grid_list_)[index].ground_flag_ = 0;
						CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
					}
				}else
				{
					/*
					float _ratio = (float)(*grid_list_)[index].current_feature_.down_centr_z_points_ 
													/ (*grid_list_)[index].current_feature_.point_size_;
					float _ratio_height = ((*grid_list_)[index].current_feature_.avg_z_ - (*grid_list_)[index].current_feature_.min_z_);
					*/
					/*if ((*grid_list_)[index].current_feature_.abs_height_ < height_threshold_)
					{
						if ((*grid_list_)[index].current_feature_.avg_z_ < _ground_threshold + height_threshold_)
						{
							(*grid_list_)[index].check_flag_ = 0;
							(*grid_list_)[index].ground_flag_ = 0;
							CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
						}
					}else*/
					{
						for (int i = 0 ; i < (*grid_list_)[index].current_feature_.point_size_ ; i++)
						{
							if ((*grid_list_)[index].grid_cloud_[i].z < _ground_threshold)
							{
								(*grid_list_)[index].grid_ground_inliers_.indices.push_back((*grid_list_)[index].grid_inliers_.indices[i]);
								(*grid_list_)[index].addGroundPoint((*grid_list_)[index].grid_cloud_[i]);
							}
						}
						(*grid_list_)[index].check_flag_ = 0;
						(*grid_list_)[index].ground_flag_ = 1;
					}
				}
			}
		}
	}
};

/************************************************* 
 Description:     栅格分割的主函数入口 
 Input:          ground_grid_list 提供参考的地面栅格集，可以在sgment过程中对其更新
 Output:         out_indices  存储分割结果地面点云序号集
 Return:          无 
 Author:         hl
 Date:           2021/1/20
*********************************************/
void GridSegment::segment(boost::shared_ptr<std::vector<Grid>> &ground_grid_list,pcl::PointIndices &out_indices)
{
	// ground_grid_list 历史地面参考集，程序启动时，可能为空，
	//需要进行初始化
	if (ground_grid_list->size() == 0)
	{
		int _grid_size = col_ * row_;
		ground_grid_list->resize(_grid_size);
		for (int i = 0; i < _grid_size; ++i)
		{
			(*ground_grid_list)[i].is_vaild_ = false;
		}
	}
	

	// 对栅格进行第一次校验，目的是判断比较明显的地面栅格
	int _grid_size = grid_index_list_.size();
	
	for (int i = 0; i < _grid_size ; ++i)
	{
		int _index = grid_index_list_[i].index_;

		if ((*grid_list_)[_index].is_vaild_)
		{
			(*grid_list_)[_index].distributeStatistics();
			firstCheckGrid(_index,ground_grid_list);
		}
	}
	

	// 对栅格进行第二次校验，参考第一次栅格结果，进行二次栅格
	for (int i = 0; i < _grid_size ; ++i)
	{
		int _index = grid_index_list_[i].index_;
		if ((*grid_list_)[_index].is_vaild_)
		{
			secondCheckGrid(_index,ground_grid_list);
		}
	}

	// 提取栅格内地面点云
	for (int i = 0; i < _grid_size ; ++i)
	{
		// ground_flag_ == 0  栅格内点云全部是地面点云
		if ((*grid_list_)[i].ground_flag_ == 0)
		{
			out_indices.indices.insert(out_indices.indices.end(),
				(*grid_list_)[i].grid_inliers_.indices.begin(),
				(*grid_list_)[i].grid_inliers_.indices.end());
		}

		// ground_flag_ == 1  栅格内点云部分是地面点云
		if ((*grid_list_)[i].ground_flag_ == 1)
		{
			out_indices.indices.insert(out_indices.indices.end(),
				(*grid_list_)[i].grid_ground_inliers_.indices.begin(),
			(*grid_list_)[i].grid_ground_inliers_.indices.end());
		}
	}

};


/************************************************* 
 Description:     根据栅格行、列号计算栅格序列号 
 Input:          col 行号，row 列号
 Output:         无
 Return:          无 
 Author:         hl
 Date:           2021/1/13
*********************************************/
int GridSegment::getIndex(int col,int row)
{
	if ((col >= 0)&&(col <= col_) && (row >= 0) && (row <= row_))
	{
		return (col* row_ + row);
	}
	return -1;
};


/************************************************* 
 Description:     获得栅格周围8个栅格索引 
 Input:          col,row 当前栅格的行号,列号
 Output:         index_vec 输出周围栅格索引
 Return:          无 
 Author:         hl
 Date:           2021/1/13
*********************************************/
void GridSegment::getRangeIndex(int col,int row ,std::vector<int> &index_vec)
{
	//查找周围八个栅格
	int _index = -1;
	_index = getIndex(col,row - 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col - 1,row - 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col - 1,row);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col - 1,row + 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col ,row + 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col + 1,row + 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col + 1,row);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col + 1,row - 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
};


/************************************************* 
 Description:     获得坐标原点所在的栅格序号
 Input:          无
 Output:         无
 Return:         int 坐标原点所在的栅格序号
 Author:         hl
 Date:           2021/1/13
*********************************************/
int GridSegment::getOriginIndex(void)
{
	 int _col = (maxx_ - 0.0) / size_;
	 int _row = (maxy_ - 0.0) / size_;
	 return (_col* row_ + _row);
};


/************************************************* 
 Description:     获得栅格周围一定范围内的地面栅格序号
 Input:         index 当前栅格序列号， distance 范围
 Output:         index_vec 输出周围栅格索引
 Return:         无
 Author:         hl
 Date:           2021/1/20
*********************************************/
void GridSegment::searchNearestGroundGrid(int index,float distance,std::vector<int> &index_vec)
{
	// 获得当前列号，行号
	int _col = (*grid_list_)[index].col_;
	int _row = (*grid_list_)[index].row_;

	int _max_col,_max_row;
	int _min_col,_min_row;

	if ((_col + distance) < col_)
	{
		_max_col = ceil(_col + distance);
	}else
	{
		_max_col = col_;
	}

	if ((_row + distance) < row_)
	{
		_max_row = ceil(_row + distance);
	}else
	{
		_max_row = row_;
	}

	if ((_col - distance) > 0)
	{
		_min_col = floor(_col - distance);
	}else
	{
		_min_col = 0;
	}

	if ((_row - distance) > 0)
	{
		_min_row = floor(_row - distance);
	}else
	{
		_min_row = row_;
	}

	for (int i = _min_col; i < _max_col; i++)
	{
		for (int j = _min_row; j < _max_row; j++)
		{
			int _index = (i* row_ + j);
			if (_index != index)
			{
				index_vec.push_back(_index);
			}
		}
	}
}


/************************************************* 
 Description:     根据栅格特征，计算栅格的相似度
 Input:          Feature 栅格特征集
 Output:         无
 Return:         float  相似度， 范围(0 ~ 1)
 Author:         hl
 Date:           2021/1/16
*********************************************/
float GridSegment::calProbability( Feature &first,Feature &second)
{
	float _min_z_probability = 1.0
		 / (1.0 + fabs(first.min_z_ - second.min_z_)
		 / (fabs(first.min_z_) + fabs(second.min_z_)));

	float _max_z_probability =  1.0 
		 / (1.0 + fabs(first.max_z_ - second.max_z_)
		 / (fabs(first.max_z_) + fabs(second.max_z_)));

	float _deviation_z_probability = 1.0 
		 / (1.0 + fabs(first.deviation_z_ - second.deviation_z_)
		 /(fabs(first.deviation_z_) + fabs(second.deviation_z_)));

	float _abs_height_probability = 1.0
		 / (1.0 + fabs(first.abs_height_ - second.abs_height_)
		 / (first.abs_height_ + second.abs_height_));

	float _avg_z_probability =  1.0
		 / (1.0 + fabs(first.avg_z_ - second.avg_z_)
		 / (fabs(first.avg_z_) + fabs(second.avg_z_)));

	float _probability = ( _deviation_z_probability
						 + _min_z_probability
						 + _max_z_probability
						 + _abs_height_probability
						 + _avg_z_probability)/5.0;

	return _probability;

};


/************************************************* 
 Description:    计算两个浮点数的相似度
 Input:          float 
 Output:         无
 Return:         float  相似度， 范围(0 ~ 1)
 Author:         hl
 Date:           2021/1/18
*********************************************/
float GridSegment::calProbability(float first,float second)
{
	float _probability = 1.0
	 / (1.0 + fabs(first - second)
	 / (fabs(first) + fabs(second)));
	return _probability;
};


/************************************************* 
 Description:    复制特征信息，将source特征信息复制给target
 Input:          Feature 特征信息 
 Output:         无
 Return:         无
 Author:         hl
 Date:           2021/1/18
*********************************************/
void GridSegment::CopyFeature(Feature &source,Feature &target)
{
	target.min_z_ = source.min_z_;
	target.max_z_ = source.max_z_;
	target.deviation_z_ = source.deviation_z_;
	target.abs_height_ = source.abs_height_;
	target.avg_z_ = source.avg_z_;
	target.point_size_ = source.point_size_;
};

}