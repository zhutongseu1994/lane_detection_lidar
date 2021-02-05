/************************************************************ 
 Copyright (C), 2019-, skywell-mobility Tech. Co., Ltd. 
 Author:hl
 Version:v1.0
 Date: 2021/1/12
 Description:栅格点云分割，基于Grid类，进行点云地面分割。
*******************************************************/
#ifndef _GRID_SEGMENT_H_
#define _GRID_SEGMENT_H_

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

#include "grid.h"

namespace skywell {

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;


class GridIndex
{
	public:
		int index_;
		int radius_;
		int col_;
		int row_;
};

// GridSegment 基于栅格分割类，该类封装了对原始点云进行栅格分割的属性和方法

class GridSegment
{
	public:

		/************************************************* 
		 Description:     设置待处理的原始点云 
		 Input:          cloud_ptr 点云
		 Output:         无 
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/12
		*********************************************/
		void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud_ptr);

		/************************************************* 
		 Description:     设置栅格点云的范围 
		 Input:          minx,miny,maxx,maxy
		 Output:         无 
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/12
		*********************************************/
		void setGridRange(float minx,float miny,float maxx,float maxy);

		/************************************************* 
		 Description:     设置栅格尺寸 
		 Input:          size
		 Output:         无 
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/12
		*********************************************/
		void setGridSize(float size);

		/************************************************* 
		 Description:     设置栅格分割方法中高度阀值 
		 Input:          height_threshold 高度阀值
		 Output:         无 
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/12
		*********************************************/
		void setHeightThreshold(float height_threshold);

		/************************************************* 
		 Description:     类初始化 
		 Input:          无
		 Output:         无 
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/12
		*********************************************/
		void initGrid(void);

		/************************************************* 
		 Description:     点云栅格处理 
		 Input:          无
		 Output:         无 
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/12
		*********************************************/
		void processGrid(void);

		/************************************************* 
		 Description:     一次栅格校验，对index序号的栅格进行校验
		 Input:          index 栅格序号，提供的参考地面栅格集
		 Output:         无 
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/19
		*********************************************/
		void firstCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list);

		/************************************************* 
		 Description:     二次栅格校验，对index序号的栅格进行校验
		 Input:          index 栅格序号，ground_grid_list 提供的参考地面栅格集
		 Output:         无 
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/19
		*********************************************/
		void secondCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list);

		/************************************************* 
		 Description:     栅格分割的主函数入口 
		 Input:          ground_grid_list 提供参考的地面栅格集，可以在sgment过程中对其更新
		 Output:         out_indices  存储分割结果地面点云序号集
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/20
		*********************************************/
		void segment(boost::shared_ptr<std::vector<Grid>> &ground_grid_list,pcl::PointIndices &out_indices);
	private:

		/************************************************* 
		 Description:     根据栅格行、列号计算栅格序列号 
		 Input:          col 行号，row 列号
		 Output:         无
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/13
		*********************************************/
		int getIndex(int col,int row);

		/************************************************* 
		 Description:     获得栅格周围8个栅格索引 
		 Input:          col,row 当前栅格的行号,列号
		 Output:         index_vec 输出周围栅格索引
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/13
		*********************************************/
		void getRangeIndex(int col,int row ,std::vector<int>  &index_vec);

		/************************************************* 
		 Description:     获得坐标原点所在的栅格序号
		 Input:          无
		 Output:         无
		 Return:         int 坐标原点所在的栅格序号
		 Author:         hl
		 Date:           2021/1/13
		*********************************************/
		int getOriginIndex(void);

		/************************************************* 
		 Description:     获得栅格周围一定范围内的地面栅格序号
		 Input:         index 当前栅格序列号， distance 范围
		 Output:         index_vec 输出周围栅格索引
		 Return:         无
		 Author:         hl
		 Date:           2021/1/20
		*********************************************/
		void searchNearestGroundGrid(int index,float distance,std::vector<int> &index_vec);

		/************************************************* 
		 Description:     根据栅格特征，计算栅格的相似度
		 Input:          Feature 栅格特征集
		 Output:         无
		 Return:         float  相似度， 范围(0 ~ 1)
		 Author:         hl
		 Date:           2021/1/16
		*********************************************/
		float calProbability( Feature &first,Feature &second);

		/************************************************* 
		 Description:    计算两个浮点数的相似度
		 Input:          float 
		 Output:         无
		 Return:         float  相似度， 范围(0 ~ 1)
		 Author:         hl
		 Date:           2021/1/18
		*********************************************/
		float calProbability(float first,float second);

		/************************************************* 
		 Description:    复制特征信息，将source特征信息复制给target
		 Input:          Feature 特征信息 
		 Output:         无
		 Return:         无
		 Author:         hl
		 Date:           2021/1/18
		*********************************************/
		void CopyFeature(Feature &source,Feature &target);

	private:

		typename pcl::PointCloud<PointT>::Ptr cloud_ptr_ {new pcl::PointCloud<PointT>()};
		float minx_,miny_,maxx_,maxy_,size_;
		float height_threshold_ = 0.2;
		std::vector<GridIndex> grid_index_list_;

	public:
		boost::shared_ptr<std::vector<Grid>> grid_list_ = boost::make_shared<std::vector<Grid>>();
		int col_ = 0;
		int row_ = 0;
};



// SubGrid 子栅格分割类，该类封装了对GridSegment 栅格点云进行栅格分割的属性和方法

class SubGrid
{
	public:

		/************************************************* 
		 Description:    复制特征信息，将source特征信息复制给target
		 Input:          Feature 特征信息 
		 Output:         无
		 Return:         无
		 Author:         hl
		 Date:           2021/1/18
		*********************************************/
		void CopyFeature(Feature &source,Feature &target)
		{
			target.min_z_ = source.min_z_;
			target.max_z_ = source.max_z_;
			target.deviation_z_ = source.deviation_z_;
			target.abs_height_ = source.abs_height_;
			target.avg_z_ = source.avg_z_;
			target.point_size_ = source.point_size_;
		};


		/************************************************* 
		 Description:    合并特征信息，将source特征信息合并给target
		 Input:          Feature 特征信息 
		 Output:         无
		 Return:         无
		 Author:         hl
		 Date:           2021/1/18
		*********************************************/
		void mergeFeature(Feature &source,Feature &target)
		{
			if (target.min_z_ > source.min_z_)
			{
				target.min_z_ = source.min_z_;
			}
			if (target.max_z_ < source.max_z_)
			{
				target.max_z_ = source.max_z_;
			}
			if (target.max_z_ < 0)
			{
				target.deviation_z_ = target.max_z_;
			}
			if (target.min_z_ > 0)
			{
				target.deviation_z_ = target.min_z_;
			}
			target.avg_z_ = (target.avg_z_ + source.avg_z_)/2.0;
			target.point_size_ = target.point_size_ + source.point_size_;
		}


		/************************************************* 
		 Description:    栅格校验，对序号index的栅格进行校验
		 Input:          index， 当前栅格序号， height_threshold 高度阀值，
						 ground 参考的地面特征， history_grid 历史栅格信息
		 Output:         无
		 Return:         无
		 Author:         hl
		 Date:           2021/1/18
		*********************************************/
		void checkGrid(int index,float height_threshold,
						Feature &ground,Grid &history_grid)
		{
			if ((*grid_list_)[index].check_flag_ == -1)
			{
				(*grid_list_)[index].check_flag_ = 1;
				if (
					((*grid_list_)[index].current_feature_.abs_height_ < height_threshold)
					&& ((*grid_list_)[index].current_feature_.avg_z_ < height_threshold)
					&& ((*grid_list_)[index].current_feature_.avg_z_ > -height_threshold)
					)
				{
					// 满足条件判断为地面点
					(*grid_list_)[index].check_flag_ = 0;
					(*grid_list_)[index].ground_flag_ = 0;
					CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
				}else
				{
					float _probability = calProbability((*grid_list_)[index].current_feature_.avg_z_,ground.avg_z_);
					if (_probability > 0.8)
					{
						(*grid_list_)[index].check_flag_ = 0;
						(*grid_list_)[index].ground_flag_ = 0;
						CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
					}else
					{
						if ((*grid_list_)[index].current_feature_.abs_height_ < height_threshold)
						{
							// 第一个条件要根据距离远近设置阀值
							if (((*grid_list_)[index].current_feature_.avg_z_ < ground.avg_z_ + height_threshold)
								|| ((*grid_list_)[index].current_feature_.avg_z_ < height_threshold))
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
							/*if ((*grid_list_)[index].current_feature_.abs_height_ < height_threshold)
							{
								if (((*grid_list_)[index].current_feature_.avg_z_ < ground.avg_z_ + height_threshold)
									|| ((*grid_list_)[index].current_feature_.avg_z_ < height_threshold))
								{
									(*grid_list_)[index].check_flag_ = 0;
									(*grid_list_)[index].ground_flag_ = 0;
									CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
								}
							}else*/
							{
								for (int i = 0 ; i < (*grid_list_)[index].current_feature_.point_size_ ; i++)
								{
									if (((*grid_list_)[index].grid_cloud_[i].z < ground.avg_z_ + 0.05)
										||((*grid_list_)[index].grid_cloud_[i].z < 0.05))
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


	public:
		/************************************************* 
		 Description:     栅格分割的主函数入口 
		 Input:          grid 需要进行二次栅格的栅格，
						 size  栅格尺寸
						 height_threshold 高度阀值
						 ground 参考地面特征
						 history_grid 历史栅格信息
		 Output:         out_indices  存储分割结果地面点云序号集
		 Return:          无 
		 Author:         hl
		 Date:           2021/1/19
		*********************************************/
		void segment(Grid &grid,float size,float height_threshold,
					Feature &ground,Grid &history_grid,
					pcl::PointIndices &out_indices)
		{
			if (grid.grid_cloud_.points.size() <= 0)
			{
				return ;
			}

			minx_ = grid.current_feature_.min_x_;
			maxx_ = grid.current_feature_.max_x_;

			miny_ = grid.current_feature_.min_y_;
			maxy_ = grid.current_feature_.max_y_;

			col_ = (maxx_ - minx_) / size + 1;
			row_ = (maxy_ - miny_) / size + 1;
			int _grid_size = col_ * row_;
			grid_list_->resize(_grid_size);
			for (int i = 0; i < _grid_size; ++i)
			{
				(*grid_list_)[i].is_vaild_ = false;
			}

			//栅格
			
			size_t _point_size = grid.grid_cloud_.points.size();
			for (size_t i = 0; i < _point_size ; ++ i)
			{
				int _col = (maxx_ - grid.grid_cloud_[i].x) / size;
				int _row = (maxy_ - grid.grid_cloud_[i].y) / size;
				int _index = _col * row_ + _row;
				if ((*grid_list_)[_index].is_vaild_ == false)
				{
					(*grid_list_)[_index].is_vaild_ = true;
				}
				(*grid_list_)[_index].addPoint(grid.grid_cloud_[i],grid.grid_inliers_.indices[i]);
			}


			//分割
			
			for (int i = 0; i < _grid_size; ++i)
			{
				if ((*grid_list_)[i].is_vaild_)
				{
					(*grid_list_)[i].distributeStatistics();
					checkGrid(i,height_threshold,ground,history_grid);
				}
			}
			for (int i = 0; i < _grid_size ; ++i)
			{
				

				if ((*grid_list_)[i].ground_flag_ == 0)
				{
					out_indices.indices.insert(out_indices.indices.end(),
						(*grid_list_)[i].grid_inliers_.indices.begin(),
						(*grid_list_)[i].grid_inliers_.indices.end());
					mergeFeature(grid.ground_feature_,(*grid_list_)[i].ground_feature_);
				}
				if ((*grid_list_)[i].ground_flag_ == 1)
				{
					//printf("i = %d\n",i);
						out_indices.indices.insert(out_indices.indices.end(),
						(*grid_list_)[i].grid_ground_inliers_.indices.begin(),
						(*grid_list_)[i].grid_ground_inliers_.indices.end());
					mergeFeature(grid.ground_feature_,(*grid_list_)[i].ground_feature_);
				}
			}
		};


		/************************************************* 
		 Description:    计算两个浮点数的相似度
		 Input:          float 
		 Output:         无
		 Return:         float  相似度， 范围(0 ~ 1)
		 Author:         hl
		 Date:           2021/1/18
		*********************************************/
		float calProbability(float first,float second)
		{
			float _probability = 1.0
			 / (1.0 + fabs(first - second)
			 / (fabs(first) + fabs(second)));
			return _probability;
		};


	public:
		float minx_,miny_,maxx_,maxy_;
		std::vector<GridIndex> grid_index_list_;
	public:
		boost::shared_ptr<std::vector<Grid>> grid_list_ = boost::make_shared<std::vector<Grid>>();
		int col_ = 0;
		int row_ = 0;	
};


}
#endif

