#pragma once
#ifndef _PRIORITY_RANK_H_
#define _PRIORITY_RANK_H_

#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>
#include "type.h"
#include "ntoh.h"

const int TCP_32960_HEADER_LEN = 11;
const int TCP_32960_TAIL_LEN = 2;
const int PER_POINT_LEN = 8;
const int PER_OBSTANCLE_LEN = 95;
const int OBSTANCLE_HEADER_LEN = 13;
const int COMMAND_FLAG = 0x88;
const int REPLY_FLAG = 0xfe;

namespace skywell
{
  typedef struct _t_vehicle_time_info
  {
    short uYear;
    short uMillSec;
    char ucMonth;
    char ucDay;
    char ucHour;
    char ucMin;
    char ucSec;
    char ucPad[3];
  } T_vehicle_time_info;
  
#define ratio_x2y (6.0f)
  class Rank
  {
  private:
    int rank_level_;

  public:
    Rank(int level = 0) : rank_level_(level){};
    ~Rank(){};
    inline int rank_level(void)
    {
      return rank_level_;
    }
    inline int set_rank_level(int level)
    {
      rank_level_ = level;
      return rank_level_;
    }
    /**
 * @brief 
 * 
 * @param objs 传入参数，排序前的物体
 * @param obj_after_rank 传出参数，排序后的物体
 * @param level 排序等级
 */
    void priority_rank(boost::shared_ptr<std::vector<skywell::Object>> objs,
                       boost::shared_ptr<std::vector<skywell::Object>> obj_after_rank,
                       int level);

    void priority_rank_adv(boost::shared_ptr<std::vector<skywell::Object>> objs,
                           boost::shared_ptr<std::vector<skywell::Object>> obj_after_rank,
                           int level);
    /**
 * @brief 
 * 
 * @param databuf 发送缓冲区
 * @param cluster_objs 物体属性
 * @param objs_seq 第几个物体
 */
    void objects_send_fill_720(unsigned char *databuf,
                               skywell::Object cluster_objs,
                               int objs_seq);

    /**
 * @brief 
 * 
 * @param databuf 物体属性
 * @param num 物体数量
 * @param id 最优先级id
 * @param time 障碍物时间戳
 */
    void objects_header_fill_720(unsigned char *databuf, unsigned char num,
                                 unsigned short id, unsigned long long time);

    /**
 * @brief 
 * 
 * @param databuf 发送缓冲区
 * @param cluster_objs 物体属性
 * @param objs_seq 第几个物体
 */
    void objects_send_fill_915(unsigned char *databuf,
                               skywell::Object cluster_objs,
                               int objs_seq);

    /**
 * @brief 
 * 
 * @param databuf 物体属性
 * @param num 物体数量
 * @param id 最优先级id
 * @param time 障碍物时间戳
 */
    void objects_header_fill_915(unsigned char *databuf, unsigned char num, unsigned long time);
  };
} // namespace skywell
#endif //_PRIORITY_RANK_H_
