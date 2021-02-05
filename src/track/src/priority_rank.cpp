#include "priority_rank.h"

namespace skywell
{
    /**
 * @brief 
 * @param objs 传入参数，排序前的物体
 * @param obj_after_rank 传出参数，排序后的物体
 * @param level 排序等级
 */
    void Rank::priority_rank(boost::shared_ptr<std::vector<skywell::Object>> objs,
                             boost::shared_ptr<std::vector<skywell::Object>> obj_after_rank,
                             int level)
    {
        int objs_num = objs->size();
        boost::shared_ptr<std::vector<skywell::Object>> temp_objs = boost::make_shared<std::vector<skywell::Object>>();

        std::vector<int> index;
        int num = 0;
        switch (level)
        {
        case 0:
        {

            //获取每个障碍物的x/y/z坐标信息，并对每个障碍物加索引
            for (int i = 0; i < objs_num; i++)
            {
                //屏蔽较小的物体
                //不参与排序输出的物体，可行驶区域外面静止的物体，
                if ((*objs)[i].transform(2) < 0.1f || //去除地面上的障碍物
                    abs((*objs)[i].transform(1)) > 20 && (*objs)[i].width < 0.2f &&
                        (*objs)[i].height < 0.2f && (*objs)[i].length < 0.2f)
                    continue;

                if ((*objs)[i].transform(0) < 0)
                {
                    (*objs)[i].weights = ((*objs)[i].transform(0) / ratio_x2y) * ((*objs)[i].transform(0) / ratio_x2y) +
                                         (*objs)[i].transform(1) * (*objs)[i].transform(1);
                }
                else
                {
                    (*objs)[i].weights = ((*objs)[i].transform(0) / (2 * ratio_x2y)) * ((*objs)[i].transform(0) / (2 * ratio_x2y)) +
                                         (*objs)[i].transform(1) * (*objs)[i].transform(1);
                }
                temp_objs->push_back((*objs)[i]);
                num++;
                index.push_back(num);
            }
            break;
        }
        case 1:
        {
            //获取每个障碍物的x/y/z坐标信息，并对每个障碍物加索引
            for (int i = 0; i < objs_num; i++)
            {
                //屏蔽较小的物体
                if ((*objs)[i].transform(2) < 0.1 ||
                    (((*objs)[i].transform)(0) < 0) ||
                    (abs(((*objs)[i].transform)(1)) > 8 && (*objs)[i].width < 0.5 &&
                     (*objs)[i].height < 0.5 && (*objs)[i].length < 0.5))
                    continue;
                else
                {

                    (*objs)[i].weights = (((*objs)[i].transform)(0) / ratio_x2y) * (((*objs)[i].transform)(0) / ratio_x2y) +
                                         ((*objs)[i].transform)(1) * ((*objs)[i].transform)(1);
                    temp_objs->push_back((*objs)[i]);
                    num++;
                    index.push_back(num);
                }
            }
            break;
        }
        case 2:
        {
            //获取每个障碍物的x/y/z坐标信息，并对每个障碍物加索引
            for (int i = 0; i < objs_num; i++)
            {
                //屏蔽较小的物体
                //不参与排序输出的物体，可行驶区域外面静止的物体，
                if ((*objs)[i].transform(2) < 0.1f || //去除地面上的障碍物
                    abs((*objs)[i].transform(1)) > 8 && (*objs)[i].width < 0.5f &&
                        (*objs)[i].height < 0.5f && (*objs)[i].length < 0.5f ||
                    abs((*objs)[i].transform(1)) > 8 && (*objs)[i].speedx < 0.5f && abs((*objs)[i].speedy) < 0.1f)
                    continue;

                if ((*objs)[i].transform(0) < 0)
                {
                    (*objs)[i].weights = ((*objs)[i].transform(0) / ratio_x2y) * ((*objs)[i].transform(0) / ratio_x2y) +
                                         (*objs)[i].transform(1) * (*objs)[i].transform(1);
                }
                else
                {
                    (*objs)[i].weights = ((*objs)[i].transform(0) / ratio_x2y) * ((*objs)[i].transform(0) / ratio_x2y) +
                                         (*objs)[i].transform(1) * (*objs)[i].transform(1);
                }
                temp_objs->push_back((*objs)[i]);
                num++;
                index.push_back(num);
            }
            break;
        }
        default:
            break;
        }

        int tem_objs_num = temp_objs->size();

        for (int i = 0; i < tem_objs_num; i++)
        {
            for (int j = 0; j < tem_objs_num - i - 1; j++)
            {
                int temp_index = 0;
                if ((*temp_objs)[(index[j] - 1)].weights > (*temp_objs)[(index[j + 1] - 1)].weights)
                {
                    temp_index = index[j + 1];
                    index[j + 1] = index[j];
                    index[j] = temp_index;
                }
            }
        }
        //将排序后的索引对应的物体结构体入栈
        for (int i = 0; i < tem_objs_num; i++)
        {
            obj_after_rank->push_back((*temp_objs)[(index[i] - 1)]);
        }

        return;
    }

    void Rank::priority_rank_adv(boost::shared_ptr<std::vector<skywell::Object>> objs,
                                 boost::shared_ptr<std::vector<skywell::Object>> obj_after_rank,
                                 int level)
    {
        int objs_num = objs->size();
        std::vector<int> index;
        switch (level)
        {
        case 0:
        { //获取每个障碍物的x/y/z坐标信息，并对每个障碍物加索引
            for (int i = 0; i < objs_num; i++)
            {
                //屏蔽较小的物体
                //不参与排序输出的物体，可行驶区域外面静止的物体，
                if ((*objs)[i].transform(2) < 0.1f || //去除地面上的障碍物
                    abs((*objs)[i].transform(1)) > 8 && (*objs)[i].width < 0.5f &&
                        (*objs)[i].height < 0.5f && (*objs)[i].length < 0.5f ||
                    abs((*objs)[i].transform(1)) > 8 && (*objs)[i].speedx < 0.5f && abs((*objs)[i].speedy) < 0.1f)
                    continue;

                if ((*objs)[i].transform(0) < 0)
                {
                    (*objs)[i].weights = ((*objs)[i].transform(0) / ratio_x2y) * ((*objs)[i].transform(0) / ratio_x2y) +
                                         (*objs)[i].transform(1) * (*objs)[i].transform(1);
                }
                else
                {
                    (*objs)[i].weights = ((*objs)[i].transform(0) / ratio_x2y) * ((*objs)[i].transform(0) / ratio_x2y) +
                                         (*objs)[i].transform(1) * (*objs)[i].transform(1);
                }
                index.push_back(i); //记录原始障碍物的下标
            }
            break;
        }
        case 1:
        {
            //获取每个障碍物的x/y/z坐标信息，并对每个障碍物加索引
            for (int i = 0; i < objs_num; i++)
            {
                //屏蔽较小的物体
                if ((*objs)[i].transform(2) < 0.1 ||
                    (((*objs)[i].transform)(0) < 0) ||
                    (abs(((*objs)[i].transform)(1)) > 8 && (*objs)[i].width < 0.5 &&
                     (*objs)[i].height < 0.5 && (*objs)[i].length < 0.5))
                    continue;
                else
                {

                    (*objs)[i].weights = (((*objs)[i].transform)(0) / ratio_x2y) * (((*objs)[i].transform)(0) / ratio_x2y) +
                                         ((*objs)[i].transform)(1) * ((*objs)[i].transform)(1);

                    index.push_back(i);
                }
            }
            break;
        }
        default:
            break;
        }

        int new_objs_num = index.size();

        for (int i = 0; i < new_objs_num; i++)
        {
            for (int j = 0; j < new_objs_num - i - 1; j++)
            {
                int temp_index = 0;
                if ((*objs)[(index[j])].weights > (*objs)[(index[j + 1])].weights)
                {
                    temp_index = index[j + 1];
                    index[j + 1] = index[j];
                    index[j] = temp_index;
                }
            }
        }
        //将排序后的索引对应的物体结构体入栈
        for (int i = 0; i < new_objs_num; i++)
        {
            obj_after_rank->push_back((*objs)[(index[i])]);
        }

        return;
    }
    /**
 * @brief 
 * 
 * @param databuf 物体属性
 * @param num 物体数量
 * @param id 最优先级id
 * @param time 障碍物时间戳
 */
    void Rank::objects_header_fill_720(unsigned char *databuf, unsigned char num,
                                       unsigned short id, unsigned long long time)
    {
        int pos = 0;
        databuf[pos++] = num & 0xff;
        databuf[pos++] = 0x00;
        databuf[pos++] = (id >> 8) & 0xff;
        databuf[pos++] = id & 0xff;
        databuf[pos++] = (time >> 24) & 0xff;
        databuf[pos++] = (time >> 16) & 0xff;
        databuf[pos++] = (time >> 8) & 0xff;
        databuf[pos++] = time & 0xff;
    }
    /**
 * @brief 
 * 
 * @param databuf 发送缓冲区
 * @param cluster_objs 物体属性
 * @param objs_seq 第几个物体
 */
    void Rank::objects_send_fill_720(unsigned char *databuf,
                                     skywell::Object cluster_objs,
                                     int objs_seq)
    {
        int pos = 8 + objs_seq * 28;

        databuf[pos++] = (cluster_objs.number >> 8) & 0xff;
        databuf[pos++] = cluster_objs.number & 0xff;
        // printf("number =%d\n", cluster_objs.number);
        databuf[pos++] = cluster_objs.life & 0xff;
        // printf("life =%d\n", cluster_objs.life);
        databuf[pos++] = cluster_objs.type & 0xff;
        // printf("type =%d\n", cluster_objs.type);
        databuf[pos++] = cluster_objs.cut_in_out & 0xff;
        //  printf("cut_in_out =%d\n", cluster_objs.cut_in_out);
        databuf[pos++] = 0x00;
        databuf[pos++] = 0x00;
        databuf[pos++] = 0x00;

        int temp = (int)(cluster_objs.length * 100);
        databuf[pos++] = (temp >> 8) & 0xff;
        databuf[pos++] = (temp)&0xff;
        // printf("length =%d\n", temp);
        temp = (int)(cluster_objs.width * 100);
        databuf[pos++] = (temp >> 8) & 0xff;
        databuf[pos++] = (temp)&0xff;
        // printf("width =%d\n", temp);
        temp = (int)(cluster_objs.height * 100);
        databuf[pos++] = (temp >> 8) & 0xff;
        databuf[pos++] = (temp)&0xff;
        // printf("height =%d\n", temp);
        short temp_sh = (short)(cluster_objs.transform(0) * 100);
        databuf[pos++] = (temp_sh >> 8) & 0xff;
        databuf[pos++] = (temp_sh)&0xff;
        // prshortf("length=%d weith=%d\n", temp_sh);
        // printf("x =%d\n", temp_sh);
        temp_sh = (short)(cluster_objs.transform(1) * 100);
        // printf("(short)(cluster_objs.transform(1) * 100)=%d\n", (short)(cluster_objs.transform(1) * 100));
        // printf("(short)(cluster_objs.transform(1) * 100)=%02x\n", temp_sh);
        // printf("size(short)=%d\n", sizeof(short));
        databuf[pos++] = (temp_sh >> 8) & 0xff;
        databuf[pos++] = (temp_sh)&0xff;
        // printf("y =%d\n", temp_sh);
        temp_sh = (short)(cluster_objs.transform(2) * 100);
        databuf[pos++] = (temp_sh >> 8) & 0xff;
        databuf[pos++] = (temp_sh)&0xff;
        // printf("z =%d\n", temp_sh);
        temp_sh = (short)(cluster_objs.speedx * 100);
        databuf[pos++] = (temp_sh >> 8) & 0xff;
        databuf[pos++] = (temp_sh)&0xff;
        // printf("v_x =%d\n", temp_sh);
        temp_sh = (short)(cluster_objs.speedy * 100);
        databuf[pos++] = (temp_sh >> 8) & 0xff;
        databuf[pos++] = (temp_sh)&0xff;
        // printf("v_y =%d\n", temp_sh);
        temp_sh = (short)(cluster_objs.accelerationx * 100);
        databuf[pos++] = (temp_sh >> 8) & 0xff;
        databuf[pos++] = (temp_sh)&0xff;
        // printf("a_x =%d\n", temp_sh);
        temp_sh = (short)(cluster_objs.accelerationy * 100);
        databuf[pos++] = (temp_sh >> 8) & 0xff;
        databuf[pos++] = (temp_sh)&0xff;
        // printf("a_y =%d\n", temp_sh);
        //  printf("\n");
        // printf(" (int)(cluster_objs.accelerationy * 100)=%d\n", (int)(cluster_objs.accelerationy * 100));
        // printf(" (int)(cluster_objs.accelerationy * 100)=%02x\n", temp_sh);

        // printf("v_x=%f v_y=%f a_x=%f a_y=%f\n", (cluster_objs.speedx * 100), (cluster_objs.speedy * 100), (cluster_objs.accelerationx * 100), (cluster_objs.accelerationy * 100));
    }
    /**
 * @brief 
 * 
 * @param databuf 物体属性
 * @param num 物体数量
 * @param time 障碍物时间戳
 */
    void Rank::objects_header_fill_915(unsigned char *databuf, unsigned char num, unsigned long time)
    {
        int protocal_version = 0;
        //printf("timestamp =%ld\n", time);
        //printf("protocal_version =%ld\n", protocal_version);
        //printf("total num: =%d\n", num);
        int pos = 0;

        databuf[pos++] = (time >> 56) & 0xff;
        databuf[pos++] = (time >> 48) & 0xff;
        databuf[pos++] = (time >> 40) & 0xff;
        databuf[pos++] = (time >> 32) & 0xff;
        databuf[pos++] = (time >> 24) & 0xff;
        databuf[pos++] = (time >> 16) & 0xff;
        databuf[pos++] = (time >> 8) & 0xff;
        databuf[pos++] = time & 0xff;

        databuf[pos++] = 0x32;
        databuf[pos++] = 0x30;
        databuf[pos++] = 0x30;
        databuf[pos++] = 0x00;
        databuf[pos] = num & 0xff;
    }
    /**
 * @brief 
 * @param databuf 发送缓冲区
 * @param cluster_objs 物体属性
 * @param objs_seq 第几个物体
 */
    void Rank::objects_send_fill_915(unsigned char *databuf,
                                     skywell::Object cluster_objs,
                                     int objs_seq)
    {
        int pos = OBSTANCLE_HEADER_LEN + objs_seq * PER_OBSTANCLE_LEN;

        databuf[pos++] = (cluster_objs.number >> 8) & 0xff;
        databuf[pos++] = cluster_objs.number & 0xff;
       // printf("\n");
       // printf("number =%d\n", cluster_objs.number);
        databuf[pos++] = cluster_objs.life & 0xff;
       // printf("life =%d\n", cluster_objs.life);
        databuf[pos++] = cluster_objs.type & 0xff;
       // printf("type =%d\n", cluster_objs.type);

        float tmp_flt = Skywell::htonf(cluster_objs.transform(0));
        memcpy(&(databuf[pos]), &tmp_flt, 4);
       // printf("pos=%d,center X:%f\n", pos, cluster_objs.transform(0));
        pos += 4;

        tmp_flt = Skywell::htonf(cluster_objs.transform(1));
        memcpy(&(databuf[pos]), &tmp_flt, 4);
       // printf("pos=%d,center Y:%f\n", pos, cluster_objs.transform(1));
        pos += 4;

        tmp_flt = Skywell::htonf(cluster_objs.transform(2));
        memcpy(&(databuf[pos]), &tmp_flt, 4);
       // printf("center Z:%f\n", cluster_objs.transform(2));
        pos += 4;

        tmp_flt = Skywell::htonf(cluster_objs.length);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        //printf("length:%f\n", cluster_objs.length);
        pos += 4;

        tmp_flt = Skywell::htonf(cluster_objs.width);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
       // printf("width:%f\n", cluster_objs.width);
       // printf("length =%d\n", temp);
        tmp_flt = Skywell::htonf(cluster_objs.height);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("heigth:%f\n", cluster_objs.height);

        tmp_flt = Skywell::htonf(cluster_objs.speedx);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("speedx:%f\n", cluster_objs.speedx);

        tmp_flt = Skywell::htonf(cluster_objs.speedy);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
       // printf("speedy:%f\n", cluster_objs.speedy);

        tmp_flt = 0.0f;
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("speedz:%f\n", tmp_flt);

        tmp_flt = Skywell::htonf(cluster_objs.accelerationx);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("accelerationx:%f\n", cluster_objs.accelerationx);

        tmp_flt = Skywell::htonf(cluster_objs.accelerationy);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
       // printf("accelerationy:%f\n", cluster_objs.accelerationy);

        tmp_flt = 0.0f;
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("accelerationz:%f\n", tmp_flt);

        tmp_flt = 0.0f;
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("yaw speed:%f\n", tmp_flt);

        tmp_flt = 0.0f;
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        //printf("yaw acc:%f\n", tmp_flt);
        pos += 4;

        databuf[pos++] = cluster_objs.cut_in_out & 0xff;
        //printf("cut_in_out =%d\n", cluster_objs.cut_in_out);

        char temp_ch = 1;
        databuf[pos] = (temp_ch)&0xff;
        //printf("polygon type:%d\n", databuf[pos]);
        pos++;

        temp_ch = 4;
        databuf[pos] = (temp_ch)&0xff;
        //printf("point number:%d,pos=%d\n", databuf[pos], pos);
        pos++;

        tmp_flt = Skywell::htonf(cluster_objs.P[0].x);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("P[0].x:%f\n", cluster_objs.P[0].x);

        tmp_flt = Skywell::htonf(cluster_objs.P[0].y);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("P[0].y:%f\n", cluster_objs.P[0].y);

        tmp_flt = Skywell::htonf(cluster_objs.P[1].x);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("P[1].x:%f\n", cluster_objs.P[1].x);

        tmp_flt = Skywell::htonf(cluster_objs.P[1].y);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("P[1].y:%f\n", cluster_objs.P[1].y);

        tmp_flt = Skywell::htonf(cluster_objs.P[2].x);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("P[2].x:%f\n", cluster_objs.P[2].x);

        tmp_flt = Skywell::htonf(cluster_objs.P[2].y);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
       // printf("P[2].y:%f\n", cluster_objs.P[2].y);

        tmp_flt = Skywell::htonf(cluster_objs.P[3].x);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        pos += 4;
        //printf("P[3].x:%f\n", cluster_objs.P[3].x);

        tmp_flt = Skywell::htonf(cluster_objs.P[3].y);
        memcpy(&(databuf[pos]), &tmp_flt, 4);
        //printf("P[3].y:%f\n", cluster_objs.P[3].y);
        pos += 4;
        // printf("z =%d\n", temp_sh);

        // printf("a_y =%d\n", temp_sh);
        //  printf("\n");
        // printf(" (int)(cluster_objs.accelerationy * 100)=%d\n", (int)(cluster_objs.accelerationy * 100));
        // printf(" (int)(cluster_objs.accelerationy * 100)=%02x\n", temp_sh);

        // printf("v_x=%f v_y=%f a_x=%f a_y=%f\n", (cluster_objs.speedx * 100), (cluster_objs.speedy * 100), (cluster_objs.accelerationx * 100), (cluster_objs.accelerationy * 100));
    }
}; // namespace skywell
