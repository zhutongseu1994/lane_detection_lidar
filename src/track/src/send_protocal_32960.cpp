

#include "send_protocal_32960.h"
#include "priority_rank.h"
namespace Skywell
{

    /*****************************************************************************/
    /** 
    * \author	   yangkun
    * \date 	   2020/05/08
    * \brief	   函数描述				将业务数据封为一帧32960标准数据报文
    * \param[in]   ucCmd			命令字标识
    * \param[in]   ucSendorReply	命令回复
    * \param[in]   pDataBuf			业务数据
    * \param[in]   uDataLen			业务数据长度
    * \param[out]  pFrameBuf		32960帧数据
    * \param[out]  iFrameBufLen		32960帧数据长度
    * \return	   0:成功 其他:失败
    * \remarks	   其他信息
    ******************************************************************************/
    int set_frame_data(unsigned char ucCmd, unsigned char ucSendorReply, unsigned char *pDataBuf,
                       unsigned short uDataLen, unsigned char *pFrameBuf, unsigned short *iFrameBufLen)
    {
        clock_t uMillSec = 0;
        unsigned short uFrameLen = 0;
        unsigned short uDataLenNetSequence = 0;
        int iRetVal = RTN_SUCCESS;

        if ((NULL == pDataBuf) || (NULL == pFrameBuf) || (NULL == iFrameBufLen))
        {
           /* //printf("Input Param Error! pDataBuf:%p pFrameBuf:%p iFrameBufLen:%u\n",
                   pDataBuf, pFrameBuf, iFrameBufLen);*/
            iRetVal = RTN_INPUT_PARAM_ERROR;
            return iRetVal;
        }

        pFrameBuf[0] = PKT_START_BYTE;
        pFrameBuf[1] = PKT_START_BYTE;
        pFrameBuf[PKT_CMD_SEGMENT_IDX] = ucCmd;
        pFrameBuf[PKT_CMD_RESULT_SEGMENT_IDX] = ucSendorReply;

        uMillSec = htonl(clock() % 1000);
        memcpy(&pFrameBuf[PKT_TIME_SEGMENT_IDX], &uMillSec, sizeof(uMillSec));
        pFrameBuf[PKT_ENCRYPT_SEGMENT_IDX] = ENCRYPT_NOTHING;

        uDataLenNetSequence = htons(uDataLen);
        memcpy(&pFrameBuf[PKT_LEN_SEGMENT_IDX], &uDataLenNetSequence, sizeof(uDataLenNetSequence));

        *iFrameBufLen = PKT_HEAD_LEN;

        memcpy(&pFrameBuf[PKT_DATA_SEGMENT_IDX], pDataBuf, uDataLen);
        *iFrameBufLen += uDataLen;

        pFrameBuf[*iFrameBufLen] = PKT_FINISH_BYTE;
        pFrameBuf[(*iFrameBufLen) + 1] = PKT_FINISH_BYTE;
        *iFrameBufLen += PKT_TAIL_LEN;

       // //printf("Set Frame Data.Cmd:%u SendRecv:%u DataLen:%u FrameLen:%u\n",
       //        ucCmd, ucSendorReply, uDataLen, *iFrameBufLen);

        return iRetVal;
    }
    void test_read_one_frame(unsigned short num, unsigned char *send_databuf)
   {
      int pos = 11;
      //printf("test one frame date\n");

      // //printf("test_command_11=%d\n", send_databuf[49]);
      // //printf("test_command_12=%d\n", send_databuf[50]);
      // //printf("test_command_13=%d\n", send_databuf[22]);
      //printf("test_num=%d\n", num);

      unsigned short temp_ush = 0;
      short temp_sh = 0;
      unsigned int temp_int = 0;
      unsigned long  temp_l = 0;
      float temp_f = 0;

      temp_l |= (unsigned long)send_databuf[pos++] << 56;
      temp_l |= (unsigned long)send_databuf[pos++] << 48;
      temp_l |= (unsigned long)send_databuf[pos++] << 40;
      temp_l |= (unsigned long)send_databuf[pos++] << 32;
      temp_l |= (unsigned long)send_databuf[pos++] << 24;
      temp_l |= (unsigned long)send_databuf[pos++] << 16;
      temp_l |= (unsigned long)send_databuf[pos++] << 8;
      temp_l |= (unsigned long)send_databuf[pos++];
      //printf("test_time=%ld\n", temp_l);
      pos = 19;
      temp_int |= send_databuf[pos++] << 24;
      temp_int |= send_databuf[pos++] << 16;
      temp_int |= send_databuf[pos++] << 8;
      temp_int |= send_databuf[pos++];
      //printf("test_version=%ld\n", temp_int);

      //printf("test_objnum=%d\n", send_databuf[pos]);

      for (size_t i = 0; i < num; i++)
      {
         pos = 11 + 13 + i * PER_OBSTANCLE_LEN;
         temp_ush = 0;
         temp_ush |= send_databuf[pos++] << 8;
         temp_ush |= send_databuf[pos++];
         //printf("test_number =%d\n", temp_ush);
         //printf("test_life=%d\n", send_databuf[pos++]);
         //printf("test_type=%d\n", send_databuf[pos++]);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         //printf("pos=%d,test_center X:%f\n", pos, temp_f);
         pos += 4;

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         //printf("pos=%d,test_center Y:%f\n", pos, temp_f);
         pos += 4;

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         //printf("test_center Z:%f\n", temp_f);
         pos += 4;

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_length:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_width:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_heigth:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_speedx:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_speedy:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_speedz:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_acc_x:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_acc_y:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_acc_z:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         //printf("test_yaw speed:%f\n", temp_f);
         pos += 4;

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         //printf("test_yaw acc:%f\n", temp_f);
         pos += 4;

         //printf("pos = %d,test_in_out=%d\n", pos, send_databuf[pos++]);
         //printf("pos = %d,test_polygon type:%d\n", pos, send_databuf[pos++]);
         //printf("pos = %d,test_point number:%d\n", pos, send_databuf[pos++]);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_P[0].x:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_P[0].y:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_P[1].x:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_P[1].y:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_P[2].x:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_P[2].y:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_P[3].x:%f\n", temp_f);

         memcpy(&temp_f, &send_databuf[pos], 4);
         temp_f = Skywell::ntohf(temp_f);
         pos += 4;
         //printf("test_P[3].y:%f\n", temp_f);

         //printf("\n");
      }

      //printf("\n");
   }

}; // namespace Skywell
