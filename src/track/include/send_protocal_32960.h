/*****************************************************************************/
/** 
* \file       protocal_32960.h
* \author     yangkun
* \date       2020/05/08
* \version    V1.0
* \brief      �ļ�����
* \note       Copyright (c) 2000-2020  ���пƼ����޹�˾
* \remarks    �޸���־
******************************************************************************/
#ifndef _PROTOCAL_32960_H_
#define _PROTOCAL_32960_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// #include <signal.h>
// #include <fcntl.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>

namespace Skywell
{

#define CMD_LOGIN 0x20
#define CMD_LOGOUT 0x21
#define CMD_HEARTBEAT 0x22
#define CMD_VEHICLE_STATE_REPORT 0x23
#define CMD_GPS_STATE_REPORT 0x28
#define CMD_TASK 0x80
#define CMD_TASK_REPORT 0x81
#define CMD_PATH_PLANNING 0x82
#define CMD_TELECONTROL 0x83
#define CMD_TELECONTROL_REPORT 0x84
#define CMD_VEHICLE_AROUND_REPORT 0x85
#define CMD_WARNING_REPORT 0x90
#define CMD_OTA_UPDATE 0xA0

#define PKT_START_BYTE 0x23
#define PKT_FINISH_BYTE 0x24
#define PKT_CMD_SEGMENT_IDX 2
#define PKT_CMD_RESULT_SEGMENT_IDX 3
#define PKT_TIME_SEGMENT_IDX 4
#define PKT_ENCRYPT_SEGMENT_IDX 8
#define PKT_LEN_SEGMENT_IDX 9
#define PKT_DATA_SEGMENT_IDX 11
#define PKT_HEAD_LEN 11
#define PKT_TAIL_LEN 2

#define ENCRYPT_NOTHING 0x01
#define ENCRYPT_RSA 0x02
#define ENCRYPT_AES 0x03
#define ENCRYPT_ERR 0xFE
#define ENCRYPT_INVALID 0xFF

#define PKT_CMD_SENDNEEDREPLY 0xFE
#define PKT_CMD_SENDNONEEDREPLY 0xA0
#define PKT_CMD_RECV_OK 0x00
#define PKT_CMD_RECV_ERR 0x01

    enum
    {
        RTN_SUCCESS = 0,
        RTN_ERROR = -1,
        RTN_CONNECT_ERROR = -2,
        RTN_FDOPEN_ERROR = -3,
        RTN_CMD_ERROR = -4,
        RTN_OPEN_ERROR = -5,
        RTN_SOCKET_ERROR = -6,
        RTN_SEND_ERROR = -7,
        RTN_RECV_ERROR = -8,
        RTN_GET_RESP_ERROR = -9,
        RTN_MEMFLOW_ERROR = -10,
        RTN_MALLOC_ERROR = -11,
        RTN_INPUT_PARAM_ERROR = -12,
        RTN_PROCESS_ERROR = -13,
        RTN_BUFF_LOCK_ERROR = -14,
        RTN_DATA_ERROR = -15,
        RTN_COPY_ERROR = -100,
        RTN_ERROR_LOGIN = 250,
    };

    int set_frame_data(unsigned char ucCmd, unsigned char ucSendorReply, unsigned char *pDataBuf,
                       unsigned short uDataLen, unsigned char *pFrameBuf, unsigned short *iFrameBufLen);
    void test_read_one_frame(unsigned short num, unsigned char *send_databuf);
}; // namespace Skywell

#endif //_PROTOCAL_32960_H_
