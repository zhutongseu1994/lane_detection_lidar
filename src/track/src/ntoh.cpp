

#include <arpa/inet.h>
#include <string.h>
#include "ntoh.h"
namespace Skywell
{
/*****************************************************************************/
/**
 * @file        ntoh_zhj.c
 * @author      ZhangHaijun
 * @date        2020-05-14
 * @version     1.0
 * @brief       大小端函数扩展
 * @copyright   Copyright (c) 2010-2030  金龙客车制造有限公司
 * @remarks     无
 ******************************************************************************/

// #include "Std_Types.h"

/*****************************************************************************/
/**
 * @brief       检测当前系统大小端
 *
 * @return      int
 * @remarks     无
 ******************************************************************************/
static inline sint32 isBigEndian(void)  //测试主机大小端
{
  union
  {
    sint32 i;
    sint8 c[4];
  } u;
  u.i = 1;
  return (u.c[0] == 0);
}

/*****************************************************************************/
/**
 * @brief       64为长整形高低字节互换
 *
 * @param[in]   x
 * @return      uint64
 * @remarks     无
 ******************************************************************************/
uint64 swab64(uint64 x)
{
  uint8* pData = (uint8*)&x;
  uint8 tmp;

  tmp = pData[0];
  pData[0] = pData[7];
  pData[7] = tmp;
  tmp = pData[1];
  pData[1] = pData[6];
  pData[6] = tmp;
  tmp = pData[2];
  pData[2] = pData[5];
  pData[5] = tmp;
  tmp = pData[3];
  pData[3] = pData[4];
  pData[4] = tmp;

  return x;
}

/*****************************************************************************/
/**
 * @brief       长整形转网络字节序
 *
 * @param[in]   x
 * @return      uint64
 * @remarks     无
 ******************************************************************************/
uint64 htonll(uint64 x)
{
  if (isBigEndian())
  {
    return x;
  }
  else
  {
    return swab64(x);
  }
}

/*****************************************************************************/
/**
 * @brief       长整形转主机字节序
 *
 * @param[in]   x
 * @return      uint64
 * @remarks     无
 ******************************************************************************/
uint64 ntohll(uint64 x)
{
  htonll(x);
}

/*****************************************************************************/
/**
 * @brief       浮点转网络字节序
 *
 * @param[in]   x
 * @return      float32
 * @remarks     无
 ******************************************************************************/
float32 htonf(float32 x)
{
  uint32* pData32;
  uint32 data;

  pData32 = (uint32*)&x;
  data = htonl(*pData32);
  memcpy(&x, &data, 4);
  return x;
}

/*****************************************************************************/
/**
 * @brief       浮点转主机字节序
 *
 * @param[in]   x
 * @return      float32
 * @remarks     无
 ******************************************************************************/
float32 ntohf(float32 x)
{
  htonf(x);
}

/*****************************************************************************/
/**
 * @brief       双精度型转网络字节序
 *
 * @param[in]   x
 * @return      float64
 * @remarks     无
 ******************************************************************************/
float64 htond(float64 x)
{
  uint64* pData64;
  uint64 data;

  pData64 = (uint64*)&x;
  data = htonll(*pData64);
  memcpy(&x, &data, 8);
  return x;
}

/*****************************************************************************/
/**
 * @brief       双精度型转主机字节序
 *
 * @param[in]   x
 * @return      float64
 * @remarks     无
 ******************************************************************************/
float64 ntohd(float64 x)
{
  htond(x);
}
}  // namespace Skywell