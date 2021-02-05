
#ifndef __NTOH_ZHJ__
#define __NTOH_ZHJ__

#include <arpa/inet.h>
#ifdef HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif 
namespace Skywell
{
/*****************************************************************************/
/**
 * @file        ntoh_zhj.h
 * @author      ZhangHaijun
 * @date        2020-05-14
 * @version     1.0
 * @brief       大小端函数扩展
 * @copyright   Copyright (c) 2010-2030  金龙客车制造有限公司
 * @remarks     无
 ******************************************************************************/

typedef char sint8;
typedef unsigned char uint8;
typedef short sint16;
typedef unsigned short uint16;
typedef int sint32;
typedef unsigned int uint32;

typedef unsigned long uint8_least;
typedef unsigned long uint16_least;
typedef unsigned long uint32_least;

typedef signed long sint8_least;
typedef signed long sint16_least;
typedef signed long sint32_least;

typedef float float32;
typedef double float64;
typedef unsigned long long uint64;
// #include "Std_Types.h"

uint64 htonll(uint64 x);
uint64 ntohll(uint64 x);
float32 htonf(float32 x);
float32 ntohf(float32 x);
float64 htond(float64 x);
float64 ntohd(float64 x);

}  // namespace Skywell

#endif /* defined(__NTOH_ZHJ__) */