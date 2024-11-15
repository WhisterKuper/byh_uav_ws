/** 
 * @brief 姿态解算
 * @author WeiXuan <2020302121154@whu.edu.cn
 * @file quaternion.h
 * @addtogroup quaternion
 * @signature: 热爱漫无边际，生活自有分寸
 */

#ifndef __QUATERNION_H_
#define __QUATERNION_H_

#include "byh_uav.h"

/** 
 * @author WeiXuan
 * @brief 平方根倒数
 * @param number
 * @returns 
 */
float InvSqrt(float number);

/** 
 * @author WeiXuan
 * @brief 四元数解算
 * @param gx
 * @param gy
 * @param gz
 * @param ax
 * @param ay
 * @param az
 * @returns 
 */
geometry_msgs::Quaternion Quaternion(float gx, float gy, float gz, float ax, float ay, float az, double time);

#endif


