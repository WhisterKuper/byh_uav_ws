/** 
 * @brief 姿态解算
 * @author WeiXuan <2020302121154@whu.edu.cn
 * @file quaternion.cpp
 * @addtogroup quaternion
 * @signature: 热爱漫无边际，生活自有分寸
 */
#include "quaternion.h"

// 解算实例 
extern sensor_msgs::Imu Imu_Sensor;

volatile float twoKp = 1.0f;     
volatile float twoKi = 0.0f;     
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;          
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; 

/** 
 * @author WeiXuan
 * @brief 平方根倒数
 * @param number
 * @returns 
 */
float InvSqrt(float number)
{
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;
  x = number * 0.5F;
  y = number;
  i = * (( long * ) &y);
  i = 0x5f375a86 - ( i >> 1 );
  y = * (( float * ) &i);
  y = y * ( f - ( x * y * y ) );

  return y;
}

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
geometry_msgs::Quaternion Quaternion(float gx, float gy, float gz, float ax, float ay, float az, double time)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
  geometry_msgs::Quaternion result;
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
  {
    // 把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;      
    // 把四元数换算成方向余弦中的第三行的三个元素
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;
    // 误差是估计的重力方向和测量的重力方向的交叉乘积之和
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    // 计算并应用积分反馈（如果启用）
    if(twoKi > 0.0f) 
    {
      integralFBx += twoKi * halfex * time;  
      integralFBy += twoKi * halfey * time;
      integralFBz += twoKi * halfez * time;
      gx += integralFBx;        
      gy += integralFBy;
      gz += integralFBz;
    }
    else 
    {
      // 四元数归一化 
      recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
      q0 *= recipNorm;
      q1 *= recipNorm;
      q2 *= recipNorm;
      q3 *= recipNorm;
      result.w = q0;
      result.x = q1;
      result.y = q2;
      result.z = q3;    
      gx += twoKp * halfex;
      gy += twoKp * halfey;
      gz += twoKp * halfez;
    }
    // 龙格库塔积分
    gx *= (0.5f * time);   
    gy *= (0.5f * time);
    gz *= (0.5f * time);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 
    // 四元数归一化 
    recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    result.w = q0;
    result.x = q1;
    result.y = q2;
    result.z = q3;    
  }
  return result;
}
