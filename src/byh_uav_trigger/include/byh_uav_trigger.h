/** 
 * @brief byh_uav 
 * @author WeiXuan <2020302121154@whu.edu.cn
 * @file byh_uav.h
 * @addtogroup byh_uav
 * @signature: 热爱漫无边际，生活自有分寸
 */

#ifndef __ROBOT_TIVA_H_
#define __ROBOT_TIVA_H_

#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <cstdio>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <vector> 
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#include <iomanip>
#include <ctime> 
#include <sys/time.h>
#include <thread>

#include "math_queue.h"

#include <byh_uav/uav_imu.h>
#include <byh_uav/uav_magnet.h>
#include <byh_uav/uav_barometer.h>
#include <byh_uav/uav_gps.h>
#include <byh_uav/uav_frequence.h>
#include <byh_uav/uav_cmd_frequence.h>
#include <byh_uav/uav_command.h>
#include <byh_uav/uav_fpga_time.h>
#include <byh_uav/uav_pps_all.h>
#include <cmath> 


using namespace std;

#define DEBUG                       0

/* 通信端口 */
	// 帧头
	#define FRAME_HEADER1     	0XAA
	#define FRAME_HEADER2     	0X55   

	// 接受数据包
	struct Receive
	{
		// Buffer
		uint8_t rx[1000];
	};

	// 使用同一存储空间，长度8个字节，高位在前，低位在后
	union         
	{
		uint64_t U64;
		uint8_t  B8[8];	
	}M_UINT64;

	// 使用同一存储空间，长度4个字节，高位在前，低位在后
	union        
	{
		uint32_t U32;
		uint8_t  B4[4];	
	}M_UINT32;

	// 使用同一存储空间，长度2个字节，高位在前，低位在后
	union        
	{
		uint16_t U16;
		uint8_t  B2[2];	
	}M_UINT16;

	// 使用同一存储空间，长度8个字节，高位在前，低位在后
	union        
	{
		int64_t B64;
		uint8_t B8[8];	
	}M_INT64;   

	// 使用同一存储空间，长度4个字节，高位在前，低位在后
	union        
	{
		int32_t B32;
		uint8_t B4[4];	
	}M_INT32;   

	// 使用同一存储空间，长度2个字节，高位在前，低位在后
	union        
	{
		int16_t B16;
		uint8_t B2[2];	
	}M_INT16;  
	
	// 使用同一存储空间，长度8个字节，高位在前，低位在后
	union        
	{
		double B64;
		uint8_t B8[8];	
	}M_DOUBLE;   

	// 使用同一存储空间，长度8个字节，高位在前，低位在后
	union        
	{
		float B32;
		uint8_t B4[4];	
	}M_FLOAT;   
/* 通信端口 */

/* 接受状态机 */
	enum State
	{
		rx_frame_header,
		rx_data,
	};
/* 接受状态机 */

// 使用构造函数初始化数据和发布话题等
class robot
{
	public:
        /** 
         * @author WeiXuan
         * @brief 构造函数
         * @returns 
         */    
		robot();   

        /** 
         * @author WeiXuan
         * @brief 析构函数
         * @returns 
         */        
		~robot();
        
        /** 
         * @author WeiXuan
         * @brief 接受数据线程
         * @returns 
         */        
		void thread_receieve(int id);  

		/** 
         * @author WeiXuan
         * @brief 处理数据线程
         * @returns 
         */        
		void thread_process(int id);  

		 /** 
         * @author WeiXuan
         * @brief 控制核心函数
         * @returns 
         */        
		void Control();  

	private:
        // 创建ROS节点句柄
		ros::NodeHandle private_nh; 
		
		// 传输方式
		int use_way;
		// 是否使用线程
		bool use_thread;

		// 数据
		Receive Receive_Data;

		/* 以太网配置 */
			int serverSocket, clientSocket;
			struct sockaddr_in serverAddr, clientAddr;
			socklen_t addrLen = sizeof(clientAddr);
			uint8_t buffer[1];
			// ip地址
			string eth_ip;
			// ip端口
			int eth_port;
			// 话题名称
			string topic;
		/* 以太网配置 */

		/* 串口配置 */
			// 声明串口对象 
			serial::Serial BYH_Serial;         
			string ch348_port_name;
			// 串口通信波特率
			int ch348_baud_rate;
		/* 串口配置 */

		/* 线程配置 */
			std::thread mission_receieve;
			std::thread mission_process;
			SafeQueue<uint8_t> data_quene;
			// 控制线程运行的标志
			bool running;        
		/* 线程配置 */
		
        // 坐标系
        string frame_id; 

		// 状态机
		State state = rx_frame_header;

        // 相机触发频率话题发布者
		ros::Publisher time_publisher;

        // PPS 时间话题订阅者  
		ros::Subscriber pps_subscriber;  

		// 端口号
		int channel;  
    
        /** 
         * @author WeiXuan
         * @brief 频率话题订阅回调函数
         * @returns 
         */        
		void PPS_Callback(const byh_uav::uav_pps_all::ConstPtr &pps_time); 

        /** 
         * @author WeiXuan
         * @brief 读取数据
         * @returns 
         */        
		bool Get_Sensor_Data( uint8_t sensor_data );
};

#endif