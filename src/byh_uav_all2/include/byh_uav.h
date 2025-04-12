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


using namespace std;

#define DEBUG                       0

// 发送数据校验标志位
#define SEND_DATA_CHECK             1          
// 接收数据校验标志位
#define READ_DATA_CHECK             0   

#define BUFFER_SIZE					1024 * 6

// 丢失最大包数
#define MAX_LOST_COUNT             	10000  

// 圆周率
#define PI 				            3.1415926f 

// 与IMU陀螺仪设置的量程有关
#define GYROSCOPE_BMI088_RATIO      0.00106526443603169529841533860381f
#define GYROSCOPE_ADIS16470_RATIO   2.6631610900792382460383465095346e-8
#define GYROSCOPE_ADIS16465_RATIO   2.6631610900792382460383465095346e-8
#define GYROSCOPE_ICM42688_RATIO    0.00013315805450396191230191732547673f

// 与IMU加速度计设置的量程有关
#define ACCEl_BMI088_RATIO 	        9.80665 / 1365
#define ACCEl_ADIS16470_RATIO 	    9.80665 / 52428800.0
#define ACCEl_ADIS16465_RATIO 	    9.80665 / 52428800.0
#define ACCEl_ICM42688_RATIO 	    9.80665 / 8192

// 与IMU磁力计设置的量程有关
#define MAGNET_RM3100_RATIO 	    0.003f 
#define MAGNET_AK8975_RATIO 	    0.003f 

// 判断是否在范围内
#define IN_RANGE(x, mid, range) 	    	( (x >= mid-range) && (x <= mid+range) ) 

/* 通信端口 */
	// 帧头
	#define FRAME_HEADER1           0X7B      
	#define FRAME_HEADER2           0X55    
	// 帧尾
	#define FRAME_TAIL              0X7D
	// 数据类型
	#define TYPE_IMU        	    0X01 
	#define TYPE_MAGNET             0X02
	#define TYPE_GPS       		    0X03
	#define TYPE_CAMERA      		0X04
	#define TYPE_BAROMETER     		0X05
	#define TYPE_COMMAND     		0X06
	// 名称
	#define NAME_ADIS16470 		    0X01
	#define NAME_ICM42688 		    0X02
	#define NAME_ICM20689 		    0X03
	#define NAME_BMI088 			0X04
	#define NAME_RM3100 			0X05
	#define NAME_AK8975 			0X06
	#define NAME_SPL06 				0X07
	#define NAME_MS5611 			0X08
	#define NAME_ZEDF9P 			0X09
	#define NAME_D435I 				0X10
	#define NAME_OAK				0X11
	#define NAME_ADIS16465 			0X12
	#define NAME_BMP581 			0X13

	// 命令名称
	#define NAME_ACQUSITION			0X01
	#define NAME_CHANGE_FREQUENCE	0X02
	#define NAME_GET_COMMAND		0X03
	
	// 采集命令
	#define START					0X01
	#define STOP					0X02
	
	// IMU数据包
	struct IMU_Sensors
	{
		uint64_t accel_gps_time;
		uint64_t accel_mcu_time;
		uint64_t gyro_gps_time;
		uint64_t gyro_mcu_time;
		int32_t accel_data_x;
		int32_t accel_data_y;
		int32_t accel_data_z;
		int32_t gyro_data_x;
		int32_t gyro_data_y;
		int32_t gyro_data_z;
	};

	// 磁力计数据包
	struct Magnet_Sensors
	{
		uint64_t magnet_gps_time;
		uint64_t magnet_mcu_time;
		int32_t magnet_data_x;
		int32_t magnet_data_y;
		int32_t magnet_data_z;
	};

	// GPS数据包
	struct GPS_Sensors
	{
		uint64_t pps_gps_time;
		uint64_t pps_mcu_time;
		int64_t gps_error_time;
		int64_t gps_extra_error_time;
		uint16_t gps_scale;
		int32_t longitude;
		int32_t latitude;
		int32_t height;
		int32_t velocity_n;
		int32_t velocity_e;
		int32_t velocity_d;
	};
	
	// 相机数据包
	struct Camera_Sensors
	{
		uint64_t pulse_gps_time;
		uint64_t pulse_mcu_time;
	};

	// 气压计数据包
	struct Barometer_Sensors
	{
		uint64_t data_gps_time;
		uint64_t data_mcu_time;
		int32_t height;
	};

	// 接受数据包
	struct Receive
	{
		// Buffer
		uint8_t rx[1000];
		uint32_t sequence[2];
		IMU_Sensors imu;
		Magnet_Sensors magnet;
		GPS_Sensors gps;
		Camera_Sensors camera;
		Barometer_Sensors barometer;
	};

	// IMU 数据包
	struct IMU_Sensor_Data
	{
		uint8_t	frame_header1; 						// 帧头
		uint8_t	frame_header2; 						// 帧头
		uint8_t length[4];							// 帧长度
		uint8_t calib[2];							// 帧头校验

		uint8_t count1[4];							// 帧ID
		uint8_t count2[4];							// 帧ID
		uint8_t type;								// 数据包类型
		uint8_t name;								// IMU名称
		uint8_t number;								// IMU序号
		
		uint8_t accel_gps_time[8];				    // GPS时间信息（nus）
		uint8_t accel_mcu_time[8];				    // MCU时间信息（nus）
		uint8_t accel_data_x[4];					// 原始数据
		uint8_t accel_data_y[4];					// 原始数据
		uint8_t accel_data_z[4];					// 原始数据
		
		uint8_t gyro_gps_time[8];					// GPS时间信息（nus）
		uint8_t gyro_mcu_time[8];					// MCU时间信息（nus）
		uint8_t gyro_data_x[4];						// 原始数据
		uint8_t gyro_data_y[4];						// 原始数据
		uint8_t gyro_data_z[4];						// 原始数据
		
		uint8_t	crc_calib; 							// 校验码
		uint8_t	frame_tail; 						// 帧尾
	};	

	// 磁力计数据包
	struct Magnet_Sensor_Data
	{
		uint8_t	frame_header1; 						// 帧头
		uint8_t	frame_header2; 						// 帧头
		uint8_t length[4];							// 帧长度
		uint8_t calib[2];							// 帧头校验

		uint8_t count1[4];							// 帧ID
		uint8_t count2[4];							// 帧ID
		uint8_t type;								// 数据包类型
		uint8_t name;								// 磁力计名称
		uint8_t number;								// 磁力计序号
		
		uint8_t magnet_gps_time[8];				    // GPS时间信息（nus）
		uint8_t magnet_mcu_time[8];				    // MCU时间信息（nus）
		uint8_t magnet_data_x[4];					// 原始数据
		uint8_t magnet_data_y[4];					// 原始数据
		uint8_t magnet_data_z[4];					// 原始数据
		
		uint8_t	crc_calib; 							// 校验码
		uint8_t	frame_tail; 						// 帧尾
	};	
		
	// GPS数据包
	struct GPS_Sensor_Data
	{
		uint8_t	frame_header1; 						// 帧头
		uint8_t	frame_header2; 						// 帧头
		uint8_t length[4];							// 帧长度
		uint8_t calib[2];							// 帧头校验
		
		uint8_t count1[4];							// 帧ID
		uint8_t count2[4];							// 帧ID
		uint8_t type;								// 数据包类型
		uint8_t name;								// GPS名称
		uint8_t number;								// GPS序号
		
		uint8_t pps_gps_time[8];					// GPS时间信息（nus）
		uint8_t pps_mcu_time[8];					// MCU时间信息（nus）
		uint8_t gps_error_time[8];				    // 晶振误差信息（nus）
		uint8_t gps_extra_error_time[8];	        // 推算误差信息（nus）
		uint8_t gps_scale[2];						// 模型修正系数
		uint8_t valid;								// GPS有效指示
		uint8_t filter_en;							// 滤波器收敛指示
		uint8_t longitude[4];						// 经度（度）
		uint8_t latitude[4];						// 纬度（度）
		uint8_t height[4];							// 高于平均海平面的高度（mm）
		uint8_t velocity_n[4];			        	// 北向速度（mm/s）
		uint8_t velocity_e[4];				    	// 东向速度（mm/s）
		uint8_t velocity_d[4];				    	// 地向速度（mm/s）
		
		uint8_t	crc_calib; 							// 校验码
		uint8_t	frame_tail; 						// 帧尾
	};	

	// 相机数据包
	struct Camera_Sensor_Data
	{
		uint8_t	frame_header1; 						// 帧头
		uint8_t	frame_header2; 						// 帧头
		uint8_t length[4];							// 帧长度
		uint8_t calib[2];							// 帧头校验
		
		uint8_t count1[4];							// 帧ID
		uint8_t count2[4];							// 帧ID
		uint8_t type;								// 数据包类型
		uint8_t name;								// 相机名称
		uint8_t number;								// 相机序号
		
		uint8_t pulse_gps_time[8];					// GPS时间信息（nus）
		uint8_t pulse_mcu_time[8];					// MCU时间信息（nus）
		
		uint8_t	crc_calib; 							// 校验码
		uint8_t	frame_tail; 						// 帧尾
	};

	// 气压计数据包
	struct Barometer_Sensor_Data
	{
		uint8_t	frame_header1; 						// 帧头
		uint8_t	frame_header2; 						// 帧头
		uint8_t length[4];							// 帧长度
		uint8_t calib[2];							// 帧头校验
		
		uint8_t count1[4];							// 帧ID
		uint8_t count2[4];							// 帧ID
		uint8_t type;								// 数据包类型
		uint8_t name;								// 气压计名称
		uint8_t number;								// 气压计序号
		
		uint8_t data_gps_time[8];					// GPS时间信息（nus）
		uint8_t data_mcu_time[8];					// MCU时间信息（nus）
		uint8_t height[4];							// 高度信息

		uint8_t	crc_calib; 							// 校验码
		uint8_t	frame_tail; 						// 帧尾
	};

	// 命令数据包
	struct Command_Data
	{
		uint8_t	frame_header1; 						// 帧头
		uint8_t	frame_header2; 						// 帧头
		uint8_t length[4];							// 帧长度
		uint8_t calib[2];							// 帧头校验
		
		uint8_t count1[4];							// 帧ID
		uint8_t count2[4];							// 帧ID
		
		uint8_t type;								// 数据包类型
		uint8_t name;								// 名称
		uint8_t command;							// 命令

		uint8_t	crc_calib; 							// 校验码
		uint8_t	frame_tail; 						// 帧尾
	};

	// 频率数据包
	struct Frequence_Command_Data
	{
		uint8_t	frame_header1; 						// 帧头
		uint8_t	frame_header2; 						// 帧头
		uint8_t length[4];							// 帧长度
		uint8_t calib[2];							// 帧头校验
		
		uint8_t count1[4];							// 帧ID
		uint8_t count2[4];							// 帧ID
		
		uint8_t type;								// 数据包类型
		uint8_t name;								// 名称
		uint8_t channel[2];							// 通道号
		uint8_t frequence[2];						// 通道号

		uint8_t	crc_calib; 							// 校验码
		uint8_t	frame_tail; 						// 帧尾
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
/* 通信端口 */

/* 接受状态机 */
	enum State
	{
		rx_frame_header,
		rx_length,
		rx_calib,
		rx_data,
	};

	enum UseIMU
	{
		USING_ADIS16470,
		USING_ICM42688,
		USING_BMI088,
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

		/* 以太网配置 */
			int serverSocket, clientSocket;
			struct sockaddr_in serverAddr, clientAddr;
			socklen_t addrLen = sizeof(clientAddr);
			uint8_t buffer[1];
			// ip地址
			string eth_ip;
			// ip端口
			int eth_port;
		/* 以太网配置 */

		/* 串口配置 */
			// 声明串口对象 
			serial::Serial BYH_Serial;         
			string uart_port_name;
			// 串口通信波特率
			int serial_baud_rate;
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
        
        // 电源电压
        float Power_voltage; 

		// 数据
		Receive Receive_Data;

		// 状态机
		State state = rx_frame_header;

        // 电压话题发布者
		ros::Publisher Voltage_publisher; 

        // IMU数据发布者
        ros::Publisher BMI088_publisher;
        ros::Publisher ADIS16470_publisher;
        ros::Publisher ICM42688_publisher;
        ros::Publisher ADIS16465_publisher;

        // 磁力计数据发布者
        ros::Publisher AK8975_publisher;
        ros::Publisher RM3100_publisher;

        // 气压计数据发布者
        ros::Publisher MS5611_publisher;
        ros::Publisher BMP581_publisher;
        ros::Publisher SPL06_publisher;

        // GPS话题发布者
		ros::Publisher ZEDF9P_publisher; 

        // 相机触发频率话题发布者
		ros::Publisher D435i_publisher; 

        // 命令话题发布者
		ros::Publisher Command_publisher; 

        // 相机触发频率话题订阅者  
		ros::Subscriber trigger_subscriber;    
    
        /** 
         * @author WeiXuan
         * @brief 频率话题订阅回调函数
         * @param &cmd_frequence
         * @returns 
         */        
		void Cmd_Frequence_Callback(const byh_uav::uav_cmd_frequence::ConstPtr &cmd_frequence); 

        /** 
         * @author WeiXuan
         * @brief 读取载体速度、IMU、电源电压数据
         * @returns 
         */        
		bool Get_Sensor_Data( uint8_t sensor_data );

        /** 
         * @author WeiXuan
         * @brief CRC校验函数
         * @param Count_Number
         * @param mode
         * @param length
         * @returns 
         */        
        unsigned char Check_Sum(unsigned char count_number, unsigned char mode, uint8_t* buffer); 

		/** 
		 * @author WeiXuan
		 * @brief 命令发送
		 * @returns 
		 */
		void Command_Send( uint8_t command, uint8_t name_command );
};

#endif