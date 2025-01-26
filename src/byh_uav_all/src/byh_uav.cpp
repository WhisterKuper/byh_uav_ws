#include "byh_uav.h"
#include "quaternion.h"

byh_uav::uav_imu BMI088;
byh_uav::uav_imu ADIS16470;
byh_uav::uav_imu ICM42688;
byh_uav::uav_magnet RM3100;
byh_uav::uav_magnet AK8975;
byh_uav::uav_barometer MS5611;
byh_uav::uav_gps ZEDF9P;
byh_uav::uav_frequence D435i;
byh_uav::uav_command Command;

bool first = true;

/** 
 * @author WeiXuan
 * @brief 主函数
 * @param argc
 * @param argv
 * @returns 
 */
int main(int argc, char** argv)
{
    // 设置中文字符
    setlocale(LC_ALL, "");

    // ROS初始化 并设置节点名称 
    ros::init(argc, argv, "byh_uav"); 

    // 实例化一个对象
    robot byhuav;
    ROS_INFO_STREAM("Welcome to BYH_UAV!");
    
    // 循环执行数据采集和发布话题等操作
    byhuav.Control(); 

    return 0;  
} 

/** 
 * @author WeiXuan
 * @brief 循环获取下位机数据与发布话题
 * @returns 
 */
void robot::Control()
{
    // 使用线程
    if( use_thread == true)
    {
        while(ros::ok())
        {
            ros::spinOnce();
        }
    }
    // 使用以太网非线程
    else if( use_way == 0 )
    {
        // 接受端口数据
        if ((clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &addrLen)) < 0)
        { 
            perror("Accepting error");
        }
        while(ros::ok())
        {
            uint64_t bytesRead = recv(clientSocket, buffer, 1, 0);
            if (bytesRead < 0)
            {
                perror("Receiving error");
                break;
                
            }
            else if (bytesRead == 0)
            {
                break;
            }
            else
            {
                std::vector<char> bufferData(buffer, buffer + bytesRead);
                if (bufferData.size() > 0)
                {
                    for(uint64_t i = 0; i < bufferData.size(); ++i)
                    {
                        Get_Sensor_Data( bufferData[i] );
                    }
                }
            }
            ros::spinOnce();
        }
    }
    // 使用串口非线程
    else
    {
        while(ros::ok())
        {
            // 下位机数据
            uint8_t data[1];
            // 读取数据
            BYH_Serial.read(data,1); 
            Get_Sensor_Data( data[0] );
            ros::spinOnce();
        }
    }
}

/** 
 * @author WeiXuan
 * @brief 串口通讯校验函数
 * @param Count_Number 数据包前几个字节加入校验
 * @param mode 对发送数据还是接收数据进行校验
 * @returns 
 */
unsigned char robot::Check_Sum(unsigned char Count_Number, unsigned char mode, uint8_t* buffer)
{
    unsigned char check_sum=0,k;
    // 接收数据模式
    if(mode==0) 
    {
        for(k=0;k<Count_Number;k++)
        {
            // 按位异或
            check_sum=check_sum^buffer[k]; 
        }
    }
    // 发送数据模式
    if(mode==1) 
    {
        for(k=0;k<Count_Number;k++)
        {
            // 按位异或
            check_sum=check_sum^buffer[k]; 
        }
    }
    return check_sum; 
}

/** 
 * @author WeiXuan
 * @brief 命令发送
 * @returns 
 */
void robot::Command_Send( uint8_t command, uint8_t name_command )
{
    static uint32_t Command_count_H;
    static uint32_t Command_count_L;

    Command_Data Send_buffer;
    uint8_t* buffer = (uint8_t*) &Send_buffer;

    // 帧头
    Send_buffer.frame_header1 = FRAME_HEADER1; 
    Send_buffer.frame_header2 = FRAME_HEADER2; 

    // 长度
    M_UINT32.U32 = sizeof(Command_Data);
    Send_buffer.length[0] = M_UINT32.B4[3];
    Send_buffer.length[1] = M_UINT32.B4[2];
    Send_buffer.length[2] = M_UINT32.B4[1];
    Send_buffer.length[3] = M_UINT32.B4[0];

    // 帧头校验
    Send_buffer.calib[0] = Check_Sum(6, 1, buffer);
    Send_buffer.calib[1] = Check_Sum(7, 1, buffer);

    // 低位计数
    M_UINT32.U32 = Command_count_L;
    Send_buffer.count1[0] = M_UINT32.B4[3];
    Send_buffer.count1[1] = M_UINT32.B4[2];
    Send_buffer.count1[2] = M_UINT32.B4[1];
    Send_buffer.count1[3] = M_UINT32.B4[0];

    // 高位计数
    M_UINT32.U32 = Command_count_H;
    Send_buffer.count2[0] = M_UINT32.B4[3];
    Send_buffer.count2[1] = M_UINT32.B4[2];
    Send_buffer.count2[2] = M_UINT32.B4[1];
    Send_buffer.count2[3] = M_UINT32.B4[0];

    // 数据类型
    Send_buffer.type = TYPE_COMMAND;
    Send_buffer.name = name_command;
    Send_buffer.command = command;

    // 校验
    Send_buffer.crc_calib = Check_Sum(sizeof(Send_buffer)-2, 1, buffer);
    Send_buffer.frame_tail = FRAME_TAIL;

    if( use_way == 0 )
    {

    }
    else
    {
        try
        {
            // 通过串口向下位机发送数据 
            BYH_Serial.write( buffer, sizeof(Send_buffer) ); 

            // 收到命令
            if( Send_buffer.name == NAME_GET_COMMAND)
            {
                if( command == START )
                    ROS_INFO_STREAM("Get Start Command");
                else if( command == STOP )
                    ROS_INFO_STREAM("Get Stop Command");
            }
            
            // 计数
            if( Command_count_L == 4294967295)
            {
                Command_count_L = 0;
                Command_count_H++;
            }
            else
                Command_count_L++;
        }
        catch (serial::IOException& e)   
        {
            ROS_ERROR_STREAM("Unable to send data through serial port");
        }
    }
}

/** 
 * @author WeiXuan
 * @brief 读取并逐帧校验下位机发送过来的数据
 * @returns 
 */
bool robot::Get_Sensor_Data( uint8_t sensor_data )
{
    // 下位机数据
    uint8_t data[1];
    data[0] = sensor_data;
    
    // 静态变量，用于计数
    static uint32_t count;
    static uint32_t length;

    // 结构体
    IMU_Sensor_Data* data_imu;
    Magnet_Sensor_Data* data_magnet;
    GPS_Sensor_Data* data_gps;
    Camera_Sensor_Data* data_camera;
    Barometer_Sensor_Data* data_barometer;
    Command_Data* data_command;

    // 判断状态机
    switch(state)
    {
        case rx_frame_header:
        {
            if( (data[0] == FRAME_HEADER1) && (count == 0) )
            {
                if( count == 0)
                {
                    Receive_Data.rx[count] = data[0];
                    count++;
                }
                else 
                {
                    ROS_ERROR("State:count!=0");
                    count = 0;
                }
            }
            else if( (data[0] == FRAME_HEADER2) && (count == 1) )
            {
                if(count == 1)
                {
                    Receive_Data.rx[count] = data[0];
                    count++;
                    state = rx_length;
                }
                else 
                {
                    ROS_ERROR("State:count!=1");
                    count = 0;
                }
            }
            else
            {
                count=0;
            }
            break;
        }   

        case rx_length:
        {
            if( count<6 )
            {
                Receive_Data.rx[count] = data[0];
                count++;
            }
            else 
            {
                Receive_Data.rx[count] = data[0];
                count++;
                // 记录接受包长度
                M_UINT32.B4[3] = Receive_Data.rx[2];
                M_UINT32.B4[2] = Receive_Data.rx[3];
                M_UINT32.B4[1] = Receive_Data.rx[4];
                M_UINT32.B4[0] = Receive_Data.rx[5];
                length = M_UINT32.U32;

                // 接受错误
                if(length>1000)
                {
                    length = 0;
                    state = rx_frame_header;
                    ROS_ERROR("State:rx_length");
                    count = 0;
                }
                else
                    state = rx_calib;
            }
            break;
        }   

        case rx_calib:
        {
            if( count<8 )
            {
                Receive_Data.rx[count] = data[0];
                count++;
            }
            else 
            {
                Receive_Data.rx[count] = data[0];
                // 解析帧头
                if( (Receive_Data.rx[6] == Check_Sum(6, READ_DATA_CHECK, Receive_Data.rx)) && (Receive_Data.rx[7] == Check_Sum(7, READ_DATA_CHECK, Receive_Data.rx)))
                {
                    count++;
                    state = rx_data;
                }
                else 
                {
                    count=0;
                    ROS_ERROR("State:rx_calib");
                    state = rx_frame_header;
                }
            }
            break;
        }   

        case rx_data:
        {
            if( count<length-1 )
            {
                Receive_Data.rx[count] = data[0];
                count++;
            }
            else 
            {
                Receive_Data.rx[count] = data[0];
                ROS_DEBUG("state: %d, length: %d, count: %d, data: %x", state, length, count, data[0]);
                state = rx_frame_header;
                count = 0;
                
                // 帧尾以及校验正确
                if( (Receive_Data.rx[length-1] == FRAME_TAIL) && (Receive_Data.rx[length-2] == Check_Sum(length-2, READ_DATA_CHECK, Receive_Data.rx)))
                {
                    // IMU 数据包
                    if(Receive_Data.rx[16] == TYPE_IMU)
                    {
                        data_imu = (IMU_Sensor_Data*) Receive_Data.rx;
                        // 序列号
                        M_UINT32.B4[3] = data_imu->count1[0];
                        M_UINT32.B4[2] = data_imu->count1[1];
                        M_UINT32.B4[1] = data_imu->count1[2];
                        M_UINT32.B4[0] = data_imu->count1[3];
                        Receive_Data.sequence[0] = M_UINT32.U32;
                        M_UINT32.B4[3] = data_imu->count2[0];
                        M_UINT32.B4[2] = data_imu->count2[1];
                        M_UINT32.B4[1] = data_imu->count2[2];
                        M_UINT32.B4[0] = data_imu->count2[3];
                        Receive_Data.sequence[1] = M_UINT32.U32;

                        // 时间
                        M_UINT64.B8[7] = data_imu->accel_gps_time[0];
                        M_UINT64.B8[6] = data_imu->accel_gps_time[1];
                        M_UINT64.B8[5] = data_imu->accel_gps_time[2];
                        M_UINT64.B8[4] = data_imu->accel_gps_time[3];
                        M_UINT64.B8[3] = data_imu->accel_gps_time[4];
                        M_UINT64.B8[2] = data_imu->accel_gps_time[5];
                        M_UINT64.B8[1] = data_imu->accel_gps_time[6];
                        M_UINT64.B8[0] = data_imu->accel_gps_time[7];
                        Receive_Data.imu.accel_gps_time = M_UINT64.U64;
                        
                        M_UINT64.B8[7] = data_imu->accel_mcu_time[0];
                        M_UINT64.B8[6] = data_imu->accel_mcu_time[1];
                        M_UINT64.B8[5] = data_imu->accel_mcu_time[2];
                        M_UINT64.B8[4] = data_imu->accel_mcu_time[3];
                        M_UINT64.B8[3] = data_imu->accel_mcu_time[4];
                        M_UINT64.B8[2] = data_imu->accel_mcu_time[5];
                        M_UINT64.B8[1] = data_imu->accel_mcu_time[6];
                        M_UINT64.B8[0] = data_imu->accel_mcu_time[7];
                        Receive_Data.imu.accel_mcu_time = M_UINT64.U64;
                        
                        M_UINT64.B8[7] = data_imu->gyro_gps_time[0];
                        M_UINT64.B8[6] = data_imu->gyro_gps_time[1];
                        M_UINT64.B8[5] = data_imu->gyro_gps_time[2];
                        M_UINT64.B8[4] = data_imu->gyro_gps_time[3];
                        M_UINT64.B8[3] = data_imu->gyro_gps_time[4];
                        M_UINT64.B8[2] = data_imu->gyro_gps_time[5];
                        M_UINT64.B8[1] = data_imu->gyro_gps_time[6];
                        M_UINT64.B8[0] = data_imu->gyro_gps_time[7];
                        Receive_Data.imu.gyro_gps_time = M_UINT64.U64;
                        
                        M_UINT64.B8[7] = data_imu->gyro_mcu_time[0];
                        M_UINT64.B8[6] = data_imu->gyro_mcu_time[1];
                        M_UINT64.B8[5] = data_imu->gyro_mcu_time[2];
                        M_UINT64.B8[4] = data_imu->gyro_mcu_time[3];
                        M_UINT64.B8[3] = data_imu->gyro_mcu_time[4];
                        M_UINT64.B8[2] = data_imu->gyro_mcu_time[5];
                        M_UINT64.B8[1] = data_imu->gyro_mcu_time[6];
                        M_UINT64.B8[0] = data_imu->gyro_mcu_time[7];
                        Receive_Data.imu.gyro_mcu_time = M_UINT64.U64;
                        
                        // 数据
                        M_INT32.B4[3] = data_imu->accel_data_x[0];
                        M_INT32.B4[2] = data_imu->accel_data_x[1];
                        M_INT32.B4[1] = data_imu->accel_data_x[2];
                        M_INT32.B4[0] = data_imu->accel_data_x[3];
                        Receive_Data.imu.accel_data_x = M_INT32.B32;
                        M_INT32.B4[3] = data_imu->accel_data_y[0];
                        M_INT32.B4[2] = data_imu->accel_data_y[1];
                        M_INT32.B4[1] = data_imu->accel_data_y[2];
                        M_INT32.B4[0] = data_imu->accel_data_y[3];
                        Receive_Data.imu.accel_data_y = M_INT32.B32;
                        M_INT32.B4[3] = data_imu->accel_data_z[0];
                        M_INT32.B4[2] = data_imu->accel_data_z[1];
                        M_INT32.B4[1] = data_imu->accel_data_z[2];
                        M_INT32.B4[0] = data_imu->accel_data_z[3];
                        Receive_Data.imu.accel_data_z = M_INT32.B32;
    
                        M_INT32.B4[3] = data_imu->gyro_data_x[0];
                        M_INT32.B4[2] = data_imu->gyro_data_x[1];
                        M_INT32.B4[1] = data_imu->gyro_data_x[2];
                        M_INT32.B4[0] = data_imu->gyro_data_x[3];
                        Receive_Data.imu.gyro_data_x = M_INT32.B32;
                        M_INT32.B4[3] = data_imu->gyro_data_y[0];
                        M_INT32.B4[2] = data_imu->gyro_data_y[1];
                        M_INT32.B4[1] = data_imu->gyro_data_y[2];
                        M_INT32.B4[0] = data_imu->gyro_data_y[3];
                        Receive_Data.imu.gyro_data_y = M_INT32.B32;
                        M_INT32.B4[3] = data_imu->gyro_data_z[0];
                        M_INT32.B4[2] = data_imu->gyro_data_z[1];
                        M_INT32.B4[1] = data_imu->gyro_data_z[2];
                        M_INT32.B4[0] = data_imu->gyro_data_z[3];
                        Receive_Data.imu.gyro_data_z = M_INT32.B32;
                                                    
                        // ADIS16470
                        if( data_imu->name == NAME_ADIS16470 )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)ADIS16470.count && ADIS16470.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)ADIS16470.count + 1 && ADIS16470.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)ADIS16470.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }
                                if(first == false)
                                    ROS_WARN("[Lost_Count] ADIS16470: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - ADIS16470.count - 1, ADIS16470.count);

                                if(first == true)
                                    first = false;
                            }
                            
                            ADIS16470.sample_time = Receive_Data.imu.gyro_mcu_time/10000000000 + Receive_Data.imu.gyro_mcu_time%10000000000*0.0000000001 - ADIS16470.gyro_mcu_time;
                            ADIS16470.name = "ADIS16470";
                            ADIS16470.header.stamp = ros::Time::now(); 
                            ADIS16470.header.frame_id = frame_id; 
                            ADIS16470.number = data_imu->number;
                            ADIS16470.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            ADIS16470.accel_gps_time = Receive_Data.imu.accel_gps_time/10000000000 + Receive_Data.imu.accel_gps_time%10000000000*0.0000000001;
                            ADIS16470.accel_mcu_time = Receive_Data.imu.accel_mcu_time/10000000000 + Receive_Data.imu.accel_mcu_time%10000000000*0.0000000001;
                            ADIS16470.gyro_gps_time = Receive_Data.imu.gyro_gps_time/10000000000 + Receive_Data.imu.gyro_gps_time%10000000000*0.0000000001;
                            ADIS16470.gyro_mcu_time = Receive_Data.imu.gyro_mcu_time/10000000000 + Receive_Data.imu.gyro_mcu_time%10000000000*0.0000000001;
                            ADIS16470.linear_acceleration.x = Receive_Data.imu.accel_data_x * ACCEl_ADIS16470_RATIO;
                            ADIS16470.linear_acceleration.y = Receive_Data.imu.accel_data_y * ACCEl_ADIS16470_RATIO;
                            ADIS16470.linear_acceleration.z = Receive_Data.imu.accel_data_z * ACCEl_ADIS16470_RATIO;
                            ADIS16470.angular_velocity.x = Receive_Data.imu.gyro_data_x * GYROSCOPE_ADIS16470_RATIO;
                            ADIS16470.angular_velocity.y = Receive_Data.imu.gyro_data_y * GYROSCOPE_ADIS16470_RATIO;
                            ADIS16470.angular_velocity.z = Receive_Data.imu.gyro_data_z * GYROSCOPE_ADIS16470_RATIO;
                            ADIS16470.orientation = Quaternion(ADIS16470.angular_velocity.x, ADIS16470.angular_velocity.y, ADIS16470.angular_velocity.z,
                                       ADIS16470.linear_acceleration.x, ADIS16470.linear_acceleration.y, ADIS16470.linear_acceleration.z, 
                                       ADIS16470.sample_time);
                            ADIS16470_publisher.publish(ADIS16470);
                        }
                        // ICM42688
                        else if( data_imu->name == NAME_ICM42688 )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)ICM42688.count && ICM42688.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)ICM42688.count + 1 && ICM42688.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)ICM42688.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }

                                if(first == false)
                                    ROS_WARN("[Lost_Count] ICM42688: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - ICM42688.count - 1, ICM42688.count);

                                if(first == true)
                                    first = false;
                            }

                            ICM42688.sample_time = Receive_Data.imu.gyro_mcu_time/10000000000 + Receive_Data.imu.gyro_mcu_time%10000000000*0.0000000001 - ICM42688.gyro_mcu_time;
                            ICM42688.name = "ICM42688";
                            ICM42688.header.stamp = ros::Time::now(); 
                            ICM42688.header.frame_id = frame_id; 
                            ICM42688.number = data_imu->number;
                            ICM42688.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            ICM42688.accel_gps_time = Receive_Data.imu.accel_gps_time/10000000000 + Receive_Data.imu.accel_gps_time%10000000000*0.0000000001;
                            ICM42688.accel_mcu_time = Receive_Data.imu.accel_mcu_time/10000000000 + Receive_Data.imu.accel_mcu_time%10000000000*0.0000000001;
                            ICM42688.gyro_gps_time = Receive_Data.imu.gyro_gps_time/10000000000 + Receive_Data.imu.gyro_gps_time%10000000000*0.0000000001;
                            ICM42688.gyro_mcu_time = Receive_Data.imu.gyro_mcu_time/10000000000 + Receive_Data.imu.gyro_mcu_time%10000000000*0.0000000001;
                            ICM42688.linear_acceleration.x = Receive_Data.imu.accel_data_x * ACCEl_ICM42688_RATIO;
                            ICM42688.linear_acceleration.y = Receive_Data.imu.accel_data_y * ACCEl_ICM42688_RATIO;
                            ICM42688.linear_acceleration.z = Receive_Data.imu.accel_data_z * ACCEl_ICM42688_RATIO;
                            ICM42688.angular_velocity.x = Receive_Data.imu.gyro_data_x * GYROSCOPE_ICM42688_RATIO;
                            ICM42688.angular_velocity.y = Receive_Data.imu.gyro_data_y * GYROSCOPE_ICM42688_RATIO;
                            ICM42688.angular_velocity.z = Receive_Data.imu.gyro_data_z * GYROSCOPE_ICM42688_RATIO;
                            ICM42688.orientation = Quaternion(ICM42688.angular_velocity.x, ICM42688.angular_velocity.y, ICM42688.angular_velocity.z,
                                       ICM42688.linear_acceleration.x, ICM42688.linear_acceleration.y, ICM42688.linear_acceleration.z, 
                                       ICM42688.sample_time);
                            ICM42688_publisher.publish(ICM42688);
                        }
                        // BMI088
                        else if( data_imu->name == NAME_BMI088 )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)BMI088.count && BMI088.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)BMI088.count + 1 && BMI088.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)BMI088.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }

                                if(first == false)
                                    ROS_WARN("[Lost_Count] BMI088: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - BMI088.count - 1, BMI088.count);

                                if(first == true)
                                    first = false;
                            }

                            BMI088.sample_time = Receive_Data.imu.gyro_mcu_time/10000000000 + Receive_Data.imu.gyro_mcu_time%10000000000*0.0000000001 - BMI088.gyro_mcu_time;
                            BMI088.name = "BMI088";
                            BMI088.header.stamp = ros::Time::now(); 
                            BMI088.header.frame_id = frame_id; 
                            BMI088.number = data_imu->number;
                            BMI088.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            BMI088.accel_gps_time = Receive_Data.imu.accel_gps_time/10000000000 + Receive_Data.imu.accel_gps_time%10000000000*0.0000000001;
                            BMI088.accel_mcu_time = Receive_Data.imu.accel_mcu_time/10000000000 + Receive_Data.imu.accel_mcu_time%10000000000*0.0000000001;
                            BMI088.gyro_gps_time = Receive_Data.imu.gyro_gps_time/10000000000 + Receive_Data.imu.gyro_gps_time%10000000000*0.0000000001;
                            BMI088.gyro_mcu_time = Receive_Data.imu.gyro_mcu_time/10000000000 + Receive_Data.imu.gyro_mcu_time%10000000000*0.0000000001;
                            BMI088.linear_acceleration.x = Receive_Data.imu.accel_data_x * ACCEl_BMI088_RATIO;
                            BMI088.linear_acceleration.y = Receive_Data.imu.accel_data_y * ACCEl_BMI088_RATIO;
                            BMI088.linear_acceleration.z = Receive_Data.imu.accel_data_z * ACCEl_BMI088_RATIO;
                            BMI088.angular_velocity.x = Receive_Data.imu.gyro_data_x * GYROSCOPE_BMI088_RATIO;
                            BMI088.angular_velocity.y = Receive_Data.imu.gyro_data_y * GYROSCOPE_BMI088_RATIO;
                            BMI088.angular_velocity.z = Receive_Data.imu.gyro_data_z * GYROSCOPE_BMI088_RATIO;
                            BMI088.orientation = Quaternion(BMI088.angular_velocity.x, BMI088.angular_velocity.y, BMI088.angular_velocity.z,
                                       BMI088.linear_acceleration.x, BMI088.linear_acceleration.y, BMI088.linear_acceleration.z, 
                                       BMI088.sample_time);
                            BMI088_publisher.publish(BMI088);
                        }

                        return true;
                    }

                    // 磁力计数据包
                    else if(Receive_Data.rx[16] == TYPE_MAGNET)
                    {
                        data_magnet = (Magnet_Sensor_Data*) Receive_Data.rx;
                        // 序列号
                        M_UINT32.B4[3] = data_magnet->count1[0];
                        M_UINT32.B4[2] = data_magnet->count1[1];
                        M_UINT32.B4[1] = data_magnet->count1[2];
                        M_UINT32.B4[0] = data_magnet->count1[3];
                        Receive_Data.sequence[0] = M_UINT32.U32;
                        M_UINT32.B4[3] = data_magnet->count2[0];
                        M_UINT32.B4[2] = data_magnet->count2[1];
                        M_UINT32.B4[1] = data_magnet->count2[2];
                        M_UINT32.B4[0] = data_magnet->count2[3];
                        Receive_Data.sequence[1] = M_UINT32.U32;
    
                        // 时间
                        M_UINT64.B8[7] = data_magnet->magnet_gps_time[0];
                        M_UINT64.B8[6] = data_magnet->magnet_gps_time[1];
                        M_UINT64.B8[5] = data_magnet->magnet_gps_time[2];
                        M_UINT64.B8[4] = data_magnet->magnet_gps_time[3];
                        M_UINT64.B8[3] = data_magnet->magnet_gps_time[4];
                        M_UINT64.B8[2] = data_magnet->magnet_gps_time[5];
                        M_UINT64.B8[1] = data_magnet->magnet_gps_time[6];
                        M_UINT64.B8[0] = data_magnet->magnet_gps_time[7];
                        Receive_Data.magnet.magnet_gps_time = M_UINT64.U64;
                        M_UINT64.B8[7] = data_magnet->magnet_mcu_time[0];
                        M_UINT64.B8[6] = data_magnet->magnet_mcu_time[1];
                        M_UINT64.B8[5] = data_magnet->magnet_mcu_time[2];
                        M_UINT64.B8[4] = data_magnet->magnet_mcu_time[3];
                        M_UINT64.B8[3] = data_magnet->magnet_mcu_time[4];
                        M_UINT64.B8[2] = data_magnet->magnet_mcu_time[5];
                        M_UINT64.B8[1] = data_magnet->magnet_mcu_time[6];
                        M_UINT64.B8[0] = data_magnet->magnet_mcu_time[7];
                        Receive_Data.magnet.magnet_mcu_time = M_UINT64.U64;

                        // 数据
                        M_INT32.B4[3] = data_magnet->magnet_data_x[0];
                        M_INT32.B4[2] = data_magnet->magnet_data_x[1];
                        M_INT32.B4[1] = data_magnet->magnet_data_x[2];
                        M_INT32.B4[0] = data_magnet->magnet_data_x[3];
                        Receive_Data.magnet.magnet_data_x = M_INT32.B32;
                        M_INT32.B4[3] = data_magnet->magnet_data_y[0];
                        M_INT32.B4[2] = data_magnet->magnet_data_y[1];
                        M_INT32.B4[1] = data_magnet->magnet_data_y[2];
                        M_INT32.B4[0] = data_magnet->magnet_data_y[3];
                        Receive_Data.magnet.magnet_data_y = M_INT32.B32;
                        M_INT32.B4[3] = data_magnet->magnet_data_z[0];
                        M_INT32.B4[2] = data_magnet->magnet_data_z[1];
                        M_INT32.B4[1] = data_magnet->magnet_data_z[2];
                        M_INT32.B4[0] = data_magnet->magnet_data_z[3];
                        Receive_Data.magnet.magnet_data_z = M_INT32.B32;

                        // RM3100
                        if( data_magnet->name == NAME_RM3100 )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)RM3100.count && RM3100.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)RM3100.count + 1 && RM3100.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)RM3100.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }
                                if(first == false)
                                    ROS_WARN("[Lost_Count] RM3100: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - RM3100.count - 1, RM3100.count);
                            
                                if(first == true)
                                    first = false;
                            }

                            RM3100.sample_time = Receive_Data.magnet.magnet_mcu_time/10000000000 + Receive_Data.magnet.magnet_mcu_time%10000000000*0.0000000001 - RM3100.magnet_mcu_time;
                            RM3100.name = "RM3100";
                            RM3100.header.stamp = ros::Time::now(); 
                            RM3100.header.frame_id = frame_id; 
                            RM3100.number = data_magnet->number;
                            RM3100.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            RM3100.magnet_gps_time = Receive_Data.magnet.magnet_gps_time/10000000000 + Receive_Data.magnet.magnet_gps_time%10000000000*0.0000000001;
                            RM3100.magnet_mcu_time = Receive_Data.magnet.magnet_mcu_time/10000000000 + Receive_Data.magnet.magnet_mcu_time%10000000000*0.0000000001;
                            RM3100.magnet.x = Receive_Data.magnet.magnet_data_x * MAGNET_RM3100_RATIO;
                            RM3100.magnet.y = Receive_Data.magnet.magnet_data_y * MAGNET_RM3100_RATIO;
                            RM3100.magnet.z = Receive_Data.magnet.magnet_data_z * MAGNET_RM3100_RATIO;
                            RM3100_publisher.publish(RM3100);
                        }
                        // AK8975
                        else if( data_magnet->name == NAME_AK8975 )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)AK8975.count && AK8975.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)AK8975.count + 1 && AK8975.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)AK8975.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }
                                if(first == false)
                                    ROS_WARN("[Lost_Count] AK8975: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - AK8975.count - 1, AK8975.count);
                            
                                if(first == true)
                                    first = false;
                            }
                            
                            AK8975.sample_time = Receive_Data.magnet.magnet_mcu_time/10000000000 + Receive_Data.magnet.magnet_mcu_time%10000000000*0.0000000001 - AK8975.magnet_mcu_time;
                            AK8975.name = "AK8975";
                            AK8975.header.stamp = ros::Time::now(); 
                            AK8975.header.frame_id = frame_id; 
                            AK8975.number = data_magnet->number;
                            AK8975.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            AK8975.magnet_gps_time = Receive_Data.magnet.magnet_gps_time/10000000000 + Receive_Data.magnet.magnet_gps_time%10000000000*0.0000000001;
                            AK8975.magnet_mcu_time = Receive_Data.magnet.magnet_mcu_time/10000000000 + Receive_Data.magnet.magnet_mcu_time%10000000000*0.0000000001;
                            AK8975.magnet.x = Receive_Data.magnet.magnet_data_x * MAGNET_AK8975_RATIO;
                            AK8975.magnet.y = Receive_Data.magnet.magnet_data_y * MAGNET_AK8975_RATIO;
                            AK8975.magnet.z = Receive_Data.magnet.magnet_data_z * MAGNET_AK8975_RATIO;
                            AK8975_publisher.publish(AK8975);
                        }
                    }
                    
                    // GPS 数据包
                    else if(Receive_Data.rx[16] == TYPE_GPS)
                    {
                        data_gps = (GPS_Sensor_Data*) Receive_Data.rx;
                        // 序列号
                        M_UINT32.B4[3] = data_gps->count1[0];
                        M_UINT32.B4[2] = data_gps->count1[1];
                        M_UINT32.B4[1] = data_gps->count1[2];
                        M_UINT32.B4[0] = data_gps->count1[3];
                        Receive_Data.sequence[0] = M_UINT32.U32;
                        M_UINT32.B4[3] = data_gps->count2[0];
                        M_UINT32.B4[2] = data_gps->count2[1];
                        M_UINT32.B4[1] = data_gps->count2[2];
                        M_UINT32.B4[0] = data_gps->count2[3];
                        Receive_Data.sequence[1] = M_UINT32.U32;

                        // 时间
                        M_UINT64.B8[7] = data_gps->pps_gps_time[0];
                        M_UINT64.B8[6] = data_gps->pps_gps_time[1];
                        M_UINT64.B8[5] = data_gps->pps_gps_time[2];
                        M_UINT64.B8[4] = data_gps->pps_gps_time[3];
                        M_UINT64.B8[3] = data_gps->pps_gps_time[4];
                        M_UINT64.B8[2] = data_gps->pps_gps_time[5];
                        M_UINT64.B8[1] = data_gps->pps_gps_time[6];
                        M_UINT64.B8[0] = data_gps->pps_gps_time[7];
                        Receive_Data.gps.pps_gps_time = M_UINT64.U64;
                        M_UINT64.B8[7] = data_gps->pps_mcu_time[0];
                        M_UINT64.B8[6] = data_gps->pps_mcu_time[1];
                        M_UINT64.B8[5] = data_gps->pps_mcu_time[2];
                        M_UINT64.B8[4] = data_gps->pps_mcu_time[3];
                        M_UINT64.B8[3] = data_gps->pps_mcu_time[4];
                        M_UINT64.B8[2] = data_gps->pps_mcu_time[5];
                        M_UINT64.B8[1] = data_gps->pps_mcu_time[6];
                        M_UINT64.B8[0] = data_gps->pps_mcu_time[7];
                        Receive_Data.gps.pps_mcu_time = M_UINT64.U64;

                        M_INT64.B8[7] = data_gps->gps_error_time[0];
                        M_INT64.B8[6] = data_gps->gps_error_time[1];
                        M_INT64.B8[5] = data_gps->gps_error_time[2];
                        M_INT64.B8[4] = data_gps->gps_error_time[3];
                        M_INT64.B8[3] = data_gps->gps_error_time[4];
                        M_INT64.B8[2] = data_gps->gps_error_time[5];
                        M_INT64.B8[1] = data_gps->gps_error_time[6];
                        M_INT64.B8[0] = data_gps->gps_error_time[7];
                        Receive_Data.gps.gps_error_time = M_INT64.B64;

                        M_INT64.B8[7] = data_gps->gps_extra_error_time[0];
                        M_INT64.B8[6] = data_gps->gps_extra_error_time[1];
                        M_INT64.B8[5] = data_gps->gps_extra_error_time[2];
                        M_INT64.B8[4] = data_gps->gps_extra_error_time[3];
                        M_INT64.B8[3] = data_gps->gps_extra_error_time[4];
                        M_INT64.B8[2] = data_gps->gps_extra_error_time[5];
                        M_INT64.B8[1] = data_gps->gps_extra_error_time[6];
                        M_INT64.B8[0] = data_gps->gps_extra_error_time[7];
                        Receive_Data.gps.gps_extra_error_time = M_INT64.B64;

                        M_UINT16.B2[1] = data_gps->gps_scale[0];
                        M_UINT16.B2[0] = data_gps->gps_scale[1];
                        Receive_Data.gps.gps_scale = M_UINT16.U16;

                        // 数据
                        M_INT32.B4[3] = data_gps->longitude[0];
                        M_INT32.B4[2] = data_gps->longitude[1];
                        M_INT32.B4[1] = data_gps->longitude[2];
                        M_INT32.B4[0] = data_gps->longitude[3];
                        Receive_Data.gps.longitude = M_INT32.B32;
                        M_INT32.B4[3] = data_gps->latitude[0];
                        M_INT32.B4[2] = data_gps->latitude[1];
                        M_INT32.B4[1] = data_gps->latitude[2];
                        M_INT32.B4[0] = data_gps->latitude[3];
                        Receive_Data.gps.latitude = M_INT32.B32;
                        M_INT32.B4[3] = data_gps->height[0];
                        M_INT32.B4[2] = data_gps->height[1];
                        M_INT32.B4[1] = data_gps->height[2];
                        M_INT32.B4[0] = data_gps->height[3];
                        Receive_Data.gps.height = M_INT32.B32;

                        M_INT32.B4[3] = data_gps->velocity_n[0];
                        M_INT32.B4[2] = data_gps->velocity_n[1];
                        M_INT32.B4[1] = data_gps->velocity_n[2];
                        M_INT32.B4[0] = data_gps->velocity_n[3];
                        Receive_Data.gps.velocity_n = M_INT32.B32;
                        M_INT32.B4[3] = data_gps->velocity_e[0];
                        M_INT32.B4[2] = data_gps->velocity_e[1];
                        M_INT32.B4[1] = data_gps->velocity_e[2];
                        M_INT32.B4[0] = data_gps->velocity_e[3];
                        Receive_Data.gps.velocity_e = M_INT32.B32;
                        M_INT32.B4[3] = data_gps->velocity_d[0];
                        M_INT32.B4[2] = data_gps->velocity_d[1];
                        M_INT32.B4[1] = data_gps->velocity_d[2];
                        M_INT32.B4[0] = data_gps->velocity_d[3];
                        Receive_Data.gps.velocity_d = M_INT32.B32;

                        // ZEDF9P
                        if( data_gps->name == NAME_ZEDF9P )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)ZEDF9P.count && ZEDF9P.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)ZEDF9P.count + 1 && ZEDF9P.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)ZEDF9P.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }
                                if(first == false)
                                    ROS_WARN("[Lost_Count] ZEDF9P: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - ZEDF9P.count - 1, ZEDF9P.count);
                            
                                if(first == true)
                                    first = false;
                            }

                            ZEDF9P.name = "ZEDF9P";
                            ZEDF9P.header.stamp = ros::Time::now(); 
                            ZEDF9P.header.frame_id = frame_id; 
                            ZEDF9P.number = data_gps->number;
                            ZEDF9P.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            ZEDF9P.valid = data_gps->valid;
                            ZEDF9P.filter_en = data_gps->filter_en;
                            ZEDF9P.gps_scale = Receive_Data.gps.gps_scale/100000000 + Receive_Data.gps.gps_scale%100000000*0.00000001+1;
                            ZEDF9P.pps_gps_time = Receive_Data.gps.pps_gps_time/10000000000 + Receive_Data.gps.pps_gps_time%10000000000*0.0000000001;
                            ZEDF9P.pps_mcu_time = Receive_Data.gps.pps_mcu_time/10000000000 + Receive_Data.gps.pps_mcu_time%10000000000*0.0000000001;
                            ZEDF9P.gps_error_time = Receive_Data.gps.gps_error_time/10000000000 + Receive_Data.gps.gps_error_time%10000000000*0.0000000001;
                            ZEDF9P.gps_extra_error_time = Receive_Data.gps.gps_extra_error_time/10000000000 + Receive_Data.gps.gps_extra_error_time%10000000000*0.0000000001;
                            ZEDF9P.latitude = Receive_Data.gps.latitude/100000 + Receive_Data.gps.latitude%100000*0.00001;
                            ZEDF9P.longitude = Receive_Data.gps.longitude/100000 + Receive_Data.gps.longitude%100000*0.00001;
                            ZEDF9P.height = (Receive_Data.gps.height/100000 + Receive_Data.gps.height%100000*0.00001)/100;
                            ZEDF9P.gps_velocity.x = (Receive_Data.gps.velocity_n/100000 + Receive_Data.gps.velocity_n%100000*0.00001)/100;
                            ZEDF9P.gps_velocity.y = (Receive_Data.gps.velocity_e/100000 + Receive_Data.gps.velocity_e%100000*0.00001)/100;
                            ZEDF9P.gps_velocity.z = (Receive_Data.gps.velocity_d/100000 + Receive_Data.gps.velocity_d%100000*0.00001)/100;
                            ZEDF9P_publisher.publish(ZEDF9P);
                        }
                    }

                    // 相机数据包
                    else if(Receive_Data.rx[16] == TYPE_CAMERA)
                    {
                        data_camera = (Camera_Sensor_Data*) Receive_Data.rx;
                        // 序列号
                        M_UINT32.B4[3] = data_camera->count1[0];
                        M_UINT32.B4[2] = data_camera->count1[1];
                        M_UINT32.B4[1] = data_camera->count1[2];
                        M_UINT32.B4[0] = data_camera->count1[3];
                        Receive_Data.sequence[0] = M_UINT32.U32;
                        M_UINT32.B4[3] = data_camera->count2[0];
                        M_UINT32.B4[2] = data_camera->count2[1];
                        M_UINT32.B4[1] = data_camera->count2[2];
                        M_UINT32.B4[0] = data_camera->count2[3];
                        Receive_Data.sequence[1] = M_UINT32.U32;

                        // 时间
                        M_UINT64.B8[7] = data_camera->pulse_gps_time[0];
                        M_UINT64.B8[6] = data_camera->pulse_gps_time[1];
                        M_UINT64.B8[5] = data_camera->pulse_gps_time[2];
                        M_UINT64.B8[4] = data_camera->pulse_gps_time[3];
                        M_UINT64.B8[3] = data_camera->pulse_gps_time[4];
                        M_UINT64.B8[2] = data_camera->pulse_gps_time[5];
                        M_UINT64.B8[1] = data_camera->pulse_gps_time[6];
                        M_UINT64.B8[0] = data_camera->pulse_gps_time[7];
                        Receive_Data.camera.pulse_gps_time = M_UINT64.U64;
                        M_UINT64.B8[7] = data_camera->pulse_mcu_time[0];
                        M_UINT64.B8[6] = data_camera->pulse_mcu_time[1];
                        M_UINT64.B8[5] = data_camera->pulse_mcu_time[2];
                        M_UINT64.B8[4] = data_camera->pulse_mcu_time[3];
                        M_UINT64.B8[3] = data_camera->pulse_mcu_time[4];
                        M_UINT64.B8[2] = data_camera->pulse_mcu_time[5];
                        M_UINT64.B8[1] = data_camera->pulse_mcu_time[6];
                        M_UINT64.B8[0] = data_camera->pulse_mcu_time[7];
                        Receive_Data.camera.pulse_mcu_time = M_UINT64.U64;

                        // D435i
                        if( data_camera->name == NAME_D435I )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)D435i.count && D435i.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)D435i.count + 1 && D435i.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)D435i.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }
                                if(first == false)
                                    ROS_WARN("[Lost_Count] D435i: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - D435i.count - 1, D435i.count);

                                if(first == true)
                                    first = false;
                            }
                            D435i.name = "D435i";
                            D435i.header.stamp = ros::Time::now(); 
                            D435i.header.frame_id = frame_id; 
                            D435i.number = data_camera->number;
                            D435i.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            D435i.pulse_gps_time = Receive_Data.camera.pulse_gps_time/10000000000 + Receive_Data.camera.pulse_gps_time%10000000000*0.0000000001;
                            D435i.pulse_mcu_time = Receive_Data.camera.pulse_mcu_time/10000000000 + Receive_Data.camera.pulse_mcu_time%10000000000*0.0000000001;
                            D435i_publisher.publish(D435i);
                        }
                    }

                    // 气压计数据包
                    else if(Receive_Data.rx[16] == TYPE_BAROMETER)
                    {
                        data_barometer = (Barometer_Sensor_Data*) Receive_Data.rx;
                        // 序列号
                        M_UINT32.B4[3] = data_barometer->count1[0];
                        M_UINT32.B4[2] = data_barometer->count1[1];
                        M_UINT32.B4[1] = data_barometer->count1[2];
                        M_UINT32.B4[0] = data_barometer->count1[3];
                        Receive_Data.sequence[0] = M_UINT32.U32;
                        M_UINT32.B4[3] = data_barometer->count2[0];
                        M_UINT32.B4[2] = data_barometer->count2[1];
                        M_UINT32.B4[1] = data_barometer->count2[2];
                        M_UINT32.B4[0] = data_barometer->count2[3];
                        Receive_Data.sequence[1] = M_UINT32.U32;

                        // 时间
                        M_UINT64.B8[7] = data_barometer->data_gps_time[0];
                        M_UINT64.B8[6] = data_barometer->data_gps_time[1];
                        M_UINT64.B8[5] = data_barometer->data_gps_time[2];
                        M_UINT64.B8[4] = data_barometer->data_gps_time[3];
                        M_UINT64.B8[3] = data_barometer->data_gps_time[4];
                        M_UINT64.B8[2] = data_barometer->data_gps_time[5];
                        M_UINT64.B8[1] = data_barometer->data_gps_time[6];
                        M_UINT64.B8[0] = data_barometer->data_gps_time[7];
                        Receive_Data.barometer.data_gps_time = M_UINT64.U64;
                        M_UINT64.B8[7] = data_barometer->data_mcu_time[0];
                        M_UINT64.B8[6] = data_barometer->data_mcu_time[1];
                        M_UINT64.B8[5] = data_barometer->data_mcu_time[2];
                        M_UINT64.B8[4] = data_barometer->data_mcu_time[3];
                        M_UINT64.B8[3] = data_barometer->data_mcu_time[4];
                        M_UINT64.B8[2] = data_barometer->data_mcu_time[5];
                        M_UINT64.B8[1] = data_barometer->data_mcu_time[6];
                        M_UINT64.B8[0] = data_barometer->data_mcu_time[7];
                        Receive_Data.barometer.data_mcu_time = M_UINT64.U64;

                        // 数据
                        M_INT32.B4[3] = data_barometer->height[0];
                        M_INT32.B4[2] = data_barometer->height[1];
                        M_INT32.B4[1] = data_barometer->height[2];
                        M_INT32.B4[0] = data_barometer->height[3];
                        Receive_Data.barometer.height = M_INT32.B32;

                        // MS5611
                        if( data_barometer->name == NAME_MS5611 )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)MS5611.count && MS5611.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)MS5611.count + 1 && MS5611.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)MS5611.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }
                                if(first == false)
                                    ROS_WARN("[Lost_Count] MS5611: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - MS5611.count - 1, MS5611.count);

                                if(first == true)
                                    first = false;
                            }

                            MS5611.name = "MS5611";
                            MS5611.header.stamp = ros::Time::now(); 
                            MS5611.header.frame_id = frame_id; 
                            MS5611.number = data_barometer->number;
                            MS5611.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            MS5611.data_gps_time = Receive_Data.barometer.data_gps_time/10000000000 + Receive_Data.barometer.data_gps_time%10000000000*0.0000000001;
                            MS5611.data_mcu_time = Receive_Data.barometer.data_mcu_time/10000000000 + Receive_Data.barometer.data_mcu_time%10000000000*0.0000000001;
                            MS5611.height = Receive_Data.barometer.height;
                            MS5611_publisher.publish(MS5611);
                        }
                    }

                    // 命令数据包
                    else if(Receive_Data.rx[16] == TYPE_COMMAND)
                    {
                        data_command = (Command_Data*) Receive_Data.rx;

                        // 序列号
                        M_UINT32.B4[3] = data_command->count1[0];
                        M_UINT32.B4[2] = data_command->count1[1];
                        M_UINT32.B4[1] = data_command->count1[2];
                        M_UINT32.B4[0] = data_command->count1[3];
                        Receive_Data.sequence[0] = M_UINT32.U32;
                        M_UINT32.B4[3] = data_command->count2[0];
                        M_UINT32.B4[2] = data_command->count2[1];
                        M_UINT32.B4[1] = data_command->count2[2];
                        M_UINT32.B4[0] = data_command->count2[3];
                        Receive_Data.sequence[1] = M_UINT32.U32;

                        // 采集命令
                        if( data_command->name == NAME_ACQUSITION )
                        {
                            // 收到以前的数据
                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 <= (long int)Command.count && Command.count !=0 )
                            {
                                return false;
                            }

                            if( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 != (long int)Command.count + 1 && Command.count !=0 )
                            {
                                // 错误过多
                                if( (IN_RANGE( Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296, (long int)Command.count, MAX_LOST_COUNT )) != true )
                                {
                                    // return false;
                                }
                                if(first == false)
                                    ROS_WARN("[Lost_Count] Command: %ld, %ld", Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296 - Command.count - 1, Command.count);
                                
                                if(first == true)
                                    first = false;
                            }
                            
                            Command.name = "Acqusition";
                            Command.count = Receive_Data.sequence[0] + Receive_Data.sequence[1] * 4294967296;
                            Command.header.stamp = ros::Time::now(); 
                            Command.command = data_command->command;
                            Command_publisher.publish(Command);
                        }

                    }
                    
                    length = 0;
                    return true;
                }
                else
                {
                    count = 0;
                    length = 0;
                    ROS_ERROR("State:rx_data");
                    state = rx_frame_header;
                    return false;
                }
            }
            break;
        }   
        
        default:
        {
            ROS_ERROR("State:default");
            state = rx_frame_header;
            length = 0;
            count = 0;
            break;
        }
    }

    return false;
}

/** 
 * @author WeiXuan
 * @brief 处理数据线程
 * @returns 
 */
void robot::thread_process(int id)
{
    ROS_INFO("Thread_process ready!");

    static uint8_t data_in[1];

    while(running)
    {
        // 获取数据
        if(data_quene.Pop(data_in) )
        {
            Get_Sensor_Data( data_in[0] );
        }
    }
}

/** 
 * @author WeiXuan
 * @brief 获取数据流送入队列
 * @returns 
 */
void robot::thread_receieve(int id)
{
    ROS_INFO("Thread_receieve ready!");
    while(running)
    {

        if(use_way == 0)
        {
            // 接受端口数据
            if ((clientSocket = accept(serverSocket, (struct sockaddr *)&clientAddr, &addrLen)) < 0)
            { 
                perror("Accepting error");
            }
        }
        while (ros::ok()) 
        {
            if(use_way == 0)
            {
                uint64_t bytesRead = recv(clientSocket, buffer, 1, 0);
                if (bytesRead < 0)
                {
                    perror("Receiving error");
                    break;
                    
                }
                else if (bytesRead == 0)
                {
                    break;
                }
                else
                {
                    std::vector<char> bufferData(buffer, buffer + bytesRead);
                    if (bufferData.size() > 0)
                    {
                        for(uint64_t i = 0; i < bufferData.size(); ++i)
                        {
                            data_quene.Push(bufferData[i]);
                        }
                    }
                }
            }
            else
            {
                // 下位机数据
                uint8_t data[1];
                // 读取数据
                BYH_Serial.read(data,1); 
                data_quene.Push(data[0]);
            }
        }
    }
}

/** 
 * @author WeiXuan
 * @brief 频率话题订阅回调函数
 * @param &cmd_frequence
 * @returns 
 */        
void robot::Cmd_Frequence_Callback(const byh_uav::uav_cmd_frequence::ConstPtr &cmd_frequence)
{
    static uint32_t Command_count_H;
    static uint32_t Command_count_L;

    Frequence_Command_Data Send_buffer;
    uint8_t* buffer = (uint8_t*) &Send_buffer;

    // 帧头
    Send_buffer.frame_header1 = FRAME_HEADER1;
    Send_buffer.frame_header2 = FRAME_HEADER2;

    // 帧长度
    M_UINT32.U32 = sizeof(Frequence_Command_Data);
    Send_buffer.length[0] = M_UINT32.B4[3];
    Send_buffer.length[1] = M_UINT32.B4[2];
    Send_buffer.length[2] = M_UINT32.B4[1];
    Send_buffer.length[3] = M_UINT32.B4[0];

    // 帧头校验
    Send_buffer.calib[0] = Check_Sum(6, SEND_DATA_CHECK, buffer);
    Send_buffer.calib[1] = Check_Sum(7, SEND_DATA_CHECK, buffer);

    // 低位计数
    M_UINT32.U32 = Command_count_L;
    Send_buffer.count1[0] = M_UINT32.B4[3];
    Send_buffer.count1[1] = M_UINT32.B4[2];
    Send_buffer.count1[2] = M_UINT32.B4[1];
    Send_buffer.count1[3] = M_UINT32.B4[0];

    // 高位计数
    M_UINT32.U32 = Command_count_H;
    Send_buffer.count2[0] = M_UINT32.B4[3];
    Send_buffer.count2[1] = M_UINT32.B4[2];
    Send_buffer.count2[2] = M_UINT32.B4[1];
    Send_buffer.count2[3] = M_UINT32.B4[0];

    // 数据类型
    Send_buffer.type = TYPE_COMMAND;
    Send_buffer.name = NAME_CHANGE_FREQUENCE;
        
    // 数据
    M_UINT16.U16 = cmd_frequence->channel;
    Send_buffer.channel[0] = M_UINT16.B2[0];
    Send_buffer.channel[1] = M_UINT16.B2[1];
    M_UINT16.U16 = cmd_frequence->frequence;
    Send_buffer.frequence[0] = M_UINT16.B2[0];
    Send_buffer.frequence[1] = M_UINT16.B2[1];

    // 校验码
    Send_buffer.crc_calib = Check_Sum(sizeof(Send_buffer)-2, SEND_DATA_CHECK, buffer);
    // 帧尾
    Send_buffer.frame_tail = FRAME_TAIL;

    // 计数
    if(Command_count_L == 4294967295)
    {
        Command_count_H++;
        Command_count_L = 0;
    }
    else
        Command_count_L++;
    
    // 以太网
    if( use_way == 0 )
    {
        // 发送数据
        int64_t bytesSend = write(clientSocket, buffer, sizeof(Frequence_Command_Data));
        if (bytesSend < 0)
        {
            ROS_INFO_STREAM("Set Frequence error!");
        }
        else
        {
            ROS_INFO_STREAM("Set Frequence ok!");
        }
    }
    // 串口
    else
    {
        try
        {
            // 通过串口向下位机发送数据 
            BYH_Serial.write( buffer, sizeof(Frequence_Command_Data) ); 
            ROS_INFO_STREAM("Set Frequence ok!");
        }
        catch (serial::IOException& e)   
        {
            ROS_INFO_STREAM("Set Frequence error!");
        }
    }
    
}

/** 
 * @author WeiXuan
 * @brief 构造函数
 * @returns 
 */
robot::robot():Power_voltage(0)
{
    /* 使用接口配置 */
        // 传输方式
        private_nh.param<int>("use_way", use_way, 0); 
        // 使用线程
        private_nh.param<bool>("use_thread", use_thread, false);
    /* 使用接口配置 */

    /* 串口配置 */
        // 固定串口号
        private_nh.param<std::string>("uart_port_name", uart_port_name, "/dev/ttyTHS0"); 
        // 和下位机通信波特率
        private_nh.param<int>("uart_baud_rate", serial_baud_rate, 3000000);
    /* 串口配置 */

    /* 以太网配置 */
        // ros服务端IP地址
        private_nh.param<std::string>("eth_ip", eth_ip, "null"); 
        // ros服务端端口号
        private_nh.param<int>("eth_port", eth_port, 5001); 
    /* 以太网配置 */

    // IMU话题对应TF坐标
    private_nh.param<std::string>("frame_id", frame_id, "byh_uav_frame"); 
 
    /* DEBUG输出 */
        std::string str;
        str = (use_way == 0) ? "ethnet" : "usart";
        ROS_INFO("The way used is %s", str.c_str());

        str = (use_thread == false) ? "false": "true";
        ROS_INFO("Using thread is %s", str.c_str());

        // 以太网传输
        if(use_way == 0)
        {
            ROS_INFO("The eth_ip is %s", eth_ip.c_str());
            ROS_INFO("The eth_port is %d", eth_port);
        }
        // 串口传输
        else if(use_way == 1)
        {
            ROS_INFO("The uart_port_name is %s", uart_port_name.c_str());
            ROS_INFO("The uart_baud_rate is %d", serial_baud_rate);
        }
        // 坐标系
        ROS_INFO("The frame_id is %s", frame_id.c_str());
    /* DEBUG输出 */

    // 创建电池电压话题发布者
    Voltage_publisher = private_nh.advertise<std_msgs::Float32>("byh_uav/Powervoltage", 10); 

    // 创建IMU话题发布者
    BMI088_publisher = private_nh.advertise<byh_uav::uav_imu>("byh_uav/BMI088", 20); 
    ADIS16470_publisher = private_nh.advertise<byh_uav::uav_imu>("byh_uav/ADIS16470", 20); 
    ICM42688_publisher = private_nh.advertise<byh_uav::uav_imu>("byh_uav/ICM42688", 20); 

    // 创建磁力计话题发布者
    AK8975_publisher = private_nh.advertise<byh_uav::uav_magnet>("byh_uav/AK8975", 20); 
    RM3100_publisher = private_nh.advertise<byh_uav::uav_magnet>("byh_uav/RM3100", 20); 

    // 创建气压计话题发布者
    MS5611_publisher = private_nh.advertise<byh_uav::uav_barometer>("byh_uav/MS5611", 20);

    // 创建GPS话题发布者
    ZEDF9P_publisher = private_nh.advertise<byh_uav::uav_gps>("byh_uav/ZEDF9P", 20);

    // 创建触发频率发布者
    D435i_publisher = private_nh.advertise<byh_uav::uav_frequence>("byh_uav/D435i", 20);
    
    // 命令发布者
    Command_publisher = private_nh.advertise<byh_uav::uav_command>("byh_uav/Command", 20);

    // Trigger订阅回调函数设置
    trigger_subscriber = private_nh.subscribe("byh_uav/cmd_frequence", 10, &robot::Cmd_Frequence_Callback, this); 

    ROS_INFO_STREAM("BYH_UAV data ready!");

    // 使用以太网传输
    if( use_way == 0 )
    {
        /* 创建套接字 */
            if ((serverSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
            {
                perror("Socket creation error");
                return;
            }
            int on = 1;
            int ret = setsockopt( serverSocket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on) );
        /* 创建套接字 */

        /* 绑定地址与端口 */
            // socket使用IPv4协议
            serverAddr.sin_family = AF_INET;
            // 监听的端口号
            serverAddr.sin_port = htons(eth_port);
            if(eth_ip == "null")
            {
                // 接收来自任何可用地址的连接请求
                serverAddr.sin_addr.s_addr = INADDR_ANY;
            }
            else
            {
                // 绑定特定IP
                char ip_address[20];
                strcpy( ip_address, eth_ip.c_str() );
                if (inet_pton(AF_INET, ip_address, &serverAddr.sin_addr) < 0)
                {
                    perror("Invalid address");
                    return;
                }
            }
            if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
            {
                perror("Binding error");
                return;
            }
        /* 绑定地址与端口 */

        /* 开启监听 */
            if (listen(serverSocket, 5) < 0)
            {
                perror("Listening error");
                return;
            }
            ROS_INFO_STREAM("BYH_ETH start listening on port!");
        /* 开启监听 */
    }
    // 使用串口传输
    else
    {
        try
        { 
            // 尝试初始化与开启串口
            BYH_Serial.setPort(uart_port_name); 
            BYH_Serial.setBaudrate(serial_baud_rate); 
            // 超时等待
            serial::Timeout _time = serial::Timeout::simpleTimeout(2000); 
            BYH_Serial.setTimeout(_time);
            BYH_Serial.open();
            
        }
        catch (serial::IOException& e)
        {
            ROS_ERROR_STREAM("BYH_UAV_UART can not open serial port,Please check the serial port cable! ");
            return;
        }
        if(BYH_Serial.isOpen())
        {
            ROS_INFO_STREAM("BYH_Serial_UART serial port opened!");
        }
    }

    if(use_thread == true)
    {
        running = true;
        mission_process = std::thread(&robot::thread_process, this, 1);
        mission_receieve = std::thread(&robot::thread_receieve, this, 2);
    }
}

/** 
 * @author WeiXuan
 * @brief 析构函数
 * @returns 
 */
robot::~robot()
{

    if( use_thread == true )
    {
        running = false;
        if (mission_receieve.joinable()) 
        {
            mission_receieve.join();
        }
        if (mission_process.joinable()) 
        {
            mission_process.join(); 
        }
    }

    if( use_way == 0 )
    {
        close(clientSocket);
        close(serverSocket);
    }
    else
    {
        BYH_Serial.close(); 
    }

    ROS_INFO_STREAM("Shutting down"); 
}
