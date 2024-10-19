#include "byh_uav_record.h"

// 创建流对象
ofstream ofs_adis16470;
ofstream ofs_icm42688;
ofstream ofs_bmi088;
ofstream ofs_rm3100;
ofstream ofs_ak8975;
ofstream ofs_ms5611;
ofstream ofs_zedf9p;
ofstream ofs_gpspps;
ofstream ofs_livox;

string direct;

// 采集标志
static bool acqusition = false;

// ADIS16470 回调函数
void ADIS16470_Callback(const byh_uav::uav_imu::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_adis16470.is_open() && (acqusition == true) ) 
    {
        ofs_adis16470 << msg->count << "\t"
            << (uint64_t)(msg->gyro_gps_time * 10000000000) / 10000000000 << "."
            << setw(10) << setfill('0') << (uint64_t)(msg->gyro_gps_time * 10000000000) % 10000000000 << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.x << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.y << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.z << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.x << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.y << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.z << "\t" << "\n";
    }
}

// ICM42688 回调函数
void ICM42688_Callback(const byh_uav::uav_imu::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_icm42688.is_open() && (acqusition == true) ) 
    {
        ofs_icm42688 << msg->count << "\t" 
            << (uint64_t)(msg->gyro_gps_time * 10000000000) / 10000000000 << "."
            << setw(10) << setfill('0') << (uint64_t)(msg->gyro_gps_time * 10000000000) % 10000000000 << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.x << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.y << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.z << "\t" 
            << fixed<<setprecision(10) << msg->linear_acceleration.x << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.y << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.z << "\t" << "\n";
    }
}

// BMI088 回调函数
void BMI088_Callback(const byh_uav::uav_imu::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_bmi088.is_open() && (acqusition == true) ) 
    {
        ofs_bmi088 << msg->count << "\t"
            << (uint64_t)(msg->gyro_gps_time * 10000000000) / 10000000000 << "."
            << setw(10) << setfill('0') << (uint64_t)(msg->gyro_gps_time * 10000000000) % 10000000000 << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.x << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.y << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.z << "\t"
            << (uint64_t)(msg->accel_gps_time * 10000000000) / 10000000000 << "."
            << setw(10) << setfill('0') << (uint64_t)(msg->accel_gps_time * 10000000000) % 10000000000 << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.x << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.y << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.z << "\t" << "\n";
    }
}

// RM3100 回调函数
void RM3100_Callback(const byh_uav::uav_magnet::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_rm3100.is_open() && (acqusition == true) ) 
    {
        ofs_rm3100 << msg->count << "\t"
            << (uint64_t)(msg->magnet_gps_time * 10000000000) / 10000000000 << "."
            << setw(10) << setfill('0') << (uint64_t)(msg->magnet_gps_time * 10000000000) % 10000000000 << "\t"
            << fixed<<setprecision(10) << msg->magnet.x << "\t"
            << fixed<<setprecision(10) << msg->magnet.y << "\t"
            << fixed<<setprecision(10) << msg->magnet.z << "\t" << "\n";
    }
}

// AK8975 回调函数
void AK8975_Callback(const byh_uav::uav_magnet::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_ak8975.is_open() && (acqusition == true) ) 
    {
        ofs_ak8975 << msg->count << "\t"
            << (uint64_t)(msg->magnet_gps_time * 10000000000) / 10000000000 << "."
            << setw(10) << setfill('0') << (uint64_t)(msg->magnet_gps_time * 10000000000) % 10000000000 << "\t"
            << fixed<<setprecision(10) << msg->magnet.x << "\t"
            << fixed<<setprecision(10) << msg->magnet.y << "\t"
            << fixed<<setprecision(10) << msg->magnet.z << "\t" << "\n";
    }
}

// MS5611 回调函数
void MS5611_Callback(const byh_uav::uav_barometer::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_ms5611.is_open() && (acqusition == true) ) 
    {
        ofs_ms5611 << msg->count << "\t"
            << (uint64_t)(msg->data_gps_time * 10000000000) / 10000000000 << "."
            << setw(10) << setfill('0') << (uint64_t)(msg->data_gps_time * 10000000000) % 10000000000 << "\t"
            << fixed<<setprecision(10) << msg->height << "\t"<< "\n";
    }
}

// ZEDF9P 回调函数
void ZEDF9P_Callback(const byh_uav::uav_gps::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_zedf9p.is_open() && (acqusition == true) ) 
    {
        ofs_zedf9p << msg->count << "\t"
            << (uint64_t)(msg->pps_gps_time * 10000000000) / 10000000000 << "."
            << setw(10) << setfill('0') << (uint64_t)(msg->pps_gps_time * 10000000000) % 10000000000 << "\t"
            << fixed<<setprecision(10) << msg->longitude << "\t"
            << fixed<<setprecision(10) << msg->latitude << "\t"
            << fixed<<setprecision(10) << msg->height << "\t" 
            << fixed<<setprecision(10) << msg->gps_velocity.x << "\t"
            << fixed<<setprecision(10) << msg->gps_velocity.y << "\t"
            << fixed<<setprecision(10) << msg->gps_velocity.z << "\t"<< "\n";
    }
}

// GPSPPS 回调函数
void GPSPPS_Callback(const byh_uav::uav_pps::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_gpspps.is_open() && (acqusition == true) ) 
    {
        ofs_gpspps << msg->count << "\t"
            << (uint64_t)(msg->pulse_gps_time.sec * 10000000000) / 10000000000 << "."
            << setw(9) << setfill('0') << right << (uint64_t)(msg->pulse_gps_time.nsec) << "\t"
            << (uint64_t)(msg->pulse_sys_time.sec * 10000000000) / 10000000000 << "."
            << setw(9) << setfill('0') << right << (uint64_t)(msg->pulse_sys_time.nsec) << "\t" << "\n";
    }
}

// LIVOX 回调函数
void LIVOX_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 文件已经打开
    if ( ofs_livox.is_open() && (acqusition == true) ) 
    {
        ofs_livox << msg->header.seq << "\t"
            << (uint64_t)(msg->header.stamp.sec * 10000000000) / 10000000000 << "."
            << setw(9) << setfill('0') << right << (uint64_t)(msg->header.stamp.nsec) << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.x << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.y << "\t"
            << fixed<<setprecision(10) << msg->angular_velocity.z << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.x * 9.80665 << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.y * 9.80665 << "\t"
            << fixed<<setprecision(10) << msg->linear_acceleration.z * 9.80665 << "\t" << "\n";
    }
}

// 命令回调函数
void Command_Callback(const byh_uav::uav_command::ConstPtr& msg)
{
    if( msg->command ==  START )
    {
        ROS_INFO("Start Collect!!!");

        std::time_t t = ros::Time::now().toSec();
        std::tm* local_time = std::localtime(&t);
        std::ostringstream os;
        os << std::put_time(local_time, "%Y_%m_%d_%H_%M_%S");
        string str2 = os.str();
        string str3 = direct + string("/data_") + str2;
        string str4;
        const char *data;
        const char *directory = str3.data();
        // 如果文件夹不存在
        if( access( directory, 0 ) == -1 )	
        {
            // 则创建
            mkdir( directory, 0771 ); 
        }

        str4 = str3 + string("/byhuav_imu_adis16470.txt");
        data = str4.data();
        ofs_adis16470.open( data , ios::in | ios::out | ios::app );

        str4 = str3 + string("/byhuav_imu_icm42688.txt");
        data = str4.data();
        ofs_icm42688.open( data , ios::in | ios::out | ios::app );

        str4 = str3 + string("/byhuav_imu_bmi088.txt");
        data = str4.data();
        ofs_bmi088.open( data , ios::in | ios::out | ios::app );

        str4 = str3 + string("/byhuav_magnet_rm3100.txt");
        data = str4.data();
        ofs_rm3100.open( data , ios::in | ios::out | ios::app );

        str4 = str3 + string("/byhuav_magnet_ak8975.txt");
        data = str4.data();
        ofs_ak8975.open( data , ios::in | ios::out | ios::app );

        str4 = str3 + string("/byhuav_barometer_ms5611.txt");
        data = str4.data();
        ofs_ms5611.open( data , ios::in | ios::out | ios::app );

        str4 = str3 + string("/byhuav_gps_zedf9p.txt");
        data = str4.data();
        ofs_zedf9p.open( data , ios::in | ios::out | ios::app );

        str4 = str3 + string("/byhuav_gps_pps.txt");
        data = str4.data();
        ofs_gpspps.open( data , ios::in | ios::out | ios::app );

        str4 = str3 + string("/byhuav_livox_imu.txt");
        data = str4.data();
        ofs_livox.open( data , ios::in | ios::out | ios::app );

        string str5 = string(" gnome-terminal -- zsh -c ") + string(" \"cd ") + str3 + string("\n rosbag record -a\"");
        data = str5.data();
        system( data );

        acqusition = true;
    }    
    else if( msg->command ==  STOP )
    {
        ROS_INFO("End Collect!!!");
        ofs_adis16470.close();
        ofs_icm42688.close();
        ofs_bmi088.close();
        ofs_rm3100.close();
        ofs_ak8975.close();
        ofs_ms5611.close();
        ofs_zedf9p.close();
        ofs_gpspps.close();
        ofs_livox.close();
        acqusition = false;
        system( "killall record" );
    }
}

/** 
 * @author WeiXuan
 * @brief 主函数，ROS初始化
 * @param argc
 * @param argv
 * @returns 
 */
int main(int argc, char** argv)
{
    // ROS初始化 并设置节点名称 
    ros::init(argc, argv, "byh_uav_record"); 

    // 声明ros句柄
    ros::NodeHandle n;
    // 记录文件路径
    n.param<std::string>("direction", direct, "/home/kuper/Ros/byh_uav_ws/src/byh_uav_record/data"); 
    // 声明订阅器
    ros::Subscriber Command_sub = n.subscribe("/byh_uav/Command", 20, Command_Callback);
    ros::Subscriber ADIS16470_sub = n.subscribe("/byh_uav/ADIS16470", 20, ADIS16470_Callback);
    ros::Subscriber ICM42688_sub = n.subscribe("/byh_uav/ICM42688", 20, ICM42688_Callback);
    ros::Subscriber BMI088_sub = n.subscribe("/byh_uav/BMI088", 20, BMI088_Callback);
    ros::Subscriber RM3100_sub = n.subscribe("/byh_uav/RM3100", 20, RM3100_Callback);
    ros::Subscriber AK8975_sub = n.subscribe("/byh_uav/AK8975", 20, AK8975_Callback);
    ros::Subscriber MS5611_sub = n.subscribe("/byh_uav/MS5611", 20, MS5611_Callback);
    ros::Subscriber ZEDF9P_sub = n.subscribe("/byh_uav/ZEDF9P", 20, ZEDF9P_Callback);
    ros::Subscriber GPSPPS_sub = n.subscribe("/byh_uav/pps_sys", 20, GPSPPS_Callback);
    ros::Subscriber LIVOX_sub = n.subscribe("/livox/imu", 20, LIVOX_Callback);

    while(ros::ok())
    {
        if(acqusition == false)
        {
            
        }
        ros::spinOnce();
    }
    
    return 0;  
} 
