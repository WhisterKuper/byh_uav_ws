#include "byh_uav_trigger.h"

byh_uav::uav_frequence trigger_time;

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
    ros::init(argc, argv, "byh_uav_trigger"); 

    // 实例化一个对象
    robot byhuav;
    ROS_INFO_STREAM("Welcome to BYH_UAV_TRIGGER!");
    
    // 循环执行数据采集和发布话题等操作
    byhuav.Control(); 

    return 0;  
} 

// PPS 回调函数
void robot::PPS_Callback(const byh_uav::uav_pps_all::ConstPtr &pps_time)
{
    
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
    // 晶振计数器计数值
	uint64_t osc_count = 0;

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
                    state = rx_data;
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
        case rx_data:
        {
            if( count<11 )
            {
                Receive_Data.rx[count] = data[0];
                count++;
            }
            else 
            {
                Receive_Data.rx[count] = data[0];
                ROS_DEBUG("state: %d, count: %d, data: %x", state, count, data[0]);
                state = rx_frame_header;
                count = 0;
                
                // 帧尾以及校验正确
                if( (Receive_Data.rx[11] == Receive_Data.rx[2]^Receive_Data.rx[3]^Receive_Data.rx[4]^Receive_Data.rx[5]^Receive_Data.rx[6]^Receive_Data.rx[7]^Receive_Data.rx[8]^Receive_Data.rx[9]^Receive_Data.rx[10]) )
                {
                    // 获取数据
                    M_UINT64.B8[7] = Receive_Data.rx[3];
                    M_UINT64.B8[6] = Receive_Data.rx[4];
                    M_UINT64.B8[5] = Receive_Data.rx[5];
                    M_UINT64.B8[4] = Receive_Data.rx[6];
                    M_UINT64.B8[3] = Receive_Data.rx[7];
                    M_UINT64.B8[2] = Receive_Data.rx[8];
                    M_UINT64.B8[1] = Receive_Data.rx[9];
                    M_UINT64.B8[0] = Receive_Data.rx[10];
                    // 获得晶振计数
                    osc_count = M_UINT64.U64;
                    // 转换时间
                    double fpga_time = double(osc_count) / 320000000;
                    // ROS_INFO("Get_data: %lf \n", fpga_time);

                    // 发布数据
                    trigger_time.header.stamp = ros::Time::now(); 
                    trigger_time.count++;
                    trigger_time.number = 1;
		            trigger_time.header.frame_id = "trigger_time"; 
                    trigger_time.pulse_mcu_time = fpga_time;
                    if(channel == 1)
                        trigger_time.name = "1V8_1";
                    else if(channel == 2)
                        trigger_time.name = "1V8_2";
                    else if(channel == 3)
                        trigger_time.name = "1V8_3";
                    else if(channel == 4)
                        trigger_time.name = "5V_1";
                    else if(channel == 5)
                        trigger_time.name = "5V_2";
                    else if(channel == 6)
                        trigger_time.name = "5V_3";
                    else if(channel == 7)
                        trigger_time.name = "3V3_1";
                    else if(channel == 8)
                        trigger_time.name = "3V3_2";
                    time_publisher.publish(trigger_time);
                    return true;
                }
                else
                {
                    count = 0;
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
 * @brief 构造函数
 * @returns 
 */
robot::robot()
{
    std::string node_name = ros::this_node::getName();
    // ROS_INFO("当前节点名称: %s", node_name.c_str());

    /* 使用接口配置 */
        // 传输方式
        private_nh.param<int>(node_name+"/use_way", use_way, 1); 
        // 使用线程
        private_nh.param<bool>(node_name+"/use_thread", use_thread, false);
    /* 使用接口配置 */
    
    /* 串口配置 */
        // 固定串口号
        private_nh.param<std::string>(node_name+"/ch348_port_name", ch348_port_name, "/dev/ttyCH9344USB8"); 
        // 和下位机通信波特率
        private_nh.param<int>(node_name+"/ch348_baud_rate", ch348_baud_rate, 3000000);
    /* 串口配置 */

    /* 以太网配置 */
        // ros服务端IP地址
        private_nh.param<std::string>(node_name+"/eth_ip", eth_ip, "null"); 
        // ros服务端端口号
        private_nh.param<int>(node_name+"/eth_port", eth_port, 5001); 
    /* 以太网配置 */

    /* 话题发送名称 */
        private_nh.param<std::string>(node_name+"/topic", topic, "/byh_uav/trigger0"); 
    /* 话题发送名称 */

    // IMU话题对应TF坐标
    private_nh.param<std::string>(node_name+"/frame_id", frame_id, "byh_uav_frame"); 
 
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
            ROS_INFO("The ch348_port_name is %s", ch348_port_name.c_str());
            ROS_INFO("The uart_baud_rate is %d", ch348_baud_rate);
        }
        // 坐标系
        ROS_INFO("The frame_id is %s", frame_id.c_str());
        // 端口号
        if(ch348_port_name == "/dev/ttyCH9344USB8")
        {
            channel = 1;
            ROS_INFO("The Current Channel is 1V8-1");
        }
        else if(ch348_port_name == "/dev/ttyCH9344USB9")
        {
            channel = 2;
            ROS_INFO("The Current Channel is 1V8-2");
        }
        else if(ch348_port_name == "/dev/ttyCH9344USB10")
        {
            channel = 3;
            ROS_INFO("The Current Channel is 1V8-3");
        }
        else if(ch348_port_name == "/dev/ttyCH9344USB11")
        {
            channel = 4;
            ROS_INFO("The Current Channel is 5V-1");
        }
        else if(ch348_port_name == "/dev/ttyCH9344USB12")
        {
            channel = 5;
            ROS_INFO("The Current Channel is 5V-2");
        }
        else if(ch348_port_name == "/dev/ttyCH9344USB13")
        {
            channel = 6;
            ROS_INFO("The Current Channel is 5V-3");
        }
        else if(ch348_port_name == "/dev/ttyCH9344USB14")
        {
            channel = 7;
            ROS_INFO("The Current Channel is 3V3-1");
        }
        else if(ch348_port_name == "/dev/ttyCH9344USB15")
        {
            channel = 8;
            ROS_INFO("The Current Channel is 3V3-2");
        }
    /* DEBUG输出 */

    // 创建触发频率发布者
    time_publisher = private_nh.advertise<byh_uav::uav_frequence>(topic, 20);

    // Trigger订阅回调函数设置
    pps_subscriber = private_nh.subscribe("byh_uav/pps_sys", 10, &robot::PPS_Callback, this); 

    ROS_INFO_STREAM("BYH_UAV_Trigger data ready!");

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
            BYH_Serial.setPort(ch348_port_name); 
            BYH_Serial.setBaudrate(ch348_baud_rate); 
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
