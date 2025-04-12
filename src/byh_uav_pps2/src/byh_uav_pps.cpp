#include "byh_uav_pps.h"

// 创建话题发布者
ros::Publisher pps_publisher;

// PPS 时间
static byh_uav::uav_pps_all pps;

// 标志
static bool flag = false;

static struct timespec offset_assert = {0, 0};

int find_source(char *path, pps_handle_t *handle, int *avail_mode)
{
	pps_params_t params;
	int ret;

	printf("trying PPS source \"%s\"\n", path);

	/* Try to find the source by using the supplied "path" name */
	ret = open(path, O_RDWR);
	if (ret < 0) 
    {
		fprintf(stderr, "unable to open device \"%s\" (%m)\n", path);
		return ret;
	}

	/* Open the PPS source (and check the file descriptor) */
	ret = time_pps_create(ret, handle);
	if (ret < 0) 
    {
		fprintf(stderr, "cannot create a PPS source from device "
				"\"%s\" (%m)\n", path);
		return -1;
	}
	printf("found PPS source \"%s\"\n", path);

	/* Find out what features are supported */
	ret = time_pps_getcap(*handle, avail_mode);
	if (ret < 0) 
    {
		fprintf(stderr, "cannot get capabilities (%m)\n");
		return -1;
	}
	if ((*avail_mode & PPS_CAPTUREASSERT) == 0) {
		fprintf(stderr, "cannot CAPTUREASSERT\n");
		return -1;
	}

	/* Capture assert timestamps */
	ret = time_pps_getparams(*handle, &params);
	if (ret < 0) 
    {
		fprintf(stderr, "cannot get parameters (%m)\n");
		return -1;
	}
	params.mode |= PPS_CAPTUREASSERT;
	/* Override any previous offset if possible */
	if ((*avail_mode & PPS_OFFSETASSERT) != 0) {
		params.mode |= PPS_OFFSETASSERT;
		params.assert_offset = offset_assert;
	}
	ret = time_pps_setparams(*handle, &params);
	if (ret < 0) 
    {
		fprintf(stderr, "cannot set parameters (%m)\n");
		return -1;
	}

	return 0;
}

int fetch_source(int i, pps_handle_t *handle, int *avail_mode)
{
	struct timespec timeout;
	pps_info_t infobuf;
	static pps_info_t last_infobuf;
	int ret;

	/* create a zero-valued timeout */
	timeout.tv_sec = 3;
	timeout.tv_nsec = 0;

retry:
    // 等待事件触发
	if (*avail_mode & PPS_CANWAIT) 
		ret = time_pps_fetch(*handle, PPS_TSFMT_TSPEC, &infobuf, &timeout);
	else 
    {
		sleep(1);
		ret = time_pps_fetch(*handle, PPS_TSFMT_TSPEC, &infobuf, &timeout);
	}

	if (ret < 0) 
    {
        
		if (ret == -EINTR) 
        {
			fprintf(stderr, "time_pps_fetch() got a signal!\n");
			goto retry;
		}

		fprintf(stderr, "time_pps_fetch() error %d (%m)\n", ret);
		return -1;
	}

	// 触发成功
	flag = true;
	printf("source %d - "
	       "assert %lld.%09ld, sequence: %ld - "
	       "clear  %lld.%09ld, sequence: %ld\n",
	       i,
	       (long long)infobuf.assert_timestamp.tv_sec,
	       infobuf.assert_timestamp.tv_nsec,
	       infobuf.assert_sequence,
	       (long long)infobuf.clear_timestamp.tv_sec,
	       infobuf.clear_timestamp.tv_nsec, infobuf.clear_sequence);
	fflush(stdout);

	// 记录数据
	pps.name = "SYS_TIME";
	pps.count = last_infobuf.assert_sequence;
	pps.number = 1;
	pps.fpga_pps_sys_time.sec = last_infobuf.assert_timestamp.tv_sec;
	pps.fpga_pps_sys_time.nsec = last_infobuf.assert_timestamp.tv_nsec;
	last_infobuf = infobuf;
	return 0;
}

// FPGA 回调函数
void FPGA_Callback(const byh_uav::uav_fpga_time::ConstPtr& msg)
{
	if( flag == true )
	{
		// pps.header.stamp = ros::Time::now(); 
		pps.header.stamp = msg->header.stamp; 
		pps.header.frame_id = "All_Time_Sync"; 

		pps.fpga_pps_fpga_time = msg->fpga_pps_fpga_time;
		pps.fpga_pps_mcu_time = msg->fpga_pps_mcu_time;
		pps.gps_pps_fpga_time = msg->gps_pps_fpga_time;
		pps.gps_pps_time = msg->gps_pps_time;
		pps_publisher.publish(pps);
		flag = false;
	}
}

void usage(char *name)
{
	fprintf(stderr, "usage: %s <ppsdev> [<ppsdev> ...]\n", name);
	exit(EXIT_FAILURE);
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
    ros::init(argc, argv, "byh_uav_pps"); 

    // 声明ros句柄
    ros::NodeHandle n;
    
    // 创建话题发布者
    pps_publisher = n.advertise<byh_uav::uav_pps_all>("byh_uav/pps_sys", 20); 

	// 创建话题订阅者
    ros::Subscriber FPGA_sub = n.subscribe("/byh_uav/FPGA", 20, FPGA_Callback);
   
    int num;
	pps_handle_t handle[4];
	int avail_mode[4];
	int i = 0;
	int ret;

	// 检查输入参数
	if (argc < 2)
		usage(argv[0]);

	for (i = 1; i < argc && i <= 4; i++) {
		ret = find_source(argv[i], &handle[i - 1], &avail_mode[i - 1]);
		if (ret < 0)
			exit(EXIT_FAILURE);
	}

	num = i - 1;
	printf("ok, found %d source(s), now start fetching data...\n", num);


    while(ros::ok())
    {
        for (i = 0; i < num; i++) 
        {
			ret = fetch_source(i, &handle[i], &avail_mode[i]);
			if (ret < 0 && errno != ETIMEDOUT)
				exit(EXIT_FAILURE);
		}
		ros::spinOnce();
    }
    
    for (; i >= 0; i--)
		time_pps_destroy(handle[i]);

	return 0;
} 
