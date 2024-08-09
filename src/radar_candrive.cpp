#include "radar_ros/radar_candrive.h"


using namespace std;

radar_candrive::radar_candrive(){
    nh.param<std::string>("object_can0_id0_topic",radar_candrive::object_can0_id0_topic,"/ars_40X/objects_front");
    nh.param<std::string>("object_can0_id1_topic",radar_candrive::object_can0_id1_topic,"/ars_40X/objects_back");//接收毫米波雷达话题
    nh.param<std::string>("object_can1_id0_topic",radar_candrive::object_can1_id0_topic,"/ars_40X/objects_left");
    nh.param<std::string>("object_can1_id1_topic",radar_candrive::object_can1_id1_topic,"/ars_40X/objects_right");

    nh.param<std::string>("cluster_can0_id0_topic",radar_candrive::cluster_can0_id0_topic,"/ars_40X/clusters_front");
    nh.param<std::string>("cluster_can0_id1_topic",radar_candrive::cluster_can0_id1_topic,"/ars_40X/clusters_back");//接收毫米波雷达话题
    nh.param<std::string>("cluster_can1_id0_topic",radar_candrive::cluster_can1_id0_topic,"/ars_40X/clusters_left");
    nh.param<std::string>("cluster_can1_id1_topic",radar_candrive::cluster_can1_id1_topic,"/ars_40X/clusters_right");

    nh.param<std::string>("odom_topic",radar_candrive::odom_topic,"/autoware_tracker/Odom");//接收自车信息，输入前向毫米波雷达
    int can0, can1;
    while(can0 =Can0Init()  < 0){
	    if (can0 =Can0Init() > 0){
	        break;
	    }
	    printf("继续bind Can0\n");
	    sleep(1);
	}
    while(can1 =Can1Init()  < 0){
	    if (can1 =Can1Init() > 0){
	        break;
	    }
	    printf("继续bind Can1\n");
	    sleep(1);
	}
    std::cout<<"初始化can口成功\n"<<std::endl;    
}

radar_candrive::~radar_candrive(){
    if (close(can_fd_1) < 0) {
        perror("Closing: ");
        printf("Error: %d", errno);
    }
    if (close(can_fd_2) < 0) {
        perror("Closing: ");
        printf("Error: %d", errno);
    }

}

int radar_candrive::Can0Init(){

    struct sockaddr_can addr1;
    struct ifreq ifr1{};
    struct can_frame frame1;
    struct can_filter rfilter1[1];

    can_fd_1 = socket(PF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
    if(can_fd_1 < 0 ){
        std::cout<<"socket can0 creat error!\n"<<std::endl;
        return -1;
    }

    strcpy(ifr1.ifr_name, "can1" );
    ioctl(can_fd_1, SIOCGIFINDEX, &ifr1); //指定 can0 设备
    addr1.can_family = AF_CAN;
    addr1.can_ifindex = ifr1.ifr_ifindex;
    int bind_res = bind(can_fd_1, (struct sockaddr *)&addr1, sizeof(addr1)); //将套接字与 can0 绑定
    if(bind_res < 0){
        std::cout<<"bind can0 error!\n"<<std::endl;
        return -1;
    }

    int error = 0;
    socklen_t len = sizeof (error);
    int retval = getsockopt (can_fd_1, SOL_SOCKET, SO_ERROR, &error, &len);
    if (retval != 0) {
        /* there was a problem getting the error code */
        printf("Error getting socket error code: %s\n", strerror(retval));
        return -1;
    }

    if (error != 0) {
        /* socket has a non zero error status */
        printf("Socket error: %s\n", strerror(error));
        return -1;
    }

    long timeout_ = 1000l;
    struct timeval timeout{};
    timeout.tv_sec = (timeout_ / 1000);
    timeout.tv_usec = (timeout_ % 1000) * 1000;

    if (setsockopt(can_fd_1, SOL_SOCKET, SO_RCVTIMEO, (char *) & timeout, sizeof(timeout)) < 0) {
        perror("Setting timeout failed");
    }

    //setsockopt(can_fd_1, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter1, sizeof(rfilter1));
    return can_fd_1;
}

int radar_candrive::Can1Init(){

    struct sockaddr_can addr2;
    struct ifreq ifr2{};
    struct can_frame frame2;
    struct can_filter rfilter2[1];

    can_fd_2 = socket(PF_CAN, SOCK_RAW, CAN_RAW); //创建套接字
    if(can_fd_2 < 0 ){
        std::cout<<"socket can1 creat error!\n"<<std::endl;
        return -1;
    }

    strcpy(ifr2.ifr_name, "can0" );
    ioctl(can_fd_2, SIOCGIFINDEX, &ifr2); //指定 can0 设备
    addr2.can_family = AF_CAN;
    addr2.can_ifindex = ifr2.ifr_ifindex;
    int bind_res = bind(can_fd_2, (struct sockaddr *)&addr2, sizeof(addr2)); //将套接字与 can0 绑定
    if(bind_res < 0){
        std::cout<<"bind can1 error!\n"<<std::endl;
        return -1;
    }

    int error = 0;
    socklen_t len = sizeof (error);
    int retval = getsockopt (can_fd_2, SOL_SOCKET, SO_ERROR, &error, &len);
    if (retval != 0) {
        /* there was a problem getting the error code */
        printf("Error getting socket error code: %s\n", strerror(retval));
        return -1;
    }

    if (error != 0) {
        /* socket has a non zero error status */
        printf("Socket error: %s\n", strerror(error));
        return -1;
    }
    long timeout_ = 1000l;
    struct timeval timeout{};
    timeout.tv_sec = (timeout_ / 1000);
    timeout.tv_usec = (timeout_ % 1000) * 1000;

    if (setsockopt(can_fd_2, SOL_SOCKET, SO_RCVTIMEO, (char *) & timeout, sizeof(timeout)) < 0) {
        perror("Setting timeout failed");
    }
    //setsockopt(can_fd_2, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter2, sizeof(rfilter2));
    return can_fd_2;
}


void radar_candrive::run(){

    object_can0_id0 = nh.advertise<radar_ros::ObjectList>(object_can0_id0_topic,1);
    object_can0_id1 = nh.advertise<radar_ros::ObjectList>(object_can0_id1_topic,1);
    object_can1_id0 = nh.advertise<radar_ros::ObjectList>(object_can1_id0_topic,1);
    object_can1_id1 = nh.advertise<radar_ros::ObjectList>(object_can1_id1_topic,1);

    cluster_can0_id0 = nh.advertise<radar_ros::ClusterList>(cluster_can0_id0_topic,1);
    cluster_can0_id1 = nh.advertise<radar_ros::ClusterList>(cluster_can0_id1_topic,1);
    cluster_can1_id0 = nh.advertise<radar_ros::ClusterList>(cluster_can1_id0_topic,1);
    cluster_can1_id1 = nh.advertise<radar_ros::ClusterList>(cluster_can1_id1_topic,1);

    std::thread t1(std::bind(&radar_candrive::RecvCanMsgThread1, this));/*创建CAN接收线程*/
    t1.detach();/*分离线程*/
    std::thread t2(std::bind(&radar_candrive::RecvCanMsgThread2, this));/*创建CAN接收线程*/
    t2.detach();/*分离线程*/

    odom_sub = nh.subscribe(odom_topic,1,&radar_candrive::odom_callback, this);

}

void radar_candrive::odom_callback(const nav_msgs::Odometry::ConstPtr &odom){
    double speed = odom->twist.twist.linear.x;
    // 将速度值转换为整数形式
    int radar_speed = static_cast<int>(std::abs(speed) / 0.02);
    speed_information speed_information_msg;
    speed_information_msg.data.RadarDevice_Speed1 = static_cast<uint64_t>((radar_speed >> 8) & 0x1F);
    speed_information_msg.data.RadarDevice_Speed2 = static_cast<uint64_t>(radar_speed & 255);

    if (speed < 0.0) {
        speed_information_msg.data.RadarDevice_SpeedDirection = RadarDevice_SpeedDirection::BACKWARD;
    } else if (speed > 0.0) {
        speed_information_msg.data.RadarDevice_SpeedDirection = RadarDevice_SpeedDirection::FORWARD;
    } else {
        speed_information_msg.data.RadarDevice_SpeedDirection = RadarDevice_SpeedDirection::STANDSTILL;
    }

    // 创建一个CAN帧
    struct can_frame frame_SpeedInformation;
    frame_SpeedInformation.can_id = 0x300;
    frame_SpeedInformation.can_dlc = 2; // 数据长度设为2字节

    // 清空数据字段
    memset(frame_SpeedInformation.data, 0, sizeof(frame_SpeedInformation.data));

    // 填充RadarDevice_Speed到数据字段中
    frame_SpeedInformation.data[0] = speed_information_msg.raw_data[0];
    frame_SpeedInformation.data[1] = speed_information_msg.raw_data[1];     

    // 打印 frame[0] 和 frame[1] 的内容
    printf("frame_SpeedInformation[0]: 0x%02X\n", frame_SpeedInformation.data[0]);  // 以十六进制形式打印
    printf("frame_SpeedInformation[1]: 0x%02X\n", frame_SpeedInformation.data[1]);  // 以十六进制形式打印

    // 发送CAN帧
    if (write(can_fd_1, &frame_SpeedInformation, sizeof(frame_SpeedInformation)) != sizeof(frame_SpeedInformation)) {
        perror("Write");
    }

    double yaw_rate = odom->twist.twist.angular.z * 180.0 / M_PI;

    int radar_yaw_rate = static_cast<int>((yaw_rate + 327.68) / 0.01);

    yaw_rate_information yaw_rate_information_msg;

    yaw_rate_information_msg.data.RadarDevice_YawRate1 = static_cast<uint64_t>(radar_yaw_rate >> 8);
    yaw_rate_information_msg.data.RadarDevice_YawRate2 = static_cast<uint64_t>(radar_yaw_rate & 255);

    // 创建一个CAN帧
    struct can_frame frame_YawRate;
    frame_YawRate.can_id = 0x301;
    frame_YawRate.can_dlc = 2; // 数据长度设为2字节

    // 清空数据字段
    memset(frame_YawRate.data, 0, sizeof(frame_YawRate.data));

    frame_YawRate.data[0] = yaw_rate_information_msg.raw_data[0];
    frame_YawRate.data[1] = yaw_rate_information_msg.raw_data[1];

    // 打印 frame[0] 和 frame[1] 的内容
    printf("frame_YawRate[0]: 0x%02X\n", frame_YawRate.data[0]);  // 以十六进制形式打印
    printf("frame_YawRate[1]: 0x%02X\n", frame_YawRate.data[1]);  // 以十六进制形式打印

    // 发送CAN帧
    if (write(can_fd_1, &frame_YawRate, sizeof(frame_YawRate)) != sizeof(frame_YawRate)) {
        perror("Write");
    }

}


bool radar_candrive::RecvCanMsgThread1(){

    int nbytes = 0;
    int ObjNum = 0;
    int object_2_quality_id_1= 0;
    int object_3_extended_id_1 = 0;
    int object_2_quality_id_2= 0;
    int object_3_extended_id_2 = 0;
    radar_ros::ObjectList Object1List;
    radar_ros::ObjectList Object2List;

    radar_ros::ClusterList Cluster1List;
    radar_ros::ClusterList Cluster2List;

    while (ros::ok())
    {
        ROS_INFO("start_can0");
        struct can_frame frame{};
        do {
            nbytes = read(can_fd_1, &frame, sizeof(struct can_frame));
        }while(nbytes == 0); //接收报文
        if(nbytes > 0)
        {
            radar_ros::Object RadarObj1;
            radar_ros::Object RadarObj2;

            radar_ros::Cluster RadarClu1;
            radar_ros::Cluster RadarClu2;
            if(frame.can_id == 0x60A) {// status
		        Object1List.header.stamp = ros::Time::now();
		        Object1List.header.frame_id = "rslidar";
                object_2_quality_id_1 = 0;
                object_3_extended_id_1 = 0;
                object_can0_id0.publish(Object1List);
		        Object1List.objects.clear();
            } else if(frame.can_id == 0x60B) {//objects
		        //ID
		        int ID = frame.data[0];
                //X坐标
		        float DistLat = (((0x07 & frame.data[2]) << 8) + frame.data[3]) * 0.2 - 204.6;
                //Y坐标
                float DistLong = ((frame.data[1] << 5) + ((frame.data[2] >> 3) & 0x1f)) * 0.2 - 500;	
                //速度X
                float VrelLat = (((frame.data[5] & 0x3f) << 3) + ((frame.data[6] >> 5) & 0x07)) * 0.25 - 64;
                //速度Y
                float VrelLong = ((frame.data[4] << 2) + ((frame.data[5] >> 6) & 0x03)) * 0.25 - 128;
		        //坐标变换
		        float x = DistLong;
                float y = DistLat;
		        float vx = VrelLong;
                float vy = VrelLat;
                //目标状态
		        int DynProp = (frame.data[6] & 0x07);
                //目标类型
                int Class = (frame.data[6] & 0x18) >> 3;
                //目标反射横截面积
                float RSC = (frame.data[7]) * 0.5 - 64;
                RadarObj1.id = ID;
                RadarObj1.position.pose.position.x = x;
                RadarObj1.position.pose.position.y = y;
                RadarObj1.position.pose.position.z = 1;
                RadarObj1.relative_velocity.twist.linear.x = vx;
                RadarObj1.relative_velocity.twist.linear.y = vy;
                RadarObj1.relative_velocity.twist.linear.z = 0;
                RadarObj1.dynamic_property = DynProp;
                RadarObj1.rcs = RSC;
                Object1List.objects.push_back(RadarObj1);
            } else if(frame.can_id == 0x60C) {//objects
                int prob_of_exist = (frame.data[6] >> 5)&0x07;
                int meas_state = (frame.data[6] >> 2)&0x07;
		        if(object_2_quality_id_1+1 <= Object1List.objects.size()){
                    Object1List.objects[object_2_quality_id_1].prob_of_exist = prob_of_exist;
                    Object1List.objects[object_2_quality_id_1].meas_state = meas_state;
                    object_2_quality_id_1++;
		        }
            } else if(frame.can_id == 0x60D) {//objects
                float length = frame.data[6] *0.2;
                float width = frame.data[7] *0.2;
                float orientation_angle = ((frame.data[4]<<2)+((frame.data[5]>>6) & 0x03))*0.4-180.0;
                int class_type = frame.data[3]&0x07;

                double half_yaw = (orientation_angle * M_PI / 180.0) / 2.0;
                double w = std::cos(half_yaw);
                double x = 0.0;
                double y = 0.0;
                double z = std::sin(half_yaw);
                // 构造四元数
                Eigen::Quaterniond q(w, x, y, z);
		        if(object_3_extended_id_1+1 <= Object1List.objects.size()){
                    Object1List.objects[object_3_extended_id_1].position.pose.orientation.w = q.w();
                    Object1List.objects[object_3_extended_id_1].position.pose.orientation.x = q.x();
                    Object1List.objects[object_3_extended_id_1].position.pose.orientation.y = q.y();
                    Object1List.objects[object_3_extended_id_1].position.pose.orientation.z = q.z();
                    Object1List.objects[object_3_extended_id_1].length = length;
                    Object1List.objects[object_3_extended_id_1].width = width;
                    Object1List.objects[object_3_extended_id_1].orientation_angle = orientation_angle;
                    Object1List.objects[object_3_extended_id_1].class_type = class_type;
                    object_3_extended_id_1++;
		        }
            } else if(frame.can_id == 0x600) {// status
                Cluster1List.header.stamp = ros::Time::now();
                Cluster1List.header.frame_id = "rslidar";
                cluster_can0_id0.publish(Cluster1List);
                Cluster1List.clusters.clear();
            } else if(frame.can_id == 0x701) {// status
		        //ID
		        int ID = frame.data[0];
                //Y坐标
		        float DistLat = (((0x03 & frame.data[2]) << 8) + frame.data[3]) * 0.2 - 102.3;
                //X坐标
                float DistLong = ((frame.data[1] << 5) + ((frame.data[2] >> 3) & 0x1f)) * 0.2 - 500;	
                //速度Y
                float VrelLat = (((frame.data[5] & 0x3f) << 3) + ((frame.data[6] >> 5) & 0x07)) * 0.25 - 64;
                //速度X
                float VrelLong = ((frame.data[4] << 2) + ((frame.data[5] >> 6) & 0x03)) * 0.25 - 128;
		        //坐标变换
		        float y = DistLat;
                float x = DistLong;
		        float vy = VrelLat;
                float vx = VrelLong;
                //目标反射横截面积
                float RSC = (frame.data[7]) * 0.5 - 64;
                RadarClu1.id = ID;
                RadarClu1.position.pose.position.x = x;
                RadarClu1.position.pose.position.y = y;
                RadarClu1.position.pose.position.z = 1;
                RadarClu1.relative_velocity.twist.linear.x = vx;
                RadarClu1.relative_velocity.twist.linear.y = vy;
                RadarClu1.relative_velocity.twist.linear.z = 0;
                RadarClu1.rcs = RSC;
                Cluster1List.clusters.push_back(RadarClu1);
            } else if(frame.can_id == 0x61A) {// status
		        Object2List.header.stamp = ros::Time::now();
		        Object2List.header.frame_id = "rslidar";
                object_2_quality_id_2 = 0;
                object_3_extended_id_2 = 0;
                object_can0_id1.publish(Object2List);
		        Object2List.objects.clear();
            } else if(frame.can_id == 0x61B) {//objects
		        //ID
		        int ID = frame.data[0];
                //X坐标
		        float DistLat = (((0x07 & frame.data[2]) << 8) + frame.data[3]) * 0.2 - 204.6;
                //Y坐标
                float DistLong = ((frame.data[1] << 5) + ((frame.data[2] >> 3) & 0x1f)) * 0.2 - 500;
		
                //速度X
                float VrelLat = (((frame.data[5] & 0x3f) << 3) + ((frame.data[6] >> 5) & 0x07)) * 0.25 - 64;
                //速度Y
                float VrelLong = ((frame.data[4] << 2) + ((frame.data[5] >> 6) & 0x03)) * 0.25 - 128;

		        //坐标变换
		        float x = DistLong;
                float y = DistLat;
		        float vx = VrelLong;
                float vy = VrelLat;

                //目标状态
		        int DynProp = (frame.data[6] & 0x07);
                //目标类型
                int Class = (frame.data[6] & 0x18) >> 3;
                //目标反射横截面积
                float RSC = (frame.data[7]) * 0.5 - 64;
                RadarObj2.id = ID;
                RadarObj2.position.pose.position.x = x;
                RadarObj2.position.pose.position.y = y;
                RadarObj2.position.pose.position.z = 1;
                RadarObj2.relative_velocity.twist.linear.x = vx;
                RadarObj2.relative_velocity.twist.linear.y = vy;
                RadarObj2.relative_velocity.twist.linear.z = 0;
                RadarObj2.dynamic_property = DynProp;
                RadarObj2.rcs = RSC;
                Object2List.objects.push_back(RadarObj2);
            } else if(frame.can_id == 0x61C) {//objects
                int prob_of_exist = (frame.data[6] >> 5)&0x07;
                int meas_state = (frame.data[6] >> 2)&0x07;
		        if(object_2_quality_id_2+1 <= Object2List.objects.size()){
                    Object2List.objects[object_2_quality_id_2].prob_of_exist = prob_of_exist;
                    Object2List.objects[object_2_quality_id_2].meas_state = meas_state;
                    object_2_quality_id_2++;
		        }
            } else if(frame.can_id == 0x61D) {//objects
                float length = frame.data[6] *0.2;
                float width = frame.data[7] *0.2;
                float orientation_angle = ((frame.data[4]<<2)+((frame.data[5]>>6) & 0x03))*0.4-180.0;
                int class_type = frame.data[3]&0x07;
                double half_yaw = (orientation_angle * M_PI / 180.0) / 2.0;
                double w = std::cos(half_yaw);
                double x = 0.0;
                double y = 0.0;
                double z = std::sin(half_yaw);
                // 构造四元数
                Eigen::Quaterniond q(w, x, y, z);
		        if(object_3_extended_id_2+1 <= Object2List.objects.size()){
                    Object2List.objects[object_3_extended_id_2].position.pose.orientation.w = q.w();
                    Object2List.objects[object_3_extended_id_2].position.pose.orientation.x = q.x();
                    Object2List.objects[object_3_extended_id_2].position.pose.orientation.y = q.y();
                    Object2List.objects[object_3_extended_id_2].position.pose.orientation.z = q.z();
                    Object2List.objects[object_3_extended_id_2].length = length;
                    Object2List.objects[object_3_extended_id_2].width = width;
                    Object2List.objects[object_3_extended_id_2].orientation_angle = orientation_angle;
                    Object2List.objects[object_3_extended_id_2].class_type = class_type;
                    object_3_extended_id_2++;
		        }
            } else if(frame.can_id == 0x610) {// status
                Cluster2List.header.stamp = ros::Time::now();
                Cluster2List.header.frame_id = "rslidar";
                cluster_can0_id1.publish(Cluster2List);
                Cluster2List.clusters.clear();
            } else if(frame.can_id == 0x711) {// status
		        //ID
		        int ID = frame.data[0];
                //Y坐标
		        float DistLat = (((0x03 & frame.data[2]) << 8) + frame.data[3]) * 0.2 - 102.3;
                //X坐标
                float DistLong = ((frame.data[1] << 5) + ((frame.data[2] >> 3) & 0x1f)) * 0.2 - 500;	
                //速度Y
                float VrelLat = (((frame.data[5] & 0x3f) << 3) + ((frame.data[6] >> 5) & 0x07)) * 0.25 - 64;
                //速度X
                float VrelLong = ((frame.data[4] << 2) + ((frame.data[5] >> 6) & 0x03)) * 0.25 - 128;
		        //坐标变换
		        float y = DistLat;
                float x = DistLong;
		        float vy = VrelLat;
                float vx = VrelLong;
                //目标反射横截面积
                float RSC = (frame.data[7]) * 0.5 - 64;
                RadarClu2.id = ID;
                RadarClu2.position.pose.position.x = x;
                RadarClu2.position.pose.position.y = y;
                RadarClu2.position.pose.position.z = 1;
                RadarClu2.relative_velocity.twist.linear.x = vx;
                RadarClu2.relative_velocity.twist.linear.y = vy;
                RadarClu2.relative_velocity.twist.linear.z = 0;
                RadarClu2.rcs = RSC;
                Cluster2List.clusters.push_back(RadarClu2);
             }else {
		        ROS_INFO("CANID=0x%X THROW EXCEPTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", frame.can_id);
               //return 1;
	        }
        } 
        else{
            std::cout<<"没有接收到数据"<<std::endl;
        }
    }
    return true;
}

bool radar_candrive::RecvCanMsgThread2(){

    int nbytes = 0;
    int ObjNum = 0;
    int object_2_quality_id_1= 0;
    int object_3_extended_id_1 = 0;
    int object_2_quality_id_2= 0;
    int object_3_extended_id_2 = 0;
    radar_ros::ObjectList Object1List;
    radar_ros::ObjectList Object2List;

    radar_ros::ClusterList Cluster1List;
    radar_ros::ClusterList Cluster2List;

    while (ros::ok())
    {
        ROS_INFO("start_can1");
        struct can_frame frame{};
        do {
            nbytes = read(can_fd_2, &frame, sizeof(struct can_frame));
        }while(nbytes == 0); //接收报文
        if(nbytes > 0)
        {
            radar_ros::Object RadarObj1;
            radar_ros::Object RadarObj2;

            radar_ros::Cluster RadarClu1;
            radar_ros::Cluster RadarClu2;
            if(frame.can_id == 0x60A) {// status
		        Object1List.header.stamp = ros::Time::now();
		        Object1List.header.frame_id = "rslidar";
                object_2_quality_id_1 = 0;
                object_3_extended_id_1 = 0;
                object_can1_id0.publish(Object1List);
		        Object1List.objects.clear();
            } else if(frame.can_id == 0x60B) {//objects
		        //ID
		        int ID = frame.data[0];
                //X坐标
		        float DistLat = (((0x07 & frame.data[2]) << 8) + frame.data[3]) * 0.2 - 204.6;
                //Y坐标
                float DistLong = ((frame.data[1] << 5) + ((frame.data[2] >> 3) & 0x1f)) * 0.2 - 500;	
                //速度X
                float VrelLat = (((frame.data[5] & 0x3f) << 3) + ((frame.data[6] >> 5) & 0x07)) * 0.25 - 64;
                //速度Y
                float VrelLong = ((frame.data[4] << 2) + ((frame.data[5] >> 6) & 0x03)) * 0.25 - 128;
		        //坐标变换
		        float x = DistLong;
                float y = DistLat;
		        float vx = VrelLong;
                float vy = VrelLat;
                //目标状态
		        int DynProp = (frame.data[6] & 0x07);
                //目标类型
                int Class = (frame.data[6] & 0x18) >> 3;
                //目标反射横截面积
                float RSC = (frame.data[7]) * 0.5 - 64;
                RadarObj1.id = ID;
                RadarObj1.position.pose.position.x = x;
                RadarObj1.position.pose.position.y = y;
                RadarObj1.position.pose.position.z = 1;
                RadarObj1.relative_velocity.twist.linear.x = vx;
                RadarObj1.relative_velocity.twist.linear.y = vy;
                RadarObj1.relative_velocity.twist.linear.z = 0;
                RadarObj1.dynamic_property = DynProp;
                RadarObj1.rcs = RSC;
                Object1List.objects.push_back(RadarObj1);
            } else if(frame.can_id == 0x60C) {//objects
                int prob_of_exist = (frame.data[6] >> 5)&0x07;
                int meas_state = (frame.data[6] >> 2)&0x07;
		        if(object_2_quality_id_1+1 <= Object1List.objects.size()){
                    Object1List.objects[object_2_quality_id_1].prob_of_exist = prob_of_exist;
                    Object1List.objects[object_2_quality_id_1].meas_state = meas_state;
                    object_2_quality_id_1++;
		        }
            } else if(frame.can_id == 0x60D) {//objects
                float length = frame.data[6] *0.2;
                float width = frame.data[7] *0.2;
                float orientation_angle = ((frame.data[4]<<2)+((frame.data[5]>>6) & 0x03))*0.4-180.0;
                int class_type = frame.data[3]&0x07;
                double half_yaw = (orientation_angle * M_PI / 180.0) / 2.0;
                double w = std::cos(half_yaw);
                double x = 0.0;
                double y = 0.0;
                double z = std::sin(half_yaw);
                // 构造四元数
                Eigen::Quaterniond q(w, x, y, z);
		        if(object_3_extended_id_1+1 <= Object1List.objects.size()){
                    Object1List.objects[object_3_extended_id_1].position.pose.orientation.w = q.w();
                    Object1List.objects[object_3_extended_id_1].position.pose.orientation.x = q.x();
                    Object1List.objects[object_3_extended_id_1].position.pose.orientation.y = q.y();
                    Object1List.objects[object_3_extended_id_1].position.pose.orientation.z = q.z();
                    Object1List.objects[object_3_extended_id_1].length = length;
                    Object1List.objects[object_3_extended_id_1].width = width;
                    Object1List.objects[object_3_extended_id_1].orientation_angle = orientation_angle;
                    Object1List.objects[object_3_extended_id_1].class_type = class_type;
                    object_3_extended_id_1++;
		        }
            } else if(frame.can_id == 0x600) {// status
                Cluster1List.header.stamp = ros::Time::now();
                Cluster1List.header.frame_id = "rslidar";
                cluster_can1_id0.publish(Cluster1List);
                Cluster1List.clusters.clear();
            } else if(frame.can_id == 0x701) {// status
		        //ID
		        int ID = frame.data[0];
                //Y坐标
		        float DistLat = (((0x03 & frame.data[2]) << 8) + frame.data[3]) * 0.2 - 102.3;
                //X坐标
                float DistLong = ((frame.data[1] << 5) + ((frame.data[2] >> 3) & 0x1f)) * 0.2 - 500;	
                //速度Y
                float VrelLat = (((frame.data[5] & 0x3f) << 3) + ((frame.data[6] >> 5) & 0x07)) * 0.25 - 64;
                //速度X
                float VrelLong = ((frame.data[4] << 2) + ((frame.data[5] >> 6) & 0x03)) * 0.25 - 128;
		        //坐标变换
		        float y = DistLat;
                float x = DistLong;
		        float vy = VrelLat;
                float vx = VrelLong;
                //目标反射横截面积
                float RSC = (frame.data[7]) * 0.5 - 64;
                RadarClu1.id = ID;
                RadarClu1.position.pose.position.x = x;
                RadarClu1.position.pose.position.y = y;
                RadarClu1.position.pose.position.z = 1;
                RadarClu1.relative_velocity.twist.linear.x = vx;
                RadarClu1.relative_velocity.twist.linear.y = vy;
                RadarClu1.relative_velocity.twist.linear.z = 0;
                RadarClu1.rcs = RSC;
                Cluster1List.clusters.push_back(RadarClu1);
            } else if(frame.can_id == 0x61A) {// status
		        Object2List.header.stamp = ros::Time::now();
		        Object2List.header.frame_id = "rslidar";
                object_2_quality_id_2 = 0;
                object_3_extended_id_2 = 0;
                object_can1_id1.publish(Object2List);
		        Object2List.objects.clear();
            } else if(frame.can_id == 0x61B) {//objects
		        //ID
		        int ID = frame.data[0];
                //X坐标
		        float DistLat = (((0x07 & frame.data[2]) << 8) + frame.data[3]) * 0.2 - 204.6;
                //Y坐标
                float DistLong = ((frame.data[1] << 5) + ((frame.data[2] >> 3) & 0x1f)) * 0.2 - 500;
		
                //速度X
                float VrelLat = (((frame.data[5] & 0x3f) << 3) + ((frame.data[6] >> 5) & 0x07)) * 0.25 - 64;
                //速度Y
                float VrelLong = ((frame.data[4] << 2) + ((frame.data[5] >> 6) & 0x03)) * 0.25 - 128;

		        //坐标变换
		        float x = DistLong;
                float y = DistLat;
		        float vx = VrelLong;
                float vy = VrelLat;

                //目标状态
		        int DynProp = (frame.data[6] & 0x07);
                //目标类型
                int Class = (frame.data[6] & 0x18) >> 3;
                //目标反射横截面积
                float RSC = (frame.data[7]) * 0.5 - 64;
                RadarObj2.id = ID;
                RadarObj2.position.pose.position.x = x;
                RadarObj2.position.pose.position.y = y;
                RadarObj2.position.pose.position.z = 1;
                RadarObj2.relative_velocity.twist.linear.x = vx;
                RadarObj2.relative_velocity.twist.linear.y = vy;
                RadarObj2.relative_velocity.twist.linear.z = 0;
                RadarObj2.dynamic_property = DynProp;
                RadarObj2.rcs = RSC;
                Object2List.objects.push_back(RadarObj2);
            } else if(frame.can_id == 0x61C) {//objects
                int prob_of_exist = (frame.data[6] >> 5)&0x07;
                int meas_state = (frame.data[6] >> 2)&0x07;
		        if(object_2_quality_id_2+1 <= Object2List.objects.size()){
                    Object2List.objects[object_2_quality_id_2].prob_of_exist = prob_of_exist;
                    Object2List.objects[object_2_quality_id_2].meas_state = meas_state;
                    object_2_quality_id_2++;
		        }
            } else if(frame.can_id == 0x61D) {//objects
                float length = frame.data[6] *0.2;
                float width = frame.data[7] *0.2;
                float orientation_angle = ((frame.data[4]<<2)+((frame.data[5]>>6) & 0x03))*0.4-180.0;
                int class_type = frame.data[3]&0x07;
                double half_yaw = (orientation_angle * M_PI / 180.0) / 2.0;
                double w = std::cos(half_yaw);
                double x = 0.0;
                double y = 0.0;
                double z = std::sin(half_yaw);
                // 构造四元数
                Eigen::Quaterniond q(w, x, y, z);
		        if(object_3_extended_id_2+1 <= Object2List.objects.size()){
                    Object2List.objects[object_3_extended_id_2].position.pose.orientation.w = q.w();
                    Object2List.objects[object_3_extended_id_2].position.pose.orientation.x = q.x();
                    Object2List.objects[object_3_extended_id_2].position.pose.orientation.y = q.y();
                    Object2List.objects[object_3_extended_id_2].position.pose.orientation.z = q.z();
                    Object2List.objects[object_3_extended_id_2].length = length;
                    Object2List.objects[object_3_extended_id_2].width = width;
                    Object2List.objects[object_3_extended_id_2].orientation_angle = orientation_angle;
                    Object2List.objects[object_3_extended_id_2].class_type = class_type;
                    object_3_extended_id_2++;
		        }
            } else if(frame.can_id == 0x610) {// status
                Cluster2List.header.stamp = ros::Time::now();
                Cluster2List.header.frame_id = "rslidar";
                cluster_can1_id1.publish(Cluster2List);
                Cluster2List.clusters.clear();
            } else if(frame.can_id == 0x711) {// status
		        //ID
		        int ID = frame.data[0];
                //Y坐标
		        float DistLat = (((0x03 & frame.data[2]) << 8) + frame.data[3]) * 0.2 - 102.3;
                //X坐标
                float DistLong = ((frame.data[1] << 5) + ((frame.data[2] >> 3) & 0x1f)) * 0.2 - 500;	
                //速度Y
                float VrelLat = (((frame.data[5] & 0x3f) << 3) + ((frame.data[6] >> 5) & 0x07)) * 0.25 - 64;
                //速度X
                float VrelLong = ((frame.data[4] << 2) + ((frame.data[5] >> 6) & 0x03)) * 0.25 - 128;
		        //坐标变换
		        float y = DistLat;
                float x = DistLong;
		        float vy = VrelLat;
                float vx = VrelLong;
                //目标反射横截面积
                float RSC = (frame.data[7]) * 0.5 - 64;
                RadarClu2.id = ID;
                RadarClu2.position.pose.position.x = x;
                RadarClu2.position.pose.position.y = y;
                RadarClu2.position.pose.position.z = 1;
                RadarClu2.relative_velocity.twist.linear.x = vx;
                RadarClu2.relative_velocity.twist.linear.y = vy;
                RadarClu2.relative_velocity.twist.linear.z = 0;
                RadarClu2.rcs = RSC;
                Cluster2List.clusters.push_back(RadarClu2);
             }else {
		        ROS_INFO("CANID=0x%X THROW EXCEPTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", frame.can_id);
               //return 1;
	        }
        }
        else{
            std::cout<<"没有接收到数据"<<std::endl;
        }
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_candrive");  //初始化

    system(downcan0);//0路给的是：radar和vision
	system(commandcan0);
	system(upcan0);

    system(downcan1);//1路给的是：radar和vision
	system(commandcan1);
	system(upcan1);

    radar_candrive rc;
    rc.run();
    ros::spin();
    return 0;
}