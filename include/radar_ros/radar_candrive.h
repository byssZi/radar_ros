#ifndef RADAR_CANDRIVE_H
#define RADAR_CANDRIVE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include "radar_ros/ObjectList.h"
#include "radar_ros/Object.h"
#include "radar_ros/ClusterList.h"
#include "radar_ros/Cluster.h"
#include "Eigen/Dense"

#define downcan0 "sudo ip link set down can0"							//关闭CAN0
#define commandcan0 "sudo ip link set can0 type can bitrate 500000" //大陆ars408_radar,srr308的波特率为500Kbps
#define upcan0 "sudo ip link set up can0"									 //打开CAN0

#define downcan1 "sudo ip link set down can1"							//关闭CAN1
#define commandcan1 "sudo ip link set can1 type can bitrate 500000" //大陆ars408_radar,srr308的波特率为500Kbps
#define upcan1 "sudo ip link set up can1"									 //打开CAN1

typedef union speed_information {
  struct {
    uint64_t RadarDevice_Speed1:5;
    uint64_t Reserved:1;
    uint64_t RadarDevice_SpeedDirection:2;
    uint64_t RadarDevice_Speed2:8;
  } data = {};

  uint8_t raw_data[2];
} speed_information;

typedef enum RadarDevice_SpeedDirection {
  STANDSTILL = 0x0,
  FORWARD = 0x1,
  BACKWARD = 0x2,
} RadarDevice_SpeedDirection;

typedef union yaw_rate_information {
  struct {
    uint64_t RadarDevice_YawRate1:8;
    uint64_t RadarDevice_YawRate2:8;
  } data = {};

  uint8_t raw_data[2];
} yaw_rate_information;


class radar_candrive
{
private:

    ros::NodeHandle nh;
    ros::Publisher object_can0_id0;
    ros::Publisher object_can0_id1;
    ros::Publisher object_can1_id0;
    ros::Publisher object_can1_id1;
    std::string object_can0_id0_topic;
    std::string object_can0_id1_topic;
    std::string object_can1_id0_topic;
    std::string object_can1_id1_topic;

    ros::Publisher cluster_can0_id0;
    ros::Publisher cluster_can0_id1;
    ros::Publisher cluster_can1_id0;
    ros::Publisher cluster_can1_id1;
    std::string cluster_can0_id0_topic;
    std::string cluster_can0_id1_topic;
    std::string cluster_can1_id0_topic;
    std::string cluster_can1_id1_topic;    

    int can_fd_1, can_fd_2;

    ros::Subscriber odom_sub;
    std::string odom_topic;



public:

    radar_candrive();
    ~radar_candrive();
    void run();
    int Can0Init();
    int Can1Init();
    bool RecvCanMsgThread1();
    bool RecvCanMsgThread2();
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
};







#endif