#ifndef DYNAMIC_CALIB_H
#define DYNAMIC_CALIB_H

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <radar_ros/radar_rosConfig.h>
class dynamic_calib{
    private:
        ros::NodeHandle nh;

        ros::Subscriber sb_front;
        ros::Publisher pb_front;
        std::string sb_front_topic;

        ros::Subscriber sb_left;
        ros::Publisher pb_left;
        std::string sb_left_topic;

        ros::Subscriber sb_right;
        ros::Publisher pb_right;
        std::string sb_right_topic;

        ros::Subscriber sb_back;
        ros::Publisher pb_back;
        std::string sb_back_topic;

        float corMain_x;
        float corMain_y;
        float corMain_z;
        float corMain_roll;
        float corMain_pitch;
        float corMain_yaw;
        Eigen::Affine3f main_transform_ ;

        float corLeft_x;
        float corLeft_y;
        float corLeft_z;
        float corLeft_roll;
        float corLeft_pitch;
        float corLeft_yaw;
        Eigen::Affine3f left_transform_ ;

        float corRight_x;
        float corRight_y;
        float corRight_z;
        float corRight_roll;
        float corRight_pitch;
        float corRight_yaw;
        Eigen::Affine3f right_transform_ ;

        float corBack_x;
        float corBack_y;
        float corBack_z;
        float corBack_roll;
        float corBack_pitch;
        float corBack_yaw;
        Eigen::Affine3f back_transform_ ;

        std::string CSV_PATH;
        std::ofstream csv;

        //创建一个参数动态配置的服务器实例
        dynamic_reconfigure::Server<radar_ros::radar_rosConfig> server;
        //定义回调函数
        dynamic_reconfigure::Server<radar_ros::radar_rosConfig>::CallbackType f;
   
    public:
        dynamic_calib();
        ~dynamic_calib();
        void run();
        void msg_Callback_front(const visualization_msgs::MarkerArray::ConstPtr& markerarray);
        void msg_Callback_left(const visualization_msgs::MarkerArray::ConstPtr& markerarray);
        void msg_Callback_right(const visualization_msgs::MarkerArray::ConstPtr& markerarray);
        void msg_Callback_back(const visualization_msgs::MarkerArray::ConstPtr& markerarray);
        void dynamicParamCallback(radar_ros::radar_rosConfig &config);
};

#endif