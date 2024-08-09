#ifndef DYNAMIC_CALIB_H
#define DYNAMIC_CALIB_H

#include <ros/ros.h>
#include "radar_ros/Calib.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>

class dynamic_calib{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sb_param_front;
        ros::Subscriber sb_param_left;
        ros::Subscriber sb_param_right;
        ros::Subscriber sb_param_back;

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
        bool get_param_front;

        float corLeft_x;
        float corLeft_y;
        float corLeft_z;
        float corLeft_roll;
        float corLeft_pitch;
        float corLeft_yaw;
        Eigen::Affine3f left_transform_ ;
        bool get_param_left;

        float corRight_x;
        float corRight_y;
        float corRight_z;
        float corRight_roll;
        float corRight_pitch;
        float corRight_yaw;
        Eigen::Affine3f right_transform_ ;
        bool get_param_right;

        float corBack_x;
        float corBack_y;
        float corBack_z;
        float corBack_roll;
        float corBack_pitch;
        float corBack_yaw;
        Eigen::Affine3f back_transform_ ;
        bool get_param_back;

        std::string CSV_PATH;
        std::ofstream csv;
   
    public:
        dynamic_calib();
        ~dynamic_calib();
        void run();
        void msg_Callback_front(const visualization_msgs::MarkerArray::ConstPtr& markerarray);
        void msg_Callback_left(const visualization_msgs::MarkerArray::ConstPtr& markerarray);
        void msg_Callback_right(const visualization_msgs::MarkerArray::ConstPtr& markerarray);
        void msg_Callback_back(const visualization_msgs::MarkerArray::ConstPtr& markerarray);
        void msg_Callback_param_front(const radar_ros::Calib::ConstPtr& calib);
        void msg_Callback_param_left(const radar_ros::Calib::ConstPtr& calib);
        void msg_Callback_param_right(const radar_ros::Calib::ConstPtr& calib);
        void msg_Callback_param_back(const radar_ros::Calib::ConstPtr& calib);
};

#endif