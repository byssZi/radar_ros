#ifndef FILTER_H
#define FILTER_H

#include <ros/ros.h>
#include "radar_ros/ObjectList.h"
#include "radar_ros/ClusterList.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Core>
#include <Eigen/Dense>
enum {
  POINT,
  CAR,
  TRUCK,
  PEDESTRIAN,
  MOTORCYCLE,
  BICYCLE,
  WIDE,
  RESERVED
};

enum {
  INVALID,
  PERCENT_25,
  PERCENT_50,
  PERCENT_75,
  PERCENT_90,
  PERCENT_99,
  PERCENT_99_9,
  PERCENT_100
};



class filter{
    private:
      ros::NodeHandle nh; 
      ros::Subscriber sb_object_front;
      ros::Subscriber sb_cluster_front;
      ros::Publisher pb_object_front;
      ros::Publisher pb_object_marker_front;
      ros::Publisher pb_object_velocity_front;
      ros::Publisher pb_cluster_front;
      ros::Publisher pb_cluster_marker_front;
      std::string radar_sub_object_topic_front;
      std::string radar_pub_object_topic_front;
      std::string visualize_object_topic_front;
      std::string visualize_velocity_topic_front;
      std::string radar_sub_cluster_topic_front;
      std::string radar_pub_cluster_topic_front;
      std::string visualize_cluster_topic_front;
      double distance_front;
    

      ros::Subscriber sb_object_left;
      ros::Subscriber sb_cluster_left;
      ros::Publisher pb_object_left;
      ros::Publisher pb_object_marker_left;
      ros::Publisher pb_object_velocity_left;
      ros::Publisher pb_cluster_left;
      ros::Publisher pb_cluster_marker_left;
      std::string radar_sub_object_topic_left;
      std::string radar_pub_object_topic_left;
      std::string visualize_object_topic_left;
      std::string visualize_velocity_topic_left;
      std::string radar_sub_cluster_topic_left;
      std::string radar_pub_cluster_topic_left;
      std::string visualize_cluster_topic_left;
      double distance_left;


      ros::Subscriber sb_object_right;
      ros::Subscriber sb_cluster_right;
      ros::Publisher pb_object_right;
      ros::Publisher pb_object_marker_right;
      ros::Publisher pb_object_velocity_right;
      ros::Publisher pb_cluster_right;
      ros::Publisher pb_cluster_marker_right;
      std::string radar_sub_object_topic_right;
      std::string radar_pub_object_topic_right;
      std::string visualize_object_topic_right;
      std::string visualize_velocity_topic_right;
      std::string radar_sub_cluster_topic_right;
      std::string radar_pub_cluster_topic_right;
      std::string visualize_cluster_topic_right; 
      double distance_right;


      ros::Subscriber sb_object_back;
      ros::Subscriber sb_cluster_back;
      ros::Publisher pb_object_back;
      ros::Publisher pb_object_marker_back;
      ros::Publisher pb_object_velocity_back;
      ros::Publisher pb_cluster_back;
      ros::Publisher pb_cluster_marker_back;
      std::string radar_sub_object_topic_back;
      std::string radar_pub_object_topic_back;
      std::string visualize_object_topic_back;
      std::string visualize_velocity_topic_back;
      std::string radar_sub_cluster_topic_back;
      std::string radar_pub_cluster_topic_back;
      std::string visualize_cluster_topic_back;  
      double distance_back;

      ros::Subscriber combine_object;
      ros::Subscriber combine_cluster;
      ros::Publisher pb_object_marker_combine;
      ros::Publisher pb_object_velocity_combine;
      ros::Publisher pb_cluster_marker_combine;

      Eigen::Matrix4d RT_front;// rotation matrix and translation vector
      std::vector<double> matrix_front;

      Eigen::Matrix4d RT_left;
      std::vector<double> matrix_left;

      Eigen::Matrix4d RT_right;
      std::vector<double> matrix_right;
      
      Eigen::Matrix4d RT_back;
      std::vector<double> matrix_back;

      void loadCalibrationData_front (void);
      void loadCalibrationData_left (void);
      void loadCalibrationData_right (void);
      void loadCalibrationData_back (void);
      void visualization_object_data(radar_ros::ObjectList object_list, ros::Publisher object_pb, ros::Publisher velocity_pb);
      radar_ros::ObjectList filter_calib_object(const radar_ros::ObjectList::ConstPtr& object_list, Eigen::Matrix4d RT, double distance);
      void visualization_cluster_data(radar_ros::ClusterList cluster_list, ros::Publisher cluster_pb);
      radar_ros::ClusterList filter_calib_cluster(const radar_ros::ClusterList::ConstPtr& cluster_list, Eigen::Matrix4d RT, double distance);

      void object_msg_Callback_front(const radar_ros::ObjectList::ConstPtr& object_list_front);
      void object_msg_Callback_left(const radar_ros::ObjectList::ConstPtr& object_list_left);
      void object_msg_Callback_right(const radar_ros::ObjectList::ConstPtr& object_list_right);
      void object_msg_Callback_back(const radar_ros::ObjectList::ConstPtr& object_list_back);

      void cluster_msg_Callback_front(const radar_ros::ClusterList::ConstPtr& cluster_list_front);
      void cluster_msg_Callback_left(const radar_ros::ClusterList::ConstPtr& cluster_list_left);
      void cluster_msg_Callback_right(const radar_ros::ClusterList::ConstPtr& cluster_list_right);
      void cluster_msg_Callback_back(const radar_ros::ClusterList::ConstPtr& cluster_list_back);

      void object_msg_Callback_combine(const radar_ros::ObjectList object_list_combine);
      void cluster_msg_Callback_combine(const radar_ros::ClusterList cluster_list_combine);

    public:
      filter();
      ~filter(){};
      void run();

};

#endif