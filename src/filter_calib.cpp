#include "radar_ros/radar_filter.h"


radar_ros::ObjectList filter::filter_calib_object(const radar_ros::ObjectList::ConstPtr& object_list, Eigen::Matrix4d RT, double distance){
    radar_ros::ObjectList object_list_new;
    radar_ros::ObjectList object_list_pub;
    object_list_new.header = object_list->header;
    object_list_pub.header = object_list->header;
    for(auto object : object_list->objects){
      switch (object.prob_of_exist){
        case  INVALID:{
          break;
        }
        case  PERCENT_25:{
          break;
        }
        case  PERCENT_50:{
          break;
        }
        case PERCENT_75:{
          break;
        }
        case PERCENT_90:{
          break;
        }
        case PERCENT_99:{
          //if (hypot(object.relative_velocity.twist.linear.x, object.relative_velocity.twist.linear.y)>0){
            //object_list_new.objects.push_back(object);
            //break;
          //}
          //else{
            //break;
          //}
          object_list_new.objects.push_back(object);
          break;
        }
        case PERCENT_99_9:{
          //if (hypot(object.relative_velocity.twist.linear.x, object.relative_velocity.twist.linear.y)>0){
            //object_list_new.objects.push_back(object);
            //break;
          //}
          //else{
            //break;
          //}
          object_list_new.objects.push_back(object);
          break;
        }
        case PERCENT_100:{
          //if (hypot(object.relative_velocity.twist.linear.x, object.relative_velocity.twist.linear.y)>0){
            //object_list_new.objects.push_back(object);
            //break;
          //}
          //else{
            //break;
          //}
          object_list_new.objects.push_back(object);
          break;
        }
      }
    }
    Eigen::Vector4d X;
    Eigen::Vector4d Y;

    Eigen::Vector3d VX;
    Eigen::Vector3d VY;
    Eigen::Matrix4d VR_tmp;
    Eigen::Matrix3d VR;

    Eigen::Quaterniond q_odom_curr_tmp;
    Eigen::Matrix3d R_odom_curr_tmp;

    Eigen::Matrix4d QX;
    Eigen::Matrix4d QY;
    Eigen::Matrix3d matrix_23f;
    Eigen::Quaterniond q_odom_curr_now;

    for (auto object : object_list_new.objects){

        X[0] = object.position.pose.position.x;//标定后坐标变换
        X[1] = object.position.pose.position.y;
        X[2] = object.position.pose.position.z;
        X[3] = 1;
        Y = RT*X;
        object.position.pose.position.x = Y[0];
        object.position.pose.position.y = Y[1];
        object.position.pose.position.z = Y[2];


        VX[0] = object.relative_velocity.twist.linear.x;
        VX[1] = object.relative_velocity.twist.linear.y;
        VX[2] = object.relative_velocity.twist.linear.z;
        VR(0,0) = RT(0,0);  VR(0,1) = RT(0,1);  VR(0,2) = RT(0,2);
        VR(1,0) = RT(1,0);  VR(1,1) = RT(1,1);  VR(1,2) = RT(1,2);
        VR(2,0) = RT(2,0);  VR(2,1) = RT(2,1);  VR(2,2) = RT(2,2);
        VY = VR*VX;
        object.relative_velocity.twist.linear.x = VY[0];
        object.relative_velocity.twist.linear.y = VY[1];
        object.relative_velocity.twist.linear.z = VY[2];


        q_odom_curr_tmp.x() = object.position.pose.orientation.x;
        q_odom_curr_tmp.y() = object.position.pose.orientation.y;
        q_odom_curr_tmp.z() = object.position.pose.orientation.z;
        q_odom_curr_tmp.w() = object.position.pose.orientation.w;

        R_odom_curr_tmp= q_odom_curr_tmp.normalized().toRotationMatrix(); 

        QX(0,0) = R_odom_curr_tmp(0,0);  QX(0,1) = R_odom_curr_tmp(0,1);  QX(0,2) = R_odom_curr_tmp(0,2);  QX(0,3) = 0;
        QX(1,0) = R_odom_curr_tmp(1,0);  QX(1,1) = R_odom_curr_tmp(1,1);  QX(1,2) = R_odom_curr_tmp(1,2);  QX(1,3) = 0;
        QX(2,0) = R_odom_curr_tmp(2,0);  QX(2,1) = R_odom_curr_tmp(2,1);  QX(2,2) = R_odom_curr_tmp(2,2);  QX(2,3) = 0;
        QX(3,0) = 0                   ;  QX(3,1) = 0                   ;  QX(3,2) = 0                   ;  QX(3,3) = 1; 

        QY = RT*QX;

        matrix_23f(0,0) = QY(0,0);  matrix_23f(0,1) = QY(0,1);  matrix_23f(0,2) = QY(0,2);
        matrix_23f(1,0) = QY(1,0);  matrix_23f(1,1) = QY(1,1);  matrix_23f(1,2) = QY(1,2);
        matrix_23f(2,0) = QY(2,0);  matrix_23f(2,1) = QY(2,1);  matrix_23f(2,2) = QY(2,2);

        q_odom_curr_now=Eigen::Quaterniond(matrix_23f);
        q_odom_curr_now.normalize();


        object.position.pose.orientation.x = q_odom_curr_now.x();
        object.position.pose.orientation.y = q_odom_curr_now.y();
        object.position.pose.orientation.z = q_odom_curr_now.z();
        object.position.pose.orientation.w = q_odom_curr_now.w(); 


      switch (object.class_type) {
        case POINT:{
          break;
        }
        case CAR:{
          if(hypot(object.position.pose.position.x, object.position.pose.position.y)>distance){
            break;
          }
          else{
            object_list_pub.objects.push_back(object);
            break;
          }
          object_list_pub.objects.push_back(object);
          break;
        }
        case TRUCK:{
          if(hypot(object.position.pose.position.x, object.position.pose.position.y)>distance){
            break;
          }
          else{
            object_list_pub.objects.push_back(object);
            break;
          }
          object_list_pub.objects.push_back(object);
          break;
        }
        case PEDESTRIAN:{
          if(hypot(object.position.pose.position.x, object.position.pose.position.y)>distance){
            break;
          }
          else{
            object_list_pub.objects.push_back(object);
            break;
          }
          object_list_pub.objects.push_back(object);
          break;
        }
        case MOTORCYCLE:{
          if(hypot(object.position.pose.position.x, object.position.pose.position.y)>distance){
            break;
          }
          else{
            object_list_pub.objects.push_back(object);
            break;
          }
          object_list_pub.objects.push_back(object);
          break;
        }
        case BICYCLE:{
           if(hypot(object.position.pose.position.x, object.position.pose.position.y)>distance){
              break;
            }
            else{
              object_list_pub.objects.push_back(object);
              break;
            }
          object_list_pub.objects.push_back(object);
          break;
        }
        case WIDE:{
           if(hypot(object.position.pose.position.x, object.position.pose.position.y)>distance){
              break;
            }
            else{
              object_list_pub.objects.push_back(object);
              break;
            }
          object_list_pub.objects.push_back(object);
          break;
        }
        case RESERVED:{
          break;
        }
      }
    }
  return object_list_pub;
}

radar_ros::ClusterList filter::filter_calib_cluster(const radar_ros::ClusterList::ConstPtr& cluster_list, Eigen::Matrix4d RT, double distance){
    radar_ros::ClusterList cluster_list_pub;
    cluster_list_pub.header = cluster_list->header;
    Eigen::Vector4d X;
    Eigen::Vector4d Y;

    Eigen::Vector3d VX;
    Eigen::Vector3d VY;
    Eigen::Matrix4d VR_tmp;
    Eigen::Matrix3d VR;
    for(auto cluster : cluster_list->clusters){
      X[0] = cluster.position.pose.position.x;//标定后坐标变换
      X[1] = cluster.position.pose.position.y;
      X[2] = cluster.position.pose.position.z;
      X[3] = 1;
      Y = RT*X;
      cluster.position.pose.position.x = Y[0];
      cluster.position.pose.position.y = Y[1];
      cluster.position.pose.position.z = Y[2];


      VX[0] = cluster.relative_velocity.twist.linear.x;
      VX[1] = cluster.relative_velocity.twist.linear.y;
      VX[2] = cluster.relative_velocity.twist.linear.z;
      VR(0,0) = RT(0,0);  VR(0,1) = RT(0,1);  VR(0,2) = RT(0,2);
      VR(1,0) = RT(1,0);  VR(1,1) = RT(1,1);  VR(1,2) = RT(1,2);
      VR(2,0) = RT(2,0);  VR(2,1) = RT(2,1);  VR(2,2) = RT(2,2);
      VY = VR*VX;
      cluster.relative_velocity.twist.linear.x = VY[0];
      cluster.relative_velocity.twist.linear.y = VY[1];
      cluster.relative_velocity.twist.linear.z = VY[2];

      if(hypot(cluster.position.pose.position.x, cluster.position.pose.position.y)<distance){
        cluster_list_pub.clusters.push_back(cluster);
      }
    }
    return cluster_list_pub;
}
