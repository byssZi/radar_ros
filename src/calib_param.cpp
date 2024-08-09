#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <radar_ros/radar_rosConfig.h>
#include "radar_ros/Calib.h"
 
float x_front = 0.0;
float y_front = 0.0;
float z_front = 0.0;
float roll_front = 0.0;
float pitch_front = 0.0;
float yaw_front = 0.0;

float x_left = 0.0;
float y_left = 0.0;
float z_left = 0.0;
float roll_left = 0.0;
float pitch_left = 0.0;
float yaw_left = 0.0;

float x_right = 0.0;
float y_right = 0.0;
float z_right = 0.0;
float roll_right = 0.0;
float pitch_right = 0.0;
float yaw_right = 0.0;

float x_back = 0.0;
float y_back = 0.0;
float z_back = 0.0;
float roll_back = 0.0;
float pitch_back = 0.0;
float yaw_back = 0.0;
 
//回调函数
void Callback(radar_ros::radar_rosConfig &config)
{
    x_front = config.x_front;
    y_front = config.y_front;
    z_front = config.z_front;
    roll_front = config.roll_front;
    pitch_front = config.pitch_front;
    yaw_front = config.yaw_front;

    x_left = config.x_left;
    y_left = config.y_left;
    z_left = config.z_left;
    roll_left = config.roll_left;
    pitch_left = config.pitch_left;
    yaw_left = config.yaw_left;

    x_right = config.x_right;
    y_right = config.y_right;
    z_right = config.z_right;
    roll_right = config.roll_right;
    pitch_right = config.pitch_right;
    yaw_right = config.yaw_right;

    x_back = config.x_back;
    y_back = config.y_back;
    z_back = config.z_back;
    roll_back = config.roll_back;
    pitch_back = config.pitch_back;
    yaw_back = config.yaw_back;
 
    ROS_INFO("x_front y_front z_front roll_front pitch_front yaw_front = [%f %f %f %f %f %f]",x_front,y_front,z_front,roll_front,pitch_front,yaw_front);
    ROS_INFO("x_left y_left z_left roll_left pitch_left yaw_left = [%f %f %f %f %f %f]",x_left,y_left,z_left,roll_left,pitch_left,yaw_left);
    ROS_INFO("x_right y_right z_right roll_right pitch_right yaw_right = [%f %f %f %f %f %f]",x_right,y_right,z_right,roll_right,pitch_right,yaw_right);
    ROS_INFO("x_back y_back z_back roll_back pitch_back yaw_back = [%f %f %f %f %f %f]",x_back,y_back,z_back,roll_back,pitch_back,yaw_back);
}
 
 
int main(int argc,char **argv)
{
    //初始化，创建节点
    ros::init(argc,argv,"calib_param");
 
    //创建一个参数动态配置的服务器实例
    dynamic_reconfigure::Server<radar_ros::radar_rosConfig> server;
    //定义回调函数
    dynamic_reconfigure::Server<radar_ros::radar_rosConfig>::CallbackType f;
 
    //将回调函数和服务端绑定，当客户端请求修改参数时，服务器跳转到回调函数进行处理
    f = boost::bind(&Callback,_1);
    server.setCallback(f);
    ros::NodeHandle n("dynamic_pub");
    ros::Publisher param_front = n.advertise<radar_ros::Calib>("radar_front_param",1);
    ros::Publisher param_left = n.advertise<radar_ros::Calib>("radar_left_param",1);
    ros::Publisher param_right = n.advertise<radar_ros::Calib>("radar_right_param",1);
    ros::Publisher param_back = n.advertise<radar_ros::Calib>("radar_back_param",1);
    ros::Rate rate(1);  
    while(ros::ok()){  
        radar_ros::Calib data_front;  
        data_front.x = x_front;
        data_front.y = y_front;
        data_front.z = z_front;
        data_front.roll = roll_front;
        data_front.pitch = pitch_front;
        data_front.yaw = yaw_front;
        // publish data  
        param_front.publish(data_front);

        radar_ros::Calib data_left;  
        data_left.x = x_left;
        data_left.y = y_left;
        data_left.z = z_left;
        data_left.roll = roll_left;
        data_left.pitch = pitch_left;
        data_left.yaw = yaw_left;
        // publish data  
        param_left.publish(data_left);

        radar_ros::Calib data_right;  
        data_right.x = x_right;
        data_right.y = y_right;
        data_right.z = z_right;
        data_right.roll = roll_right;
        data_right.pitch = pitch_right;
        data_right.yaw = yaw_right;
        // publish data  
        param_right.publish(data_right); 

        radar_ros::Calib data_back;  
        data_back.x = x_back;
        data_back.y = y_back;
        data_back.z = z_back;
        data_back.roll = roll_back;
        data_back.pitch = pitch_back;
        data_back.yaw = yaw_back;
        // publish data  
        param_back.publish(data_back);    
        // callback function  
        ros::spinOnce();  
        // sleep   
        rate.sleep();  
    }  
 
    return 0;
}