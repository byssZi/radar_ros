#include "radar_ros/radar_filter.h"

using namespace std;


filter::filter(){

    nh.param<double>("radar_filter_distance_front",filter::distance_front,70.0);
    nh.param<std::string>("object_can0_id0_topic",filter::radar_sub_object_topic_front,"/ars_40X/objects_front");
    nh.param<std::string>("radar_output_object_topic_front",filter::radar_pub_object_topic_front,"/ars_40X/filter/objects_front");
    nh.param<std::string>("radar_visualize_object_topic_front", filter::visualize_object_topic_front,"/visualize_objects_front");
    nh.param<std::string>("radar_visualize_velocity_topic_front", filter::visualize_velocity_topic_front,"/visualize_velocity_front");
    nh.param<std::string>("cluster_can0_id0_topic",filter::radar_sub_cluster_topic_front,"/ars_40X/clusters_front");
    nh.param<std::string>("radar_output_cluster_topic_front",filter::radar_pub_cluster_topic_front,"/ars_40X/filter/clusters_front");
    nh.param<std::string>("radar_visualize_cluster_topic_front", filter::visualize_cluster_topic_front,"/visualize_clusters_front");

    nh.param<double>("radar_filter_distance_left",filter::distance_left,70.0);
    nh.param<std::string>("object_can1_id0_topic",filter::radar_sub_object_topic_left,"/ars_40X/objects_left");
    nh.param<std::string>("radar_output_object_topic_left",filter::radar_pub_object_topic_left,"/ars_40X/filter/objects_left");
    nh.param<std::string>("radar_visualize_object_topic_left", filter::visualize_object_topic_left,"/visualize_objects_left");
    nh.param<std::string>("radar_visualize_velocity_topic_left", filter::visualize_velocity_topic_left,"/visualize_velocity_left");
    nh.param<std::string>("cluster_can1_id0_topic",filter::radar_sub_cluster_topic_left,"/ars_40X/clusters_left");
    nh.param<std::string>("radar_output_cluster_topic_left",filter::radar_pub_cluster_topic_left,"/ars_40X/filter/clusters_left");
    nh.param<std::string>("radar_visualize_cluster_topic_left", filter::visualize_cluster_topic_left,"/visualize_clusters_left");

    nh.param<double>("radar_filter_distance_right",filter::distance_right,70.0);
    nh.param<std::string>("object_can1_id1_topic",filter::radar_sub_object_topic_right,"/ars_40X/objects_right");
    nh.param<std::string>("radar_output_object_topic_right",filter::radar_pub_object_topic_right,"/ars_40X/filter/objects_right"); 
    nh.param<std::string>("radar_visualize_object_topic_right", filter::visualize_object_topic_right,"/visualize_objects_right");
    nh.param<std::string>("radar_visualize_velocity_topic_right", filter::visualize_velocity_topic_right,"/visualize_velocity_right");
    nh.param<std::string>("cluster_can1_id1_topic",filter::radar_sub_cluster_topic_right,"/ars_40X/clusters_right");
    nh.param<std::string>("radar_output_cluster_topic_right",filter::radar_pub_cluster_topic_right,"/ars_40X/filter/clusters_right");
    nh.param<std::string>("radar_visualize_cluster_topic_right", filter::visualize_cluster_topic_right,"/visualize_clusters_right");

    nh.param<double>("radar_filter_distance_back",filter::distance_back,70.0);
    nh.param<std::string>("object_can0_id1_topic",filter::radar_sub_object_topic_back,"/ars_40X/objects_back");
    nh.param<std::string>("radar_output_object_topic_back",filter::radar_pub_object_topic_back,"/ars_40X/filter/objects_back"); 
    nh.param<std::string>("radar_visualize_object_topic_back", filter::visualize_object_topic_back,"/visualize_objects_back");
    nh.param<std::string>("radar_visualize_velocity_topic_back", filter::visualize_velocity_topic_back,"/visualize_velocity_back");
    nh.param<std::string>("cluster_can0_id1_topic",filter::radar_sub_cluster_topic_back,"/ars_40X/clusters_back");
    nh.param<std::string>("radar_output_cluster_topic_back",filter::radar_pub_cluster_topic_back,"/ars_40X/filter/clusters_back");
    nh.param<std::string>("radar_visualize_cluster_topic_back", filter::visualize_cluster_topic_back,"/visualize_clusters_back");

    nh.param<vector<double>>("radar/RT_front", filter::matrix_front,vector<double>());    
    nh.param<vector<double>>("radar/RT_left", filter::matrix_left,vector<double>());
    nh.param<vector<double>>("radar/RT_right", filter::matrix_right,vector<double>()); 
    nh.param<vector<double>>("radar/RT_back", filter::matrix_back,vector<double>()); 
    filter::loadCalibrationData_front();
    filter::loadCalibrationData_left();
    filter::loadCalibrationData_right();
    filter::loadCalibrationData_back();
};

void filter::run(){

    sb_object_front = nh.subscribe(radar_sub_object_topic_front,50,&filter::object_msg_Callback_front, this);
    pb_object_front = nh.advertise<radar_ros::ObjectList>(radar_pub_object_topic_front, 10);
    pb_object_marker_front = nh.advertise<visualization_msgs::MarkerArray>(visualize_object_topic_front,50);
    pb_object_velocity_front = nh.advertise<visualization_msgs::MarkerArray>(visualize_velocity_topic_front,50);
    sb_cluster_front = nh.subscribe(radar_sub_cluster_topic_front,50,&filter::cluster_msg_Callback_front, this);
    pb_cluster_front = nh.advertise<radar_ros::ClusterList>(radar_pub_cluster_topic_front, 10);
    pb_cluster_marker_front = nh.advertise<visualization_msgs::MarkerArray>(visualize_cluster_topic_front,50);

    sb_object_left = nh.subscribe(radar_sub_object_topic_left,50,&filter::object_msg_Callback_left, this);
    pb_object_left= nh.advertise<radar_ros::ObjectList>(radar_pub_object_topic_left, 10);
    pb_object_marker_left = nh.advertise<visualization_msgs::MarkerArray>(visualize_object_topic_left,50);
    pb_object_velocity_left = nh.advertise<visualization_msgs::MarkerArray>(visualize_velocity_topic_left,50);
    sb_cluster_left = nh.subscribe(radar_sub_cluster_topic_left,50,&filter::cluster_msg_Callback_left, this);
    pb_cluster_left = nh.advertise<radar_ros::ClusterList>(radar_pub_cluster_topic_left, 10);
    pb_cluster_marker_left = nh.advertise<visualization_msgs::MarkerArray>(visualize_cluster_topic_left,50);

    sb_object_right = nh.subscribe(radar_sub_object_topic_right,50,&filter::object_msg_Callback_right, this);
    pb_object_right = nh.advertise<radar_ros::ObjectList>(radar_pub_object_topic_right, 10);
    pb_object_marker_right = nh.advertise<visualization_msgs::MarkerArray>(visualize_object_topic_right,50);
    pb_object_velocity_right = nh.advertise<visualization_msgs::MarkerArray>(visualize_velocity_topic_right,50);
    sb_cluster_right = nh.subscribe(radar_sub_cluster_topic_right,50,&filter::cluster_msg_Callback_right, this);
    pb_cluster_right = nh.advertise<radar_ros::ClusterList>(radar_pub_cluster_topic_right, 10);
    pb_cluster_marker_right = nh.advertise<visualization_msgs::MarkerArray>(visualize_cluster_topic_right,50); 

    sb_object_back = nh.subscribe(radar_sub_object_topic_back,50,&filter::object_msg_Callback_back, this);
    pb_object_back = nh.advertise<radar_ros::ObjectList>(radar_pub_object_topic_back, 10);
    pb_object_marker_back = nh.advertise<visualization_msgs::MarkerArray>(visualize_object_topic_back,50);
    pb_object_velocity_back = nh.advertise<visualization_msgs::MarkerArray>(visualize_velocity_topic_back,50);
    sb_cluster_back = nh.subscribe(radar_sub_cluster_topic_back,50,&filter::cluster_msg_Callback_back, this);
    pb_cluster_back = nh.advertise<radar_ros::ClusterList>(radar_pub_cluster_topic_back, 10);
    pb_cluster_marker_back = nh.advertise<visualization_msgs::MarkerArray>(visualize_cluster_topic_back,50);

    combine_object = nh.subscribe("/ars_40X/combined_objects",50,&filter::object_msg_Callback_combine, this);
    combine_cluster = nh.subscribe("/ars_40X/combined_clusters",50,&filter::cluster_msg_Callback_combine, this);
    pb_object_marker_combine = nh.advertise<visualization_msgs::MarkerArray>("/visualize_objects_combine",50);
    pb_object_velocity_combine = nh.advertise<visualization_msgs::MarkerArray>("/visualize_velocity_combine",50);  
    pb_cluster_marker_combine = nh.advertise<visualization_msgs::MarkerArray>("/visualize_clusters_combine",50); 

};

void filter::loadCalibrationData_front(void){
    RT_front(0,0) = matrix_front[0]; RT_front(0,1) = matrix_front[1]; RT_front(0,2) = matrix_front[2]; RT_front(0,3) = matrix_front[3];//
    RT_front(1,0) = matrix_front[4]; RT_front(1,1) = matrix_front[5]; RT_front(1,2) = matrix_front[6]; RT_front(1,3) = matrix_front[7];
    RT_front(2,0) = matrix_front[8]; RT_front(2,1) = matrix_front[9]; RT_front(2,2) = matrix_front[10]; RT_front(2,3) = matrix_front[11];
    RT_front(3,0) = matrix_front[12]; RT_front(3,1) = matrix_front[13]; RT_front(3,2) = matrix_front[14]; RT_front(3,3) = matrix_front[15];
};

void filter::loadCalibrationData_left(void){
    RT_left(0,0) = matrix_left[0]; RT_left(0,1) = matrix_left[1]; RT_left(0,2) = matrix_left[2]; RT_left(0,3) = matrix_left[3];//
    RT_left(1,0) = matrix_left[4]; RT_left(1,1) = matrix_left[5]; RT_left(1,2) = matrix_left[6]; RT_left(1,3) = matrix_left[7];
    RT_left(2,0) = matrix_left[8]; RT_left(2,1) = matrix_left[9]; RT_left(2,2) = matrix_left[10]; RT_left(2,3) = matrix_left[11];
    RT_left(3,0) = matrix_left[12]; RT_left(3,1) = matrix_left[13]; RT_left(3,2) = matrix_left[14]; RT_left(3,3) = matrix_left[15];
};

void filter::loadCalibrationData_right(void){
    RT_right(0,0) = matrix_right[0]; RT_right(0,1) = matrix_right[1]; RT_right(0,2) = matrix_right[2]; RT_right(0,3) = matrix_right[3];//
    RT_right(1,0) = matrix_right[4]; RT_right(1,1) = matrix_right[5]; RT_right(1,2) = matrix_right[6]; RT_right(1,3) = matrix_right[7];
    RT_right(2,0) = matrix_right[8]; RT_right(2,1) = matrix_right[9]; RT_right(2,2) = matrix_right[10]; RT_right(2,3) = matrix_right[11];
    RT_right(3,0) = matrix_right[12]; RT_right(3,1) = matrix_right[13]; RT_right(3,2) = matrix_right[14]; RT_right(3,3) = matrix_right[15];
};

void filter::loadCalibrationData_back(void){
    RT_back(0,0) = matrix_back[0]; RT_back(0,1) = matrix_back[1]; RT_back(0,2) = matrix_back[2]; RT_back(0,3) = matrix_back[3];//
    RT_back(1,0) = matrix_back[4]; RT_back(1,1) = matrix_back[5]; RT_back(1,2) = matrix_back[6]; RT_back(1,3) = matrix_back[7];
    RT_back(2,0) = matrix_back[8]; RT_back(2,1) = matrix_back[9]; RT_back(2,2) = matrix_back[10]; RT_back(2,3) = matrix_back[11];
    RT_back(3,0) = matrix_back[12]; RT_back(3,1) = matrix_back[13]; RT_back(3,2) = matrix_back[14]; RT_back(3,3) = matrix_back[15];
};

void filter::object_msg_Callback_front(const radar_ros::ObjectList::ConstPtr& object_list_front){
    radar_ros::ObjectList object_list_pub;
    object_list_pub = filter_calib_object(object_list_front, RT_front, distance_front);
/*     printf ("\nMethod #2: using an Affine3f\n");
    std::cout<<RT<<RT_lidar<<std::endl; */
    pb_object_front.publish(object_list_pub);
    visualization_object_data(object_list_pub,pb_object_marker_front,pb_object_velocity_front);
};

void filter::object_msg_Callback_left(const radar_ros::ObjectList::ConstPtr& object_list_left){
    radar_ros::ObjectList object_list_pub;
    object_list_pub = filter_calib_object(object_list_left, RT_left, distance_left);
/*     printf ("\nMethod #2: using an Affine3f_left\n");
    std::cout<<RT_left<<RT_left_lidar<<std::endl; */
    pb_object_left.publish(object_list_pub);
    visualization_object_data(object_list_pub,pb_object_marker_left,pb_object_velocity_left);
};

void filter::object_msg_Callback_right(const radar_ros::ObjectList::ConstPtr& object_list_right){
    radar_ros::ObjectList object_list_pub;
    object_list_pub = filter_calib_object(object_list_right, RT_right, distance_right);
/*     printf ("\nMethod #2: using an Affine3f_right\n");
    std::cout<<RT_right<<RT_right_lidar<<std::endl; */
    pb_object_right.publish(object_list_pub);
    visualization_object_data(object_list_pub,pb_object_marker_right,pb_object_velocity_right);
};

void filter::object_msg_Callback_back(const radar_ros::ObjectList::ConstPtr& object_list_back){
    radar_ros::ObjectList object_list_pub;
    object_list_pub = filter_calib_object(object_list_back, RT_back, distance_back);
/*     printf ("\nMethod #2: using an Affine3f_back\n");
    std::cout<<RT_back<<RT_back_lidar<<std::endl; */
    pb_object_back.publish(object_list_pub);
    visualization_object_data(object_list_pub,pb_object_marker_back,pb_object_velocity_back);
};

void filter::cluster_msg_Callback_front(const radar_ros::ClusterList::ConstPtr& cluster_list_front){
    radar_ros::ClusterList cluster_list_pub;
    cluster_list_pub = filter_calib_cluster(cluster_list_front, RT_front, distance_front);
/*     printf ("\nMethod #2: using an Affine3f\n");
    std::cout<<RT<<RT_lidar<<std::endl; */
    pb_cluster_front.publish(cluster_list_pub);
    visualization_cluster_data(cluster_list_pub,pb_cluster_marker_front);
};

void filter::cluster_msg_Callback_left(const radar_ros::ClusterList::ConstPtr& cluster_list_left){
    radar_ros::ClusterList cluster_list_pub;
    cluster_list_pub = filter_calib_cluster(cluster_list_left, RT_left, distance_left);
/*     printf ("\nMethod #2: using an Affine3f\n");
    std::cout<<RT<<RT_lidar<<std::endl; */
    pb_cluster_left.publish(cluster_list_pub);
    visualization_cluster_data(cluster_list_pub,pb_cluster_marker_left);
};

void filter::cluster_msg_Callback_right(const radar_ros::ClusterList::ConstPtr& cluster_list_right){
    radar_ros::ClusterList cluster_list_pub;
    cluster_list_pub = filter_calib_cluster(cluster_list_right, RT_right, distance_right);
/*     printf ("\nMethod #2: using an Affine3f\n");
    std::cout<<RT<<RT_lidar<<std::endl; */
    pb_cluster_right.publish(cluster_list_pub);
    visualization_cluster_data(cluster_list_pub,pb_cluster_marker_right);
};

void filter::cluster_msg_Callback_back(const radar_ros::ClusterList::ConstPtr& cluster_list_back){
    radar_ros::ClusterList cluster_list_pub;
    cluster_list_pub = filter_calib_cluster(cluster_list_back, RT_back, distance_back);
/*     printf ("\nMethod #2: using an Affine3f\n");
    std::cout<<RT<<RT_lidar<<std::endl; */
    pb_cluster_back.publish(cluster_list_pub);
    visualization_cluster_data(cluster_list_pub,pb_cluster_marker_back);
};

void filter::object_msg_Callback_combine(const radar_ros::ObjectList object_list_combine){
    visualization_object_data(object_list_combine,pb_object_marker_combine,pb_object_velocity_combine);
}

void filter::cluster_msg_Callback_combine(const radar_ros::ClusterList cluster_list_combine){
    visualization_cluster_data(cluster_list_combine,pb_cluster_marker_combine);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_filter");  //初始化
    filter fl;
    fl.run();
    ros::spin();
    return 0;
}