#include "radar_ros/dynamic_calib.h"

using namespace std;



dynamic_calib::dynamic_calib(){
    nh.param<std::string>("radar_calib_visualize_front",dynamic_calib::sb_front_topic,"/visualize_objects_front");
    nh.param<std::string>("radar_calib_visualize_left",dynamic_calib::sb_left_topic,"/visualize_objects_left");
    nh.param<std::string>("radar_calib_visualize_right",dynamic_calib::sb_right_topic,"/visualize_objects_right");
    nh.param<std::string>("radar_calib_visualize_back",dynamic_calib::sb_back_topic,"/visualize_objects_back");
    nh.param<std::string>("radar_calib_result_path",dynamic_calib::CSV_PATH,"/home/nuc/catkin_ws/src/radar_ros/data_radar.txt");
    csv.open(CSV_PATH);
    // Dynamic Parameter Server & Function
    f = boost::bind(&dynamic_calib::dynamicParamCallback, this, _1);
    server.setCallback(f);
};

dynamic_calib::~dynamic_calib(){
    csv<< "main_transform_ = " << "\n"
       << main_transform_(0,0) << "," << main_transform_(0,1) << "," << main_transform_(0,2) << "," << main_transform_(0,3) << "\n"
       << main_transform_(1,0) << "," << main_transform_(1,1) << "," << main_transform_(1,2) << "," << main_transform_(1,3) << "\n"
       << main_transform_(2,0) << "," << main_transform_(2,1) << "," << main_transform_(2,2) << "," << main_transform_(2,3) << "\n"
       << main_transform_(3,0) << "," << main_transform_(3,1) <<","  << main_transform_(3,2) << "," << main_transform_(3,3) << "\n"

       << "left_transform_ = " << "\n"
       << left_transform_(0,0) << "," << left_transform_(0,1) << "," << left_transform_(0,2) << "," << left_transform_(0,3) << "\n"
       << left_transform_(1,0) << "," << left_transform_(1,1) << "," << left_transform_(1,2) << "," << left_transform_(1,3) << "\n"
       << left_transform_(2,0) << "," << left_transform_(2,1) << "," << left_transform_(2,2) << "," << left_transform_(2,3) << "\n"
       << left_transform_(3,0) << "," << left_transform_(3,1) <<","  << left_transform_(3,2) << "," << left_transform_(3,3) << "\n"

       << "right_transform_ = " << "\n"
       << right_transform_(0,0) << "," << right_transform_(0,1) << "," << right_transform_(0,2) << "," << right_transform_(0,3) << "\n"
       << right_transform_(1,0) << "," << right_transform_(1,1) << "," << right_transform_(1,2) << "," << right_transform_(1,3) << "\n"
       << right_transform_(2,0) << "," << right_transform_(2,1) << "," << right_transform_(2,2) << "," << right_transform_(2,3) << "\n"
       << right_transform_(3,0) << "," << right_transform_(3,1) <<","  << right_transform_(3,2) << "," << right_transform_(3,3) << "\n"

       << "back_transform_ = " << "\n"
       << back_transform_(0,0) << "," << back_transform_(0,1) << "," << back_transform_(0,2) << "," << back_transform_(0,3) << "\n"
       << back_transform_(1,0) << "," << back_transform_(1,1) << "," << back_transform_(1,2) << "," << back_transform_(1,3) << "\n"
       << back_transform_(2,0) << "," << back_transform_(2,1) << "," << back_transform_(2,2) << "," << back_transform_(2,3) << "\n"
       << back_transform_(3,0) << "," << back_transform_(3,1) <<","  << back_transform_(3,2) << "," << back_transform_(3,3) << "\n";
    csv.close();
};

void dynamic_calib::run(){
    sb_front = nh.subscribe(sb_front_topic,1,&dynamic_calib::msg_Callback_front, this);
    sb_left = nh.subscribe(sb_left_topic,1,&dynamic_calib::msg_Callback_left, this);
    sb_right = nh.subscribe(sb_right_topic,1,&dynamic_calib::msg_Callback_right, this);
    sb_back = nh.subscribe(sb_back_topic,1,&dynamic_calib::msg_Callback_back, this);
 
    pb_front = nh.advertise<visualization_msgs::MarkerArray>("/calib/calib_front",1);
    pb_left = nh.advertise<visualization_msgs::MarkerArray>("/calib/calib_left",1);
    pb_right = nh.advertise<visualization_msgs::MarkerArray>("/calib/calib_right",1);
    pb_back = nh.advertise<visualization_msgs::MarkerArray>("/calib/calib_back",1); 
};

//回调函数
void dynamic_calib::dynamicParamCallback(radar_ros::radar_rosConfig &config){
    corMain_x = config.x_front;
    corMain_y = config.y_front;
    corMain_z = config.z_front;
    corMain_roll = config.roll_front;
    corMain_pitch = config.pitch_front;
    corMain_yaw = config.yaw_front;

    main_transform_ = Eigen::Affine3f::Identity();
    main_transform_.rotate(Eigen::AngleAxisf(corMain_yaw, Eigen::Vector3f::UnitZ()));
    main_transform_.rotate(Eigen::AngleAxisf(corMain_pitch, Eigen::Vector3f::UnitY()));
    main_transform_.rotate(Eigen::AngleAxisf(corMain_roll, Eigen::Vector3f::UnitX()));
    main_transform_.translation() << corMain_x, corMain_y, corMain_z;

    corLeft_x = config.x_left;
    corLeft_y = config.y_left;
    corLeft_z = config.z_left;
    corLeft_roll = config.roll_left;
    corLeft_pitch = config.pitch_left;
    corLeft_yaw = config.yaw_left;

    left_transform_ = Eigen::Affine3f::Identity();
    left_transform_.rotate(Eigen::AngleAxisf(corLeft_yaw, Eigen::Vector3f::UnitZ()));
    left_transform_.rotate(Eigen::AngleAxisf(corLeft_pitch, Eigen::Vector3f::UnitY()));
    left_transform_.rotate(Eigen::AngleAxisf(corLeft_roll, Eigen::Vector3f::UnitX()));
    left_transform_.translation() << corLeft_x, corLeft_y, corLeft_z;

    corRight_x = config.x_right;
    corRight_y = config.y_right;
    corRight_z = config.z_right;
    corRight_roll = config.roll_right;
    corRight_pitch = config.pitch_right;
    corRight_yaw = config.yaw_right;

    right_transform_ = Eigen::Affine3f::Identity();
    right_transform_.rotate(Eigen::AngleAxisf(corRight_yaw, Eigen::Vector3f::UnitZ()));
    right_transform_.rotate(Eigen::AngleAxisf(corRight_pitch, Eigen::Vector3f::UnitY()));
    right_transform_.rotate(Eigen::AngleAxisf(corRight_roll, Eigen::Vector3f::UnitX()));
    right_transform_.translation() << corRight_x, corRight_y, corRight_z;

    corBack_x = config.x_back;
    corBack_y = config.y_back;
    corBack_z = config.z_back;
    corBack_roll = config.roll_back;
    corBack_pitch = config.pitch_back;
    corBack_yaw = config.yaw_back;

    back_transform_ = Eigen::Affine3f::Identity();
    back_transform_.rotate(Eigen::AngleAxisf(corBack_yaw, Eigen::Vector3f::UnitZ()));
    back_transform_.rotate(Eigen::AngleAxisf(corBack_pitch, Eigen::Vector3f::UnitY()));
    back_transform_.rotate(Eigen::AngleAxisf(corBack_roll, Eigen::Vector3f::UnitX()));
    back_transform_.translation() << corBack_x, corBack_y, corBack_z;
}

void dynamic_calib::msg_Callback_front(const visualization_msgs::MarkerArray::ConstPtr& markerarray){
    Eigen::Matrix3f rotate_matrix = main_transform_.linear();
    Eigen::Matrix3f R_odom_curr_tmp;
    R_odom_curr_tmp= Eigen::Quaternionf(rotate_matrix).normalized();
    // 创建一个新的 MarkerArray 以保存变换后的对象
    visualization_msgs::MarkerArray transformed_markers;
    for(auto marker : markerarray->markers){
        // 创建一个新的 Marker 并进行复制
        visualization_msgs::Marker transformed_marker = marker;    
        Eigen::Vector4f pose;
        pose[0] = marker.pose.position.x;
        pose[1] = marker.pose.position.y;
        pose[2] = marker.pose.position.z;
        pose[3] = 1;
        Eigen::Vector4f trans_pose;
        trans_pose = main_transform_*pose;
        transformed_marker.pose.position.x = trans_pose[0];
        transformed_marker.pose.position.y = trans_pose[1];
        transformed_marker.pose.position.z = trans_pose[2];

        Eigen::Quaternionf q_odom_curr_tmp; 
        q_odom_curr_tmp.x() = marker.pose.orientation.x;
        q_odom_curr_tmp.y() = marker.pose.orientation.y;
        q_odom_curr_tmp.z() = marker.pose.orientation.z;
        q_odom_curr_tmp.w() = marker.pose.orientation.w;
        q_odom_curr_tmp.normalize();

        Eigen::Quaternionf transform_orientation;
        transform_orientation = R_odom_curr_tmp*q_odom_curr_tmp;
        transform_orientation.normalize();

        transformed_marker.pose.orientation.x = transform_orientation.x();
        transformed_marker.pose.orientation.y = transform_orientation.y();
        transformed_marker.pose.orientation.z = transform_orientation.z();
        transformed_marker.pose.orientation.w = transform_orientation.w();

        transformed_markers.markers.push_back(transformed_marker);
    }
    pb_front.publish(transformed_markers);
};

void dynamic_calib::msg_Callback_left(const visualization_msgs::MarkerArray::ConstPtr& markerarray){
    Eigen::Matrix3f rotate_matrix = left_transform_.linear();
    Eigen::Matrix3f R_odom_curr_tmp;
    R_odom_curr_tmp= Eigen::Quaternionf(rotate_matrix).normalized();
    // 创建一个新的 MarkerArray 以保存变换后的对象
    visualization_msgs::MarkerArray transformed_markers;
    for(auto marker : markerarray->markers){
        // 创建一个新的 Marker 并进行复制
        visualization_msgs::Marker transformed_marker = marker;    
        Eigen::Vector4f pose;
        pose[0] = marker.pose.position.x;
        pose[1] = marker.pose.position.y;
        pose[2] = marker.pose.position.z;
        pose[3] = 1;
        Eigen::Vector4f trans_pose;
        trans_pose = left_transform_*pose;
        transformed_marker.pose.position.x = trans_pose[0];
        transformed_marker.pose.position.y = trans_pose[1];
        transformed_marker.pose.position.z = trans_pose[2];

        Eigen::Quaternionf q_odom_curr_tmp; 
        q_odom_curr_tmp.x() = marker.pose.orientation.x;
        q_odom_curr_tmp.y() = marker.pose.orientation.y;
        q_odom_curr_tmp.z() = marker.pose.orientation.z;
        q_odom_curr_tmp.w() = marker.pose.orientation.w;
        q_odom_curr_tmp.normalize();

        Eigen::Quaternionf transform_orientation;
        transform_orientation = R_odom_curr_tmp*q_odom_curr_tmp;
        transform_orientation.normalize();

        transformed_marker.pose.orientation.x = transform_orientation.x();
        transformed_marker.pose.orientation.y = transform_orientation.y();
        transformed_marker.pose.orientation.z = transform_orientation.z();
        transformed_marker.pose.orientation.w = transform_orientation.w();

        transformed_markers.markers.push_back(transformed_marker);
    }
    pb_left.publish(transformed_markers);
};

void dynamic_calib::msg_Callback_right(const visualization_msgs::MarkerArray::ConstPtr& markerarray){
    Eigen::Matrix3f rotate_matrix = right_transform_.linear();
    Eigen::Matrix3f R_odom_curr_tmp;
    R_odom_curr_tmp= Eigen::Quaternionf(rotate_matrix).normalized();
    // 创建一个新的 MarkerArray 以保存变换后的对象
    visualization_msgs::MarkerArray transformed_markers;
    for(auto marker : markerarray->markers){
        // 创建一个新的 Marker 并进行复制
        visualization_msgs::Marker transformed_marker = marker;    
        Eigen::Vector4f pose;
        pose[0] = marker.pose.position.x;
        pose[1] = marker.pose.position.y;
        pose[2] = marker.pose.position.z;
        pose[3] = 1;
        Eigen::Vector4f trans_pose;
        trans_pose = right_transform_*pose;
        transformed_marker.pose.position.x = trans_pose[0];
        transformed_marker.pose.position.y = trans_pose[1];
        transformed_marker.pose.position.z = trans_pose[2];

        Eigen::Quaternionf q_odom_curr_tmp; 
        q_odom_curr_tmp.x() = marker.pose.orientation.x;
        q_odom_curr_tmp.y() = marker.pose.orientation.y;
        q_odom_curr_tmp.z() = marker.pose.orientation.z;
        q_odom_curr_tmp.w() = marker.pose.orientation.w;
        q_odom_curr_tmp.normalize();

        Eigen::Quaternionf transform_orientation;
        transform_orientation = R_odom_curr_tmp*q_odom_curr_tmp;
        transform_orientation.normalize();

        transformed_marker.pose.orientation.x = transform_orientation.x();
        transformed_marker.pose.orientation.y = transform_orientation.y();
        transformed_marker.pose.orientation.z = transform_orientation.z();
        transformed_marker.pose.orientation.w = transform_orientation.w();

        transformed_markers.markers.push_back(transformed_marker);
    }
    pb_right.publish(transformed_markers);
};

void dynamic_calib::msg_Callback_back(const visualization_msgs::MarkerArray::ConstPtr& markerarray){
    Eigen::Matrix3f rotate_matrix = back_transform_.linear();
    Eigen::Matrix3f R_odom_curr_tmp;
    R_odom_curr_tmp= Eigen::Quaternionf(rotate_matrix).normalized();
    // 创建一个新的 MarkerArray 以保存变换后的对象
    visualization_msgs::MarkerArray transformed_markers;
    for(auto marker : markerarray->markers){
        // 创建一个新的 Marker 并进行复制
        visualization_msgs::Marker transformed_marker = marker;    
        Eigen::Vector4f pose;
        pose[0] = marker.pose.position.x;
        pose[1] = marker.pose.position.y;
        pose[2] = marker.pose.position.z;
        pose[3] = 1;
        Eigen::Vector4f trans_pose;
        trans_pose = back_transform_*pose;
        transformed_marker.pose.position.x = trans_pose[0];
        transformed_marker.pose.position.y = trans_pose[1];
        transformed_marker.pose.position.z = trans_pose[2];

        Eigen::Quaternionf q_odom_curr_tmp; 
        q_odom_curr_tmp.x() = marker.pose.orientation.x;
        q_odom_curr_tmp.y() = marker.pose.orientation.y;
        q_odom_curr_tmp.z() = marker.pose.orientation.z;
        q_odom_curr_tmp.w() = marker.pose.orientation.w;
        q_odom_curr_tmp.normalize();

        Eigen::Quaternionf transform_orientation;
        transform_orientation = R_odom_curr_tmp*q_odom_curr_tmp;
        transform_orientation.normalize();

        transformed_marker.pose.orientation.x = transform_orientation.x();
        transformed_marker.pose.orientation.y = transform_orientation.y();
        transformed_marker.pose.orientation.z = transform_orientation.z();
        transformed_marker.pose.orientation.w = transform_orientation.w();

        transformed_markers.markers.push_back(transformed_marker);
    }
    pb_back.publish(transformed_markers);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "radar_calib");  //初始化
    dynamic_calib dc;
    dc.run();
    ros::spin();
    return 0;
}

