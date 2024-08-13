#include <ros/ros.h>
#include "radar_ros/ObjectList.h"
#include "radar_ros/Object.h"
#include "radar_ros/ClusterList.h"
#include "radar_ros/Cluster.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "radar_ros/dbscan.h"
#include <limits>
#include <unordered_map>
#include <cmath>

#define MINIMUM_POINTS 4     // minimum number of cluster
#define EPSILON (0.75*0.75)  // distance for clustering, metre^2

struct ClusterInfo {
    float minX, maxX, minY, maxY, minZ, maxZ;
    float sumVx, sumVy, sumRcs;
    int pointCount;

    ClusterInfo() {
        minX = minY = minZ = std::numeric_limits<float>::max();
        maxX = maxY = maxZ = std::numeric_limits<float>::lowest();
        sumVx = sumVy = 0.0;
        pointCount = 0;
    }

    void update(const Point_& point) {
        if (point.x < minX) minX = point.x;
        if (point.x > maxX) maxX = point.x;
        if (point.y < minY) minY = point.y;
        if (point.y > maxY) maxY = point.y;
        if (point.z < minZ) minZ = point.z;
        if (point.z > maxZ) maxZ = point.z;

        sumVx += point.vx;
        sumVy += point.vy;
        sumRcs += point.rcs;
        pointCount++;
    }

    float centerX() const { return (minX + maxX) / 2.0f; }
    float centerY() const { return (minY + maxY) / 2.0f; }
    float centerZ() const { return (minZ + maxZ) / 2.0f; }
    float length() const { return maxX - minX; }
    float width() const { return maxY - minY; }
    float avgVx() const { return sumVx / pointCount; }
    float avgVy() const { return sumVy / pointCount; }
    float avgRcs() const { return sumRcs / pointCount; }
};

// 计算两个框之间的IoU
float calculateIoU(const radar_ros::Object& obj1, const radar_ros::Object& obj2) {
    float x1_min = obj1.position.pose.position.x - obj1.length / 2;
    float x1_max = obj1.position.pose.position.x + obj1.length / 2;
    float y1_min = obj1.position.pose.position.y - obj1.width / 2;
    float y1_max = obj1.position.pose.position.y + obj1.width / 2;

    float x2_min = obj2.position.pose.position.x - obj2.length / 2;
    float x2_max = obj2.position.pose.position.x + obj2.length / 2;
    float y2_min = obj2.position.pose.position.y - obj2.width / 2;
    float y2_max = obj2.position.pose.position.y + obj2.width / 2;

    float intersect_x_min = std::max(x1_min, x2_min);
    float intersect_y_min = std::max(y1_min, y2_min);
    float intersect_x_max = std::min(x1_max, x2_max);
    float intersect_y_max = std::min(y1_max, y2_max);

    float intersect_width = std::max(0.0f, intersect_x_max - intersect_x_min);
    float intersect_height = std::max(0.0f, intersect_y_max - intersect_y_min);
    float intersection_area = intersect_width * intersect_height;

    float area1 = obj1.length * obj1.width;
    float area2 = obj2.length * obj2.width;

    return intersection_area / (area1 + area2 - intersection_area);
}

void standardNMS(radar_ros::ObjectList& objects, float iou_threshold) {
    std::vector<radar_ros::Object> nms_objects;
    
    // 根据 prob_of_exist 从高到低排序
    std::sort(objects.objects.begin(), objects.objects.end(),
              [](const radar_ros::Object& a, const radar_ros::Object& b) {
                  return a.prob_of_exist > b.prob_of_exist;
              });

    std::vector<bool> suppressed(objects.objects.size(), false);

    for (size_t i = 0; i < objects.objects.size(); ++i) {
        if (suppressed[i]) continue;

        nms_objects.push_back(objects.objects[i]);
        for (size_t j = i + 1; j < objects.objects.size(); ++j) {
            if (suppressed[j]) continue;

            float iou = calculateIoU(objects.objects[i], objects.objects[j]);
            if (iou > iou_threshold) {
                suppressed[j] = true;
            }
        }
    }

    objects.objects = std::move(nms_objects);
}

void objectCallback(const radar_ros::ObjectList::ConstPtr& msg1,
                   const radar_ros::ObjectList::ConstPtr& msg2,
                   const radar_ros::ObjectList::ConstPtr& msg3,
                   const radar_ros::ObjectList::ConstPtr& msg4,
                   ros::Publisher& pub) {
    radar_ros::ObjectList combined_msg;
    combined_msg.header.stamp = ros::Time::now();
    combined_msg.header.frame_id = "rslidar";

    combined_msg.objects.insert(combined_msg.objects.end(), msg1->objects.begin(), msg1->objects.end());
    combined_msg.objects.insert(combined_msg.objects.end(), msg2->objects.begin(), msg2->objects.end());
    combined_msg.objects.insert(combined_msg.objects.end(), msg3->objects.begin(), msg3->objects.end());
    combined_msg.objects.insert(combined_msg.objects.end(), msg4->objects.begin(), msg4->objects.end());

    float iou_threshold = 0.01; // IoU 阈值

    standardNMS(combined_msg, iou_threshold);

    pub.publish(combined_msg);
}

std::unordered_map<int, ClusterInfo> calculateClusterBoundingBoxes(const std::vector<Point_>& m_points) {
    std::unordered_map<int, ClusterInfo> clusterInfoMap;

    for (const auto& point : m_points) {
        clusterInfoMap[point.clusterID].update(point);
    }

    return clusterInfoMap;
}

void clusterCallback(const radar_ros::ClusterList::ConstPtr& msg1,
                   const radar_ros::ClusterList::ConstPtr& msg2,
                   const radar_ros::ClusterList::ConstPtr& msg3,
                   const radar_ros::ClusterList::ConstPtr& msg4,
                   ros::Publisher& pub, ros::Publisher& pub2)
{
    radar_ros::ClusterList combined_msg;
    combined_msg.header.stamp = ros::Time::now();
    combined_msg.header.frame_id = "rslidar";

    // 合并数据
    combined_msg.clusters.insert(combined_msg.clusters.end(), msg1->clusters.begin(), msg1->clusters.end());
    combined_msg.clusters.insert(combined_msg.clusters.end(), msg2->clusters.begin(), msg2->clusters.end());
    combined_msg.clusters.insert(combined_msg.clusters.end(), msg3->clusters.begin(), msg3->clusters.end());
    combined_msg.clusters.insert(combined_msg.clusters.end(), msg4->clusters.begin(), msg4->clusters.end());

    std::vector<Point> points;

    for(auto cluster : combined_msg.clusters){
        Point point;
        point.x = cluster.position.pose.position.x;
        point.y = cluster.position.pose.position.y;
        point.z = cluster.position.pose.position.z;
        point.vx = cluster.relative_velocity.twist.linear.x;
        point.vy = cluster.relative_velocity.twist.linear.y;
        point.rcs = cluster.rcs;
        point.clusterID = UNCLASSIFIED;
        points.push_back(point);
    }
    DBSCAN ds(MINIMUM_POINTS, EPSILON, points);//dbscan聚类
    ds.run();
    
    auto clusterInfoMap = calculateClusterBoundingBoxes(ds.m_points);

    radar_ros::ObjectList cluster_msgs;
    cluster_msgs.header = combined_msg.header;
    for (const auto& [clusterID, info] : clusterInfoMap) {
        radar_ros::Object cluster_msg;
        cluster_msg.width = info.width();
        cluster_msg.length = info.length();
        cluster_msg.orientation_angle = 0;
        cluster_msg.rcs = info.avgRcs();
        cluster_msg.id = clusterID;
        cluster_msg.relative_velocity.twist.linear.x = info.avgVx();
        cluster_msg.relative_velocity.twist.linear.y = info.avgVy();
        cluster_msg.position.pose.position.x = info.centerX();
        cluster_msg.position.pose.position.y = info.centerY();
        cluster_msg.position.pose.position.z = info.centerZ();
        cluster_msg.position.pose.orientation.w = 1;
        cluster_msg.position.pose.orientation.x = 0;
        cluster_msg.position.pose.orientation.y = 0;
        cluster_msg.position.pose.orientation.z = 0;
        cluster_msg.class_type = 0;
        cluster_msg.prob_of_exist = 1;
        cluster_msgs.objects.push_back(cluster_msg);
    }


    pub.publish(combined_msg);
    pub2.publish(cluster_msgs);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_data_combiner");
    ros::NodeHandle nh;
    std::string combined_object_front;
    std::string combined_object_left;
    std::string combined_object_right;
    std::string combined_object_back;
    std::string combined_cluster_front;
    std::string combined_cluster_left;
    std::string combined_cluster_right;
    std::string combined_cluster_back;
    nh.param<std::string>("radar_output_object_topic_front",combined_object_front,"/ars_40X/filter/objects_front");
    nh.param<std::string>("radar_output_object_topic_left",combined_object_left,"/ars_40X/filter/objects_left");
    nh.param<std::string>("radar_output_object_topic_right",combined_object_right,"/ars_40X/filter/objects_right");
    nh.param<std::string>("radar_output_object_topic_back",combined_object_back,"/ars_40X/filter/objects_back");

    nh.param<std::string>("radar_output_cluster_topic_front",combined_cluster_front,"/ars_40X/filter/clusters_front");
    nh.param<std::string>("radar_output_cluster_topic_left",combined_cluster_left,"/ars_40X/filter/clusters_left");
    nh.param<std::string>("radar_output_cluster_topic_right",combined_cluster_right,"/ars_40X/filter/clusters_right");
    nh.param<std::string>("radar_output_cluster_topic_back",combined_cluster_back,"/ars_40X/filter/clusters_back");
    message_filters::Subscriber<radar_ros::ObjectList> sub_object_1(nh, combined_object_front, 1);
    message_filters::Subscriber<radar_ros::ObjectList> sub_object_2(nh, combined_object_left, 1);
    message_filters::Subscriber<radar_ros::ObjectList> sub_object_3(nh, combined_object_right, 1);
    message_filters::Subscriber<radar_ros::ObjectList> sub_object_4(nh, combined_object_back, 1);

    message_filters::Subscriber<radar_ros::ClusterList> sub_cluster_1(nh, combined_cluster_front, 1);
    message_filters::Subscriber<radar_ros::ClusterList> sub_cluster_2(nh, combined_cluster_left, 1);
    message_filters::Subscriber<radar_ros::ClusterList> sub_cluster_3(nh, combined_cluster_right, 1);
    message_filters::Subscriber<radar_ros::ClusterList> sub_cluster_4(nh, combined_cluster_back, 1);

    // 使用ApproximateTimeSynchronizer进行时间近似同步
    typedef message_filters::sync_policies::ApproximateTime<radar_ros::ObjectList, radar_ros::ObjectList, radar_ros::ObjectList, radar_ros::ObjectList> MySyncPolicy_object;
    message_filters::Synchronizer<MySyncPolicy_object> sync_object(MySyncPolicy_object(10), sub_object_1, sub_object_2, sub_object_3, sub_object_4); 
    ros::Publisher combined_object_pub = nh.advertise<radar_ros::ObjectList>("/ars_40X/combined_objects", 10);
    sync_object.registerCallback(boost::bind(&objectCallback, _1, _2, _3, _4, boost::ref(combined_object_pub)));

    typedef message_filters::sync_policies::ApproximateTime<radar_ros::ClusterList, radar_ros::ClusterList, radar_ros::ClusterList, radar_ros::ClusterList> MySyncPolicy_cluster;
    message_filters::Synchronizer<MySyncPolicy_cluster> sync_cluster(MySyncPolicy_cluster(10), sub_cluster_1, sub_cluster_2, sub_cluster_3, sub_cluster_4); 
    ros::Publisher combined_cluster_pub = nh.advertise<radar_ros::ClusterList>("/ars_40X/combined_clusters", 10);
    ros::Publisher combined_dbscan_pub = nh.advertise<radar_ros::ObjectList>("/ars_40X/combined_dbscan", 10);
    sync_cluster.registerCallback(boost::bind(&clusterCallback, _1, _2, _3, _4, boost::ref(combined_cluster_pub), boost::ref(combined_dbscan_pub)));

    ros::spin();

    return 0;
}
