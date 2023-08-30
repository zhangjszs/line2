#include "ros/ros.h"
#include "huat_msgs/HUAT_rviz.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include  "visualization_msgs/Marker.h"
#include "sensor_msgs/PointCloud2.h"
ros::Publisher state_pub; // 实例化发布者对象1
ros::Publisher marker_pub;
nav_msgs::Path ros_path;
ros::Publisher pub;
void Draw(const huat_msgs::HUAT_rvizConstPtr position)
{
  //路径
  ros_path.header.frame_id ="velodyne";
  ros_path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose1;
  pose1.header = ros_path.header;
  pose1.pose.position.x = position->finally.x;
  pose1.pose.position.y = position->finally.y;
  pose1.pose.position.z = position->finally.z;
  ros_path.poses.push_back(pose1);
  state_pub.publish(ros_path);
 
    //桶锥
    int temp;
  temp=position->points.size();
  static int codes=0;
  for(int i=0;i<temp;i++)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id ="velodyne";
  marker.header.stamp = ros::Time::now();
  marker.id = i;                             //指定唯一的编号
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = position->points[i].x;
  marker.pose.position.y = position->points[i].y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z =0.5;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}
}
void doMsg(const sensor_msgs::PointCloud2ConstPtr dian)
{
   sensor_msgs::PointCloud2 temp;
   temp=*dian;
   temp.header.frame_id="velodyne";
   pub.publish(temp);
}
int main(int argc, char *argv[])
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "line_rviz");
  ros::NodeHandle nh;
  ros::Subscriber display;
  ros::Subscriber dianyun;
  display = nh.subscribe("/line_control/zuobiao", 1, Draw);
    dianyun=nh.subscribe("/velodyne_points",10,doMsg);
  state_pub = nh.advertise<nav_msgs::Path>("gps_path", 10);
  marker_pub=nh.advertise<visualization_msgs::Marker>("gps_point",10);
 pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
  ros::spin();
  return 0;
}