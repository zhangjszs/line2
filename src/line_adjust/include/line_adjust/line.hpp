#ifndef LINE_HPP
#define LINE_HPP

#include "ros/ros.h"
#include "iostream"
#include "mutex"
#include <line_adjust/type.h>
#include <geometry_msgs/PoseStamped.h>
#include "huat_msgs/HUAT_ins_p2.h"
#include "huat_msgs/Cone.h"
#include "huat_msgs/HUAT_rviz.h"
using namespace std;
namespace line_creat{
  class Line
  {
    public:
    Line(ros::NodeHandle node);
  void createPath();
  bool genTraj();
  ~Line();
   Trajectory trajectory_;
    bool getPath = false;                                                                       //判断
    double allow_angle_error = 1.0;                                              //转角允许误差
    huat_msgs::Cone cluster;                               
    ros::Subscriber cone_points_set_;
    ros::NodeHandle nh_;                                 
    ros::Subscriber ins;
    std::mutex line_mutex;
    ros::Publisher pub_zuobiao;
    geometry_msgs::Point end_point;
    void cone_position_callback(const huat_msgs::ConePtr& cone_position);

    void inscallback(const huat_msgs::HUAT_ins_p2& ins_);
    int flag1=0;                                                                //结束的标志
    int flag2=0;
    double thera;
    double hode_speed;                                            //理想速度
    int num_N;                                                               //次数N
    double distance;                                                    //间距
    //double time_dt;                                                     //时间步长
    //double max_lat_acc;
    huat_msgs::HUAT_rviz result;
  };
}
#endif