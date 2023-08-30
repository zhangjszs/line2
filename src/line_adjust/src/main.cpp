#include "ros/ros.h"
#include "line_adjust/line.hpp"
#include "iostream"
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // 转换函数所需的头文件
#include <cmath>
using namespace std;

vector<vector<double>>temp_points; 
vector<vector<double>>temp_points1;
double correct_x=0,correct_y=0;
double temp_licsence=0;
namespace line_creat{
  Line::Line(ros::NodeHandle node)
  {
     // 在构造函数中可以初始化一些成员变量或执行其他必要的操作
     nh_=node;
      hode_speed=4.0;
     distance=0.05;
     num_N=40;
  }  
  void Line::createPath() {
    // 实现路径创建的方法
    line_mutex.lock();
   if(cluster.points.size() == 0)
      {
        //cout<<"数据为空"<<endl;
        line_mutex.unlock();
            return;
      } 
    int accumulator[180][201]={0};
    double p,p1,p2,Y_right,Y_left;
    int theta1,theta2;
    for(int i=0; i<cluster.points.size();i++)
    {
        if(cluster.points[i].y > 2 || cluster.points[i].y < -2)
            continue;
        for (int j=0; j<180; j++)
        {
            p=(cluster.points[i].x * cos(j * M_PI / 180)+cluster.points[i].y*sin(j * M_PI / 180))*5;
            if(p > 100)
                p = 100;
            accumulator[j][(int)p+100]+=1;            //累加
       }
    }
    int max1 = 0;
    int max2 = 0;
    for(int i = 90 - allow_angle_error; i < 90 + allow_angle_error; i++)
    {
        for(int j = 0; j < 100; j++)
        {
            if(accumulator[i][j] >= max1)
            {
                max1 = accumulator[i][j];
                p1=((float)j-100)/5;
                theta1=i;
            }
        }
    }

    for(int i = 90 - allow_angle_error; i < 90 + allow_angle_error; i++)//角度阈值
    {
        for(int j = 100; j < 200; j++)
        {
            if(accumulator[i][j] >= max2)
            {
                max2 = accumulator[i][j];
                p2=((float)j-100)/5;
                theta2=i;
            }
        }
    }

    if (theta1==theta2)
	{
		if  (fabs(p1)<2 && fabs(p2)<2 )
        {
            getPath=true;
            cout<<"find ideal path"<<endl;
        }
	}
    else
    {
        double check_x=(p1*cos((float)theta2*M_PI/180.0)-p2*cos((float)theta1*M_PI/180.0))/(sin((float)theta1*M_PI/180.0)*cos((float)theta2*M_PI/180.0)-sin((float)theta2*M_PI/180.0)*cos((float)theta1*M_PI/180.0));
        if ((check_x > 200 || check_x < -200)&&(fabs(p1)<3 && fabs(p2)<3))//直线距离车小鱼于3米
        {
            getPath=true;
			//cout<<"find path"<<endl;
        }
        else
        {
            getPath=false;
            //cout<<"未找到"<<endl;
            line_mutex.unlock();
            return;
        }
    }
    double tmp=cluster.points[0].x;
    for(int i=1;i<cluster.points.size();i++)
    {
        if(fabs(cluster.points[i].y)<=2)
            {
                   if(fabs(cluster.points[i].x>tmp))
                   {
                    tmp=cluster.points[i].x;
                   }
            }
    }
   int path_length1=tmp;
    Y_right = (p1-path_length1*cos((float)theta1*M_PI/180.0))/sin((float)theta1*M_PI/180.0);
    Y_left = (p2-path_length1*cos((float)theta2*M_PI/180.0))/sin((float)theta2*M_PI/180.0);
    
    double temp1=(Y_left+Y_right)/2;
    if(fabs(temp1)<=2&&flag1==0)
         {
             end_point.x = path_length1;
             end_point.y = temp1;
             flag1=1;
         }
    else
   { 
        //cout<<"跳出"<<endl;
        line_mutex.unlock();
        return;
   } 
   
    cout<<end_point.x<<"  "<<end_point.y<<endl;
    line_mutex.unlock();
  }
  Line::~Line() {
    // 析构函数，可以在这里进行一些资源的释放操作
  }
  void Line::cone_position_callback(const huat_msgs::ConePtr& cone_position){
   cluster=*cone_position;
   
  /*for(int i=0;i<cluster.points.size();i++)
   {
    cout<<cluster.points[i].x<<"     "<<cluster.points[i].y<<endl;

    }*/
    createPath();
    correct_x=1.87*sin(thera);
    correct_y=1.87*cos(thera);
   while(1)
    {
         if(temp_licsence>85)
          {
                /*             
                for(int i=0;i<temp_points1.size();i++)
                {
                    //temp_points1[i][0]=temp_points1[i][0]+1.87* sin(thera);
                    //temp_points1[i][1]=temp_points1[i][1]+1.87*cos(thera);
                    m<<"  "<<temp_points1[i][0]<<"   "<<temp_points1[i][1]<<endl;
                    result.finally.x=temp_points1[i][0];
                    result.finally.y=temp_points1[i][1];
                    pub_zuobiao.publish(result);
                }
                */
                break;
          }
           genTraj();
           ofstream m;
           m.open("/home/tb/line_creat2/duan.txt",ios::out|ios::app);         
           m << std::setprecision(20);      
           for(int i=0;i<temp_points.size();i++)
           {
                temp_points[i][0]=temp_points[i][0]+correct_x;      
                //temp_points[i][1]=temp_points[i][1]+correct_y;     
                m<<"  "<<temp_points[i][0]<<"   "<<temp_points[i][1]<<endl;
                //temp_points1.push_back({temp_points[i][0],temp_points[i][1]});
                result.finally.x=temp_points[i][0];
                result.finally.y=temp_points[i][1];
                pub_zuobiao.publish(result);
           }
           m.close();
          correct_x=temp_points[temp_points.size()-1][0];
          correct_y=temp_points[temp_points.size()-1][1];
          temp_licsence=correct_x*correct_x+correct_y*correct_y-1.87;
          temp_licsence=sqrt(temp_licsence);
          temp_points.clear();
    }
  } 
bool Line::genTraj() {
    // 实现分割逻辑
        const double interval = distance;                                                                //interval轨迹点间隔
        const double desire_vel = hode_speed;                                                 //desire_vel期望速度

        double temp_distance = std::hypot(end_point.x, end_point.y);   //计算从起点到终点得距离
        double gradient =end_point.y/end_point.x;                                      //计算斜率
        TrajectoryPoint tmp_pt;
        trajectory_.clear();                                                                                           //清空轨迹点，准备存储新的轨迹点
        if (interval == 0){                                                                                                //判断间隔是否过小
            ROS_ERROR("Interval is too small.");
            return false;
        }                                                                                                                                //开始分割
        for (double i = 0; i < temp_distance; i=i+interval) {                            // y = kx and i^2 = x^2 + y^2
            tmp_pt.pts.x = i / hypot(1.0, gradient);                                               //x=i*cos(angle)
            tmp_pt.pts.y = tmp_pt.pts.x * gradient;                                             //y=x*tan(angle)
            tmp_pt.yaw = atan2(tmp_pt.pts.y, tmp_pt.pts.x);                        //偏航角=arctan(tan(angle))
            tmp_pt.curvature = 0;                                                                               //曲率设为0
            tmp_pt.velocity = desire_vel;                                                                //期望速度
            tmp_pt.r = 0;                                                                                                 //曲率半径设为0
            trajectory_.push_back(tmp_pt);                                                         //存放容器里
            double temp_x, temp_y;                                                                        //进行了一个坐标系的转换
            temp_x = tmp_pt.pts.x *cos(thera);
            temp_y = tmp_pt.pts.y *sin(thera);
            temp_points.push_back({temp_x,temp_y});
        }
        return true;
}
void Line::inscallback(const huat_msgs::HUAT_ins_p2& ins_)
{
              thera=ins_.Heading;
              flag2=1;
}
}
int main(int argc, char *argv[]) {
  ofstream m;
  m.open("/home/tb/line_creat2/duan.txt",ios::out); 
  m.close();      
  setlocale(LC_ALL,"");
  ros::init(argc, argv, "Line_creat");
  ros::NodeHandle nodeHandle("~");
  line_creat::Line myLine(nodeHandle);
  myLine.cone_points_set_ = nodeHandle.subscribe("/cone_position",10, &line_creat::Line::cone_position_callback, &myLine);	
  myLine.pub_zuobiao=nodeHandle.advertise<huat_msgs::HUAT_rviz>("/line_control/zuobiao",1);
  myLine.ins=nodeHandle.subscribe("/insMsg",1,&line_creat::Line::inscallback,&myLine);
  ros::spin();
  return 0;
}