#include "ros/ros.h"                 //引入ros头文件
#include<iostream> //C++标准输入输出库
#include "std_msgs/String.h"         //引入msg
#include <geometry_msgs/Twist.h>                                              // 订阅的cmd消息格式
#include "huat_msgs/HUAT_ASENSING.h"
#include "huat_msgs/HUAT_VehcileCmd.h"
#include "huat_msgs/HUAT_ControlCommand.h"
#include<cmath>
#include "fstream"

using namespace std;
huat_msgs::HUAT_VehcileCmd vehicle_cmd_msg;     //传感器消息格式
ros::Publisher vehicle_cmd_msg_pub;                              //发布方 跟车最终的通信
// 速度  刹车力 踏板比例 齿轮位置 工作模式 比赛号码 赛车状态
int my_steering,my_brake_force,my_pedal_ratio,my_gear_position,my_working_mode,my_racing_num,my_racing_status;
long my_checksum;             //检查数
int input_mode=2;               //输入模式 分为1 2 3 4测试，直线，八字，高速

float longError,longCurrent,longLast,currentSpeed;

void angleCallback(huat_msgs::HUAT_ControlCommand input_angle)
{
    cout<<"read!"<<endl;
    if(input_mode == 2)//line mode           
    {
	    my_gear_position = 1;                //齿轮
        my_working_mode = 1;
        my_steering = int(input_angle.steering_angle.data*180/3.14159265358979*3.73)+110;//这里的角度数加了110  他这里110才是中心
        cout<<"my_steering "<<my_steering<<endl;                               //转角
        my_pedal_ratio = int(input_angle.throttle.data);                       //踏板率
        cout<<"my_pedal_ratio "<<my_pedal_ratio<<endl;
        ofstream m;
        m.open("/home/tb/line_creat2/zhuanjiao.txt",ios::out|ios::app);      
	    m << std::setprecision(20);           
        m<<my_steering<<endl;
        m.close();

/////////////关于转角的判断///////////////////////
        if(my_steering <0 )
        {
            my_steering = 0;
        }
        else if(my_steering > 220)
        {
            my_steering = 220;
        }

        if(my_pedal_ratio <0 )                 //踏板率
        {
            my_pedal_ratio = 0;
        }
        else if(my_pedal_ratio > 100)           //踏板率
        {
            my_pedal_ratio = 100;
        }

        if(input_angle.racing_status ==  4)
        {
            my_brake_force = 85;
            my_racing_status = 4;
        }
        vehicle_cmd_msg.head1 = 0XAA;
        vehicle_cmd_msg.head2 = 0X55;
        vehicle_cmd_msg.length = 10;
        vehicle_cmd_msg.steering = my_steering;
        vehicle_cmd_msg.brake_force = my_brake_force;
        vehicle_cmd_msg.pedal_ratio = my_pedal_ratio;
        vehicle_cmd_msg.gear_position = my_gear_position;
        vehicle_cmd_msg.working_mode = my_working_mode;
        vehicle_cmd_msg.racing_num = my_racing_num;
        vehicle_cmd_msg.racing_status = my_racing_status;
        vehicle_cmd_msg.checksum = my_steering+my_brake_force+my_pedal_ratio+my_gear_position+my_working_mode+my_racing_num+my_racing_status;
        vehicle_cmd_msg_pub.publish(vehicle_cmd_msg);

    }
}

int main(int argc, char **argv)
{
    ofstream m;
    m.open("/home/tb/line_creat2/zhuanjiao.txt",ios::out);      
	m.close();
    // 初始化ROS节点
    ros::init(argc, argv, "ros_controller");
    ros::Time::init();
    ros::Rate loop_rate(10);                                         //原为1


    // 创建节点句柄
    ros::NodeHandle n;
    vehicle_cmd_msg_pub = n.advertise<huat_msgs::HUAT_VehcileCmd>("vehcileCMDMsg", 1000);
    ros::Subscriber sub2 = n.subscribe("/control/pure_pursuit/control_command", 1, angleCallback);

    vehicle_cmd_msg.head1 = 0XAA;
    vehicle_cmd_msg.head2 = 0X55;
    vehicle_cmd_msg.length = 10;
    vehicle_cmd_msg.steering = 90;
    vehicle_cmd_msg.brake_force = 0;
    vehicle_cmd_msg.pedal_ratio = 0;
    vehicle_cmd_msg.gear_position = 0;
    vehicle_cmd_msg.working_mode = 0;
    vehicle_cmd_msg.racing_num = 0;
    vehicle_cmd_msg.racing_status = 0;
    vehicle_cmd_msg.checksum = 100;

    my_steering = 90;
    my_brake_force = 0;
    my_pedal_ratio = 0;
    my_gear_position = 0;
    my_working_mode = 0;
    my_racing_num = 0;
    my_racing_status = 0;
    my_checksum = 100;

    ros::spin();

    return 0;


}
