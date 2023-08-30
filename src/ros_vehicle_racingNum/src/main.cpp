/*
Author: Adams
Time: 22/3/25
*/

#include <sstream>
#include "ros/ros.h"                 //引入ros头文件
#include<iostream> //C++标准输入输出库
#include <fstream>
#include<string> //C++标准输入输出库
#include <common_msgs/vehicle_status.h>


using namespace std;

// outfile.precision(10);
void racingNumCmdCallback(const common_msgs::vehicle_status v_status){
    int racingNum = int(v_status.racing_num);
    cout<<"racingNum:"<<racingNum<<endl;

    // ofstream outfile("~/autoStartGkj/command",ofstream::out | ofstream::trunc);
    ofstream outfile("/home/tb/autoStartGkj/command",ofstream::trunc);
    // if(racingNum !=0){
    if (outfile.is_open()){
        cout<<"File Open Succ!"<<endl;
        outfile << racingNum;
        // cout<<"."<<endl;
        outfile.close();
        if(racingNum!=0)
            exit(0);
    }else{
        cout<<"File Open Failed!"<<endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "racingNumCmd");
    ros::NodeHandle n;
    ros::Subscriber ins_sub = n.subscribe("vehicleStatusMsg", 1, racingNumCmdCallback);
    ros::spin();
    return 0;
}
