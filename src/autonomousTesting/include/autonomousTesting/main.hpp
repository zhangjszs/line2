/*
* @Author: Adams
* @Date:   2022-09-07 10:14:26
* @Email: adams_aka@126.com
* @Last Modified by:   Adams
* @Last Modified time: 2022-09-07 12:22:02
*/

#include "ros/ros.h"                
#include <chrono>
#include <iostream>
#include <common_msgs/vehicle_cmd.h>
#include <common_msgs/vehicle_status.h>
#include <boost/thread/thread.hpp>
#include <boost/functional.hpp>
#include <string>
using namespace std;
common_msgs::vehicle_cmd v_cmd;
common_msgs::vehicle_status status;
ros::Publisher pub;
bool ending = false;
bool timeStartCounting = true;
const int initSteering = 110;
const int interval = 30;
int first = 1;
						 // 110 -> 140 -> 90 -> 110
const int taskValue[4] = {initSteering,initSteering+interval,initSteering-2*interval,initSteering};
const int sinStatus[4] = {0,1,2,3};
int nowStatus = 0;
void start();
boost::thread *p_vehicle_contorl_thread = new boost::thread(boost::bind(&start));
auto startTime = std::chrono::steady_clock::now();
auto endTime = std::chrono::steady_clock::now();
int timeThre = 30;//30 s

void timeCheck(){
	if(timeStartCounting){
		startTime = std::chrono::steady_clock::now();
		timeStartCounting = false;
	}else{
		endTime = std::chrono::steady_clock::now();
		auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		if(elapsedTime.count()/1000>=timeThre){
			v_cmd.racing_status = 4; // finish
			pub.publish(v_cmd);
			ROS_INFO_STREAM("task compete!");
			// p_vehicle_contorl_thread->interrupt();
			// p_vehicle_contorl_thread->join();
			exit(0);
		}else{
			std::cout << "Time: "<<elapsedTime.count() / 1000<< " s" << std::endl;	
		}
	}
}
void sinSteering(){
	if(first){
		v_cmd.steering = 110;
		pub.publish(v_cmd);
		first = 0;
	}
	// auto elasepT = st - timeStart()
	else if( ((status.steering <=taskValue[0]+2) || status.steering >=taskValue[0]-2  ) && (nowStatus == sinStatus[0])){
		ROS_INFO_STREAM("task 1: turning right 30 degree...\n");
		v_cmd.steering = taskValue[1];
		pub.publish(v_cmd);
		nowStatus = 1;
	}
	else if( ((status.steering <=taskValue[1]+5) || status.steering >=taskValue[1]-5  ) && (nowStatus == sinStatus[1])){
		ROS_INFO_STREAM("task 1: turning left 60 degree...\n");
		v_cmd.steering = taskValue[2];
		pub.publish(v_cmd);
		nowStatus = 2;
	}else if( ((status.steering <=taskValue[2]+5) || status.steering >=taskValue[2]-5  ) && (nowStatus == sinStatus[2])){
		ROS_INFO_STREAM("task 1: turning right 30 degree!\n");
		v_cmd.steering = taskValue[3];
		pub.publish(v_cmd);
		nowStatus = 3;
	}else if( ((status.steering <=taskValue[3]+5) || status.steering >=taskValue[3]-5  ) && (nowStatus == sinStatus[2])){
		ROS_INFO_STREAM("task 1: turning left 30 degree!\n");
		nowStatus = 0;
	}
}