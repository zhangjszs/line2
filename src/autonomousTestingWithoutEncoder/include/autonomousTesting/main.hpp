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
bool sensorcheck = false;
const int initSteering = 110;
const int interval = 30;
const int steeringTimeInterval30 = 800;
int first = 1;
auto st = std::chrono::steady_clock::now();
auto et = std::chrono::steady_clock::now();
auto calcTime=std::chrono::duration_cast
	<std::chrono::milliseconds>(et - st);
						 // 110 -> 140 -> 90 -> 110
const int taskValue[4] = {initSteering,initSteering+interval,initSteering-2*interval,initSteering};
const int sinStatus[4] = {0,1,2,3};
int nowStatus = 0;
void start();
boost::thread *p_vehicle_contorl_thread = new boost::thread(boost::bind(&start));
auto startTime = std::chrono::steady_clock::now();
auto endTime = std::chrono::steady_clock::now();
int timeThre = 40;//30 s
void timeStart(auto &startTime){
	startTime = std::chrono::steady_clock::now();
}

void timeEnd(auto startTime,auto &calcTime){
	auto endTime = std::chrono::steady_clock::now();
	calcTime=std::chrono::duration_cast
	<std::chrono::milliseconds>(endTime - startTime);
}

void timeCheck(){
	if(timeStartCounting){
		startTime = std::chrono::steady_clock::now();
		timeStartCounting = false;
	}else{
		endTime = std::chrono::steady_clock::now();
		auto elapsedTime=std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
		if(elapsedTime.count()/1000>=timeThre){
			v_cmd.racing_status = 4; // finish
			v_cmd.pedal_ratio = 0;
			v_cmd.brake_force = 80;
			pub.publish(v_cmd);
			ROS_INFO_STREAM("task compete!");
			// p_vehicle_contorl_thread->interrupt();
			// p_vehicle_contorl_thread->join();
			// exit(0);
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
	std::cout<<"nowStatus:"<<nowStatus<<std::endl;
	if((nowStatus == sinStatus[0])){
		ROS_INFO_STREAM("task 1: turning right 30 degree...\n");
		v_cmd.steering = taskValue[1];
		pub.publish(v_cmd);
		timeStart(st);
		nowStatus = 1;
	}
	timeEnd(st,calcTime);
	if( calcTime.count() >=steeringTimeInterval30 && (nowStatus == sinStatus[1])){
		ROS_INFO_STREAM("task 1: turning left 60 degree! "<<calcTime.count()<<"\n");
		v_cmd.steering = taskValue[2];
		pub.publish(v_cmd);
		timeStart(st);
		nowStatus = 2;
	}
	timeEnd(st,calcTime);
	if(  calcTime.count() >=steeringTimeInterval30*2 && (nowStatus == sinStatus[2])){
		ROS_INFO_STREAM("task 1: turning right 30 degree! "<<calcTime.count()<<"\n");
		v_cmd.steering = taskValue[3];
		pub.publish(v_cmd);
		timeStart(st);
		nowStatus = 3;
	}
	timeEnd(st,calcTime);
	if( calcTime.count() >=steeringTimeInterval30&& (nowStatus == sinStatus[3])){
		ROS_INFO_STREAM("task 1: turning left 30 degree!\n");
		nowStatus = 0;
	}
}
void sensorcheckrun()
{
	sensorcheck = true;
}