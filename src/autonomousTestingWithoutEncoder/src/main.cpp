/*
* @Author: Adams
* @Date:   2022-09-07 10:14:26
* @Email: adams_aka@126.com
* @Last Modified by:   Adams
* @Last Modified time: 2022-09-07 12:22:02
*/
#include <main.hpp>
using namespace std;
void init(){

	       //  v_cmd.steering = steering;
        // v_cmd.pedal_ratio = currentSpeed;
        // v_cmd.working_mode = 0x01;
        // v_cmd.gear_position = 0x01;
        // v_cmd.racing_num = 0x02;
        // v_cmd.brake_force = brake_force;
	v_cmd.brake_force = 0;
	v_cmd.pedal_ratio = 5;
	v_cmd.gear_position = 0x01;// straight forward
	v_cmd.working_mode = 0x00;//autonomous
	// v_cmd.racing_status = 3;
	// v_cmd.racing_num = 3;// autonomous testing

	// status.work_mode = 0;
}
void start(){
	ROS_INFO_STREAM("thread started");
	while(!ending){
		boost::this_thread::sleep(boost::posix_time::milliseconds(20));
		//status.work_mode == 1
		if(1){
			v_cmd.working_mode = 0x01;
			pub.publish(v_cmd); // start electric machinery
			timeCheck();
			sinSteering();
		}
		else{
			// ROS_INFO_STREAM("working_mode:"<<status.working_mode);
			ROS_INFO_STREAM("waiting... working_mode:"<<(int)status.work_mode<<" racing_num:"<<(int)status.racing_num);
		}
	}
}
void callback(const common_msgs::vehicle_statusConstPtr& s){
	status = *s;
	int steer = status.steering;
	// ROS_INFO_STREAM("currentSteer; command");
	// ROS_INFO_STREAM(status.steering<<" "<<status.command);
	ROS_INFO_STREAM("Current Steering: " << status.steering << ", Command: " << status.command);
}

int main(int argc, char **argv) {
    if (sensorcheckrun()) {
        ros::init(argc, argv, "autonomousTesting");
        ros::NodeHandle n;
        init();
        pub = n.advertise<common_msgs::vehicle_cmd>("vehcileCMDMsg", 5);
        ros::Subscriber testing_sub = n.subscribe("vehicleStatusMsg", 10, &callback);
        ros::Rate loop_rate(100);
        ros::spin();
    } else {
        ROS_INFO("Sensor check is not true. Exiting...");
    }

    return 0;
}

