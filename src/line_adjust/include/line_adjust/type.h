
#ifndef TYPE_H
#define TYPE_H
#include <iostream>
#include <utility>
#include "std_msgs/Float64MultiArray.h"

// custom messages
/*#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"*/
#include "huat_msgs/HUAT_CarState.h"
#include "huat_msgs/HUAT_Map.h"
#include "huat_msgs/HUAT_ControlCommand.h"
// STL
#include <cmath>
#include <vector>

namespace line_creat{
    struct VehicleState {
        double x;
        double y;
        double yaw;
        double v;
        double r;
        double a;
        double w;
        double Delta;
        double D;

        VehicleState(huat_msgs::HUAT_CarState state,huat_msgs::HUAT_ControlCommand cmd) {
            x = state.car_state.x;                                       //车辆横坐标
            y = state.car_state.y;                                       //车辆纵坐标
            yaw = state.car_state.theta;                         //车辆的航向角(朝向) 
           v = std::hypot(state.car_state.x, state.car_state.y);       //车辆的速度 
            r = state.car_state.theta;                                                                                            //车辆的角速度
            a = std::hypot(state.car_state.x, state.car_state.y);         //车辆的加速度
            w =  state.car_state.theta;                                                                                            //车辆的角加速度

           D = cmd.throttle.data;                                                                                                                                   //车辆的油门输出
           Delta = cmd.steering_angle.data;                                                                                                            //车辆的油门转向输出
        }

        VehicleState() {

        }
    };
   typedef struct Boundary{
     double x;
     double y;
   } Point2f;
   
    struct TrajectoryPoint {
        Point2f pts;                    //包含x,y
        double yaw;              
        double curvature;             //曲率
        double velocity;
        double r;
        double acc;
    };

    typedef std::vector<TrajectoryPoint> Trajectory;

}

#endif