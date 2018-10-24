#include <cstdio>
#include <iostream>
#include <cmath>
#include <thread>
#include <ctime>
#include <cstdbool>
#include <fstream>

#include "ekf_car.h"

#include "ros/ros.h"
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "ros/package.h"

#define EKF_RATE 100 //Hz
#define PRINT_RATE 100 //Hz


double v_in, phi_in;

//std::string bot_num, vicon_obj;

EKF car_ekf;
std::mutex ekf_mutex;
std::thread ekf_thread;

ros::Publisher state_pub;


std::string dir_path;
char time_buffer[80];


void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    geometry_msgs::Point point = pose.pose.position;
    
    ekf_mutex.lock();
    car_ekf.ViconUpdate(point.x, point.y, point.z);
    ekf_mutex.unlock();
}

void getIMUdata()
{
    ekf_mutex.lock();
    car_ekf.IMUUpdate(gyro, acc);
    ekf_mutex.unlock();
}

void getInputs(const AckermannMsgPtr& cmd)
{
    v_in = cmd->drive.speed;
    theta_in = cmd->drive.steering_angle;
}

/* Idea: add measurements to a vector (or augment matrix), and only update ekf once after the prediction step */
void estimator()
{
    ros::Rate r(EKF_RATE);

    while(ros::ok())
    {
        ekf_mutex.lock();
        car_ekf.stateEstimatorPredict(1./EKF_RATE, v_in, phi_in);
		
        vec3d_t pos = car_ekf.getLocation();
        // Need to convert phi into quaternion (assume roll=pitch=0, and yaw=phi)
        ekf_mutex.unlock();
        
        r.sleep();
    }
    
}

int main(int argc, char **argv)
{
    current_waypoint.x = current_waypoint.y = current_waypoint.z = 0;
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n("~");
    
    //n.param<std::string>("vicon_obj", vicon_obj, "f1car");
    //n.param<std::string>("bot_num", bot_num, "bot0");

    state_pub = n.advertise<geometry_msgs::Pose>("/carPose", 1);

    ros::Subscriber vicon_sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getViconPosition);
    ros::Subscriber imu_sub = n.subscribe("/imu/data", 1, getIMUdata);
    ros::Subscriber inputs = n.subscribe("/ackermann_cmd", 1, getInputs); 

    
    ekf_thread = std::thread(estimator);

    ros::spin();
    
    ekf_thread.join();
    
    
    return 0;
}
