#include <cstdio>
#include <iostream>
#include <cmath>
#include <thread>
#include <ctime>
#include <cstdbool>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "MPC.h"
#include "ros/ros.h"
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "ros/package.h"
#include <chrono>

#define WP_RATE 10.0 //Hz
#define PRINT_RATE 50.0 //Hz

#define DELTA_DIRECTION  0.01
#define DELTA_SPEED      0.25
#define EPSILON_RADIUS   0.25
#define EPSILON_ANGLE    0.1

bool isDriving = false;
bool gotWP = false;
bool slow_flag = false;
double speed = 0, direction = 0;
geometry_msgs::Point curr_loc;
double curr_ang = 0;

std::string bot_num, vicon_obj;
std::vector<geometry_msgs::Point> waypoints;

ros::Publisher drive_pub;
ros::Publisher reached_pub;

geometry_msgs::Point vicon_position;
geometry_msgs::Point current_waypoint;  // VICON coords
geometry_msgs::Quaternion quat; //Get orientation
geometry_msgs::Vector3 vicon_vel;

std::string dir_path;
char time_buffer[80];
std::thread drive_thread, print_thread, cmd_thread;
ros::Time wp_time;
ros::Time slow_time;

Eigen::Vector3d state;

Eigen::Quaterniond quat_eig;
Eigen::Vector3d vel_eig, vel_tf;
double vel_error_int = 0, vel_error_deriv = 0;
const double Kp = 1, Ki = 1; Kd = 0.1;

void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_position = pose.pose.position;
    quat = pose.pose.orientation;
}

void getViconVel(const geometry_msgs::TwistStamped& data)
{
    vicon_vel = data.twist.linear;
}

inline double goalDist(const geometry_msgs::Point pos, const geometry_msgs::Point goal)
{
    return sqrt(pow(pos.x - goal.x, 2) + pow(pos.y - goal.y, 2));
}

void drive()
{
    ros::Rate r(WP_RATE);

    MPC mpc;

    while(ros::ok())
    {

        curr_loc = vicon_position;
        curr_ang = atan2(2 * (quat.x * quat.y + quat.w * quat.z), pow(quat.w,2) + pow(quat.x,2) - pow(quat.y,2) - pow(quat.z,2));
        state << curr_loc.x, curr_loc.y, curr_ang;

        // Acknowledge that we reached the desired waypoint
        if (gotWP)
        {
            ROS_INFO("x: %f, y: %f", curr_loc.x, curr_loc.y);

            if ((goalDist(curr_loc, current_waypoint) < EPSILON_RADIUS) || (slow_flag == true))
            {
                // tell STARL if waypoint is reached
                // for now assume we only do that once we reach the final dest
                std_msgs::String wp_reached;
                wp_reached.data = "TRUE";
                reached_pub.publish(wp_reached);
                ROS_INFO("Goal dist: %f", goalDist(curr_loc, current_waypoint));
                // Stop moving
                gotWP = false;
                speed = 0;
                direction = 0;
                slow_flag = false;
                
                waypoints.clear();
            }
            else
            {
                while (waypoints.size() < 10)
                {
                    // Pad vector with last element if smaller than 10
                    // Last element should be equal to current_waypoint
                    waypoints.push_back(waypoints.back());
                }
                
                auto tic = std::chrono::high_resolution_clock::now();
                std::vector<double> solution = mpc.Solve(state, waypoints);
                auto toc = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic);
                std::cout << "MPC time: " << duration.count() / 1000000. << std::endl; //Time in seconds
                
                direction = solution.at(0);
                speed = solution.at(1);
                ROS_INFO("speed: %f, steering: %f", speed, direction);
                
                waypoints.erase(waypoints.begin()); //delete first element
            }
        }
        r.sleep();
    }
}


void drive_cmd()
{   
    ros::Rate r(100);
    
    double vel_error = 0;
    double prev_vel = 0;
    
    while (ros::ok())
    {
        quat_eig = Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
        vel_eig << vicon_vel.x, vicon_vel.y, vicon_vel.z;
        
        vel_tf = vel_eig.rotate(quat_eig);
        double car_vel = vel_tf(0);
        
        std::cout << "Original vel x: " << vel_eig(0) << ", y: " << vel_eig(1) << "; ";
        std::cout << "Rotate vel x: " << vel_tf(0) << ", y: " << vel_tf(1) << std::endl;
        
        if (fabs(car_vel) < 0.001) car_vel = 0; //Vicon noise...
        
        vel_error = speed - car_vel;
        vel_error_int += vel_error;
        vel_error_int = fmax(fmin(vel_error_int, 4.0), -4.0);
        vel_error_deriv = 0.9 * vel_error + 0.1 * vel_error_deriv;
        
        double vel_cmd = Kp * vel_error + Ki * vel_error_int + Kd * vel_error_deriv;
        vel_cmd = fmax(fmin(vel_cmd, 4.0), -4.0);
        
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = vel_cmd;
        drive_msg.drive.steering_angle = direction;
        drive_pub.publish(drive_msg);
        
        ROS_INFO("Ackermann speed: %f", vel_cmd);
        
        r.sleep()
    }
    
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = 0;
    drive_msg.drive.steering_angle = 0;
    drive_pub.publish(drive_msg);
}

void printToFile()
{
    std::ofstream positionFile;
    positionFile.open (dir_path+"/posData_"+time_buffer+".txt", std::ios::app);

    // Sleep and don't print anything while we are not flying
    while(ros::ok() && !isDriving)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Once we send the takeoff command, start printing
    ros::Rate printrate(PRINT_RATE);
    ros::Time time_start = ros::Time::now();
    while(ros::ok() && isDriving)
    {
        ros::Duration time_since_start = ros::Time::now() - time_start;
        positionFile << time_since_start.toNSec() / 1000 << ", "; //Print time in useconds
        positionFile << vicon_position.x << ", " << vicon_position.y << ", " << vicon_position.z << "\r\n ";

        printrate.sleep();
    }
    positionFile.close();
}

void getWP(const geometry_msgs::PoseStamped& stamped_point)
{
    geometry_msgs::Point point = stamped_point.pose.position;
    
    if (waypoints.size() == 0) waypoints.push_back(vicon_position);

    waypoints.push_back(point);
    std::cout << "Got Point x: " << point.x << ", y: " << point.y << std::endl;

    // wait until we get the final point
    if(stamped_point.header.frame_id == "1")
    {
        current_waypoint = point;
        gotWP = true;
        slow_time = ros::Time::now();
        slow_flag = false;
    }

    if(!isDriving)
    {
        isDriving = true;
    }
}

int main(int argc, char **argv)
{
    current_waypoint.x = current_waypoint.y = current_waypoint.z = 0;
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n("~");

    n.param<std::string>("vicon_obj", vicon_obj, "hotdec_car");

    std::cout << "Vicon Object: " << vicon_obj << std::endl;

    reached_pub = n.advertise<std_msgs::String>("reached", 1);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);

    ros::Subscriber sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getViconPosition);
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/twist", 1, getViconVel);
    ros::Subscriber waypoint = n.subscribe("waypoint", 50, getWP);  // second parameter is num of buffered messages

    dir_path = ros::package::getPath("rrt_car");

    // Gets the current time so we can add to data output
    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo;
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 80, "%G%m%dT%H%M%S", timeinfo);

    std::cout << "Starting waypoint follower" << std::endl;

    drive_thread = std::thread(drive);
    cmd_thread = std::thread(drive_cmd);
    //print_thread = std::thread(printToFile);

    ros::spin();

    drive_thread.join();
    cmd_thread.join();
    //print_thread.join();

    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = 0;
    drive_msg.drive.steering_angle = 0;
    drive_pub.publish(drive_msg);

    return 0;
}

