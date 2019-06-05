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
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "ros/package.h"

#define WP_RATE 100.0 //Hz
#define PRINT_RATE 100.0 //Hz

#define DELTA_DIRECTION  0.01
#define DELTA_SPEED      0.25
#define EPSILON_RADIUS   0.25
#define EPSILON_ANGLE    0.1

bool starl_flag = false;
bool isDriving = false;
bool gotWP = false;
double speed = 0, direction = 0;
geometry_msgs::Point prev_loc, curr_loc;

std::string bot_num, vicon_obj;
std::vector<geometry_msgs::Point> waypoints;

ros::Publisher drive_pub;
ros::Publisher reached_pub;

geometry_msgs::Point deca_position, vicon_position;
geometry_msgs::Point current_waypoint;  // VICON coords
geometry_msgs::Quaternion quat; //Get orientation

std::string dir_path;
char time_buffer[80];
std::thread drive_thread, print_thread;

Eigen::VectorXd state(3);

void getDecaPosition(const geometry_msgs::Point& point)
{
    deca_position = point;
}

void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_position = pose.pose.position;
}

void getViconOrientation(const geometry_msgs::PoseStamped& pose)
{
    quat = pose.pose.orientation;
}

void drive()
{
    ros::Rate r(WP_RATE);

    MPC mpc;

    while(ros::ok())
    {

        prev_loc = curr_loc;
        curr_loc = vicon_position;
        curr_ang = atan2(2 * (quat.x * quat.y + quat.w * quat.z), quat.w**2 + quat.x**2 - quat.y**2 - quat.z**2);
                state << curr_loc.x, curr_loc.y, curr_ang

        // Acknowledge that we reached the desired waypoint
        if (starl_flag)
        {
            if (sqrt(pow(curr_loc.x - current_waypoint.x,2) + pow(curr_loc.y - current_waypoint.y,2)) < 0.25)
            {
                waypoints.erase(waypoints.begin()); //delete first element

                if(waypoints.size() == 0) //reached last point
                {
                    // tell STARL if waypoint is reached
                    // for now assume we only do that once we reach the final dest
                    std_msgs::String wp_reached;
                    wp_reached.data = "TRUE";
                    starl_flag = false;
                    reached_pub.publish(wp_reached);

                    gotWP = false;
                    speed = 0;
                    direction = 0;
                }
                else
                {
                    current_waypoint = waypoints.front();

                }
            }
        }

        //ROS_INFO("x: %f, y: %f, z: %f\n", curr_loc.x, curr_loc.y, curr_loc.z);

        if (gotWP)
        {
            auto solution = mpc.Solve(state);
            direction = solution[0];
            speed = solution[1];
        }

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = direction;
        drive_pub.publish(drive_msg);
        //ROS_INFO("speed: %f, steering: %f, a_error: %f", speed, direction, a_error);
        r.sleep();
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
        positionFile << vicon_position.x << ", " << vicon_position.y << ", " << vicon_position.z << ", ";
        positionFile << deca_position.x << ", " << deca_position.y << ", " << deca_position.z << "\r\n";

        printrate.sleep();
    }
    positionFile.close();
}

void getWP(const geometry_msgs::PointStamped& stamped_point)
{
    geometry_msgs::Point point = stamped_point.point;

    if (waypoints.size() == 0)
    {
        current_waypoint.x = point.x;
        current_waypoint.y = point.y;
        //current_waypoint.z = point.z;
    }

    waypoints.push_back(point);

    // wait until we get the final point
    if(stamped_point.header.frame_id == "1")
    {
        gotWP = true;
        starl_flag = true;
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
    n.param<std::string>("bot_num", bot_num, "bot1");

    std::cout << "Vicon Object: " << vicon_obj << ", bot_num: " << bot_num << std::endl;

    reached_pub = n.advertise<std_msgs::String>("/Reached", 1);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);

    ros::Subscriber deca_pos = n.subscribe("/decaPos", 1, getDecaPosition);
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getViconPosition);
    ros::Subscriber waypoint = n.subscribe("/Waypoint_"+bot_num, 10, getWP);  // second parameter is num of buffered messages

    dir_path = ros::package::getPath("cyphy_car");

    // Gets the current time so we can add to data output
    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo;
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 80, "%G%m%dT%H%M%S", timeinfo);

    prev_loc.x = 0;
    prev_loc.y = 0;
    curr_loc.x = 0;
    curr_loc.y = 0;
    curr_ang = 0;

    std::cout << "Starting waypoint follower" << std::endl;

    drive_thread = std::thread(drive);
    //print_thread = std::thread(printToFile);

    ros::spin();

    drive_thread.join();
    //print_thread.join();

    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = 0;
    drive_msg.drive.steering_angle = 0;
    drive_pub.publish(drive_msg);

    return 0;
}

