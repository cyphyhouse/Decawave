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
#include <chrono>

#define WP_RATE 50.0 //Hz
#define PRINT_RATE 50.0 //Hz

#define DELTA_DIRECTION  0.01
#define DELTA_SPEED      0.25
#define EPSILON_RADIUS   0.25
#define EPSILON_ANGLE    0.1

bool starl_flag = false;
bool isDriving = false;
bool gotWP = false;
double speed = 0, direction = 0;
geometry_msgs::Point prev_loc, curr_loc;
double curr_ang = 0;

std::string bot_num, vicon_obj;
std::vector<geometry_msgs::Point> waypoints;
std::vector<double> waypoints_x;
std::vector<double> waypoints_y;

ros::Publisher drive_pub;
ros::Publisher reached_pub;

geometry_msgs::Point deca_position, vicon_position;
geometry_msgs::Point current_waypoint;  // VICON coords
geometry_msgs::Quaternion quat; //Get orientation

std::string dir_path;
char time_buffer[80];
std::thread drive_thread, print_thread;

Eigen::VectorXd state(5);

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}


void getDecaPosition(const geometry_msgs::Point& point)
{
    deca_position = point;
}

void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_position = pose.pose.position;
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
        curr_ang = atan2(2 * (quat.x * quat.y + quat.w * quat.z), pow(quat.w,2) + pow(quat.x,2) - pow(quat.y,2) - pow(quat.z,2));

        // Acknowledge that we reached the desired waypoint
        if (starl_flag)
        {
            if (sqrt(pow(curr_loc.x - current_waypoint.x,2) + pow(curr_loc.y - current_waypoint.y,2)) < EPSILON_RADIUS)            {
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
            int n_wp = 10; //Examine first 10 waypoints in list
            for (int i =0; i < n_wp; ++i){
                waypoints_x.push_back(waypoints[i].x);
                waypoints_y.push_back(waypoints[i].y);
            }
            double* wpx = &waypoints_x[0];
            double* wpy = &waypoints_y[0];
            Eigen::Map<Eigen::VectorXd> wpx_solve(wpx, n_wp);
            Eigen::Map<Eigen::VectorXd> wpy_solve(wpy, n_wp);
            auto coeffs(wpx_solve, wpy_solve, 3);
            double cte = polyeval(coeffs,0);
            double epsi = -atan(coeffs[1]);
            state << curr_loc.x, curr_loc.y, curr_ang, cte, epsi;
            auto tic = std::chrono::high_resolution_clock::now();
            vector<double> solution = mpc.Solve(state,coeffs);
            auto toc = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic);
            std::cout << "MPC time: " << duration.count() / 1000000. << std::endl; //Time in seconds
            
            direction = solution.at(0);
            speed = solution.at(1);
            ROS_INFO("speed: %f, steering: %f", speed, direction);
        }

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = direction;
        drive_pub.publish(drive_msg);
        //ROS_INFO("speed: %f, steering: %f",  speed, direction);
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

void getWP(const geometry_msgs::PoseStamped& stamped_point)
{
    geometry_msgs::Point point = stamped_point.pose.position;

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

    reached_pub = n.advertise<std_msgs::String>("reached", 1);
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);

    ros::Subscriber deca_pos = n.subscribe("decaPos", 1, getDecaPosition);
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getViconPosition);
    ros::Subscriber waypoint = n.subscribe("waypoint", 10, getWP);  // second parameter is num of buffered messages

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

