#include <cstdio>
#include <iostream>
#include <cmath>
#include <thread>
#include <ctime>
#include <cstdbool>
#include <fstream>
#include <deque>


#include "ros/ros.h"
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "ros/package.h"

#define WP_RATE          100.0 //Hz
#define PRINT_RATE       100.0 //Hz

#define DELTA_DIRECTION  0.01
#define DELTA_SPEED      0.25
#define EPSILON_RADIUS   0.25
#define EPSILON_ANGLE    0.1

#define MAX_SPEED        4.0
#define MAX_ANGLE        0.35

bool isDriving = false;
bool gotWP = false;
double speed = 0, direction = 0;
geometry_msgs::Point prev_loc, curr_loc;

std::string bot_num, vicon_obj;
std::deque<geometry_msgs::Point> waypoints;

ros::Publisher drive_pub;
ros::Publisher reached_pub;

geometry_msgs::Point vicon_position;
geometry_msgs::Point current_waypoint;  // VICON coords
geometry_msgs::Quaternion quat; //Get orientation

std::string dir_path;
char time_buffer[80];
std::thread drive_thread, print_thread;

int vel_sign, dir_sign;

void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_position = pose.pose.position;
    quat = pose.pose.orientation;
}

inline double goalDist(const geometry_msgs::Point pos, const geometry_msgs::Point goal)
{
    return sqrt(pow(pos.x - goal.x, 2) + pow(pos.y - goal.y, 2));
}

double get_angle_error(geometry_msgs::Point center, geometry_msgs::Point waypoint, geometry_msgs::Quaternion q)
{
    double curr_ang = atan2(2 * (q.x * q.y + q.w * q.z), pow(q.w,2) + pow(q.x,2) - pow(q.y,2) - pow(q.z,2));
    double v_x = waypoint.x - center.x;
    double v_y = waypoint.y - center.y;
    //double abs_v = sqrt(v_x * v_x + v_y * v_y);
    //double wp_ang = acos((v_x)/(abs_v)); // angle between next wp and x axis.
    double wp_ang = atan2(v_y, v_x);
    std::cout << curr_ang << ", " << wp_ang-curr_ang << std::endl;
    return wp_ang - curr_ang;
}

double a_error = 0;
double a_integral = 0;
double a_prev = 0;
double d_target = 0;
double d_prev = 0;
double d_integral = 0;
double a_diff = 0;
double d_diff = 0;

double get_pid_distance(double d_err_curr, double d_err_prev)
{
    static double Kp = 1.2;
    static double Kd = 0.01;
    static double Ki = 0.01;
    static double i_limit = 2.0;
    d_integral += d_err_curr*(1.0/WP_RATE);
    d_integral = fmin(d_integral, i_limit);
    
    double diff = (d_err_curr - d_err_prev)*WP_RATE;
    d_diff = 0.2*d_diff + 0.8*diff;
    
    return Kp*d_err_curr + Kd*d_diff + Ki*d_integral;
}

double get_pid_angle(double a_err_curr, double a_err_prev)
{
    static double Kp = 1;
    static double Kd = 0.1;
    static double Ki = 0.01;
    static double a_limit = 0.2;
    a_integral += a_err_curr*(1.0/WP_RATE);
    a_integral = fmin(fmax(a_integral,-a_limit),a_limit);
    
    double diff = (a_err_curr - a_err_prev)*WP_RATE;
    a_diff = 0.3*a_diff + 0.7*diff;
    
    return Kp*a_err_curr + Kd*a_diff + Ki*a_integral;
}



void drive()
{
    ros::Rate r(WP_RATE);

    while(ros::ok())
    {
        
        curr_loc = vicon_position;
        
        // Acknowledge that we reached the desired waypoint
        if (gotWP)
        {
            if (goalDist(curr_loc, current_waypoint) < EPSILON_RADIUS)
            {
                waypoints.pop_front(); //delete first element
                
                if(waypoints.size() == 0) //reached last point
                {
                    // tell STARL if waypoint is reached
                    // for now assume we only do that once we reach the final dest
                    std_msgs::String wp_reached;
                    wp_reached.data = "TRUE";
                    gotWP = false;
                    reached_pub.publish(wp_reached);
                    
                    gotWP = false;
                    speed = 0;
                    direction = 0;
                    a_error = 0;
                    a_integral = 0;
                    a_prev = 0;
                    d_target = 0;
                    d_prev = 0;
                    d_integral = 0;
                }
                else
                {
                    current_waypoint = waypoints.front();
                    //might also need to reset the angle errors
                    a_integral = 0;
                    a_prev = 0;
                    d_integral = 0;
                    d_prev = 0;
                }
            }
        }

        //ROS_INFO("x: %f, y: %f, z: %f\n", curr_loc.x, curr_loc.y, curr_loc.z);

        if (gotWP)
        {
            a_error = get_angle_error(curr_loc, current_waypoint, quat);
            //if (a_error > M_PI) a_error -= 2*M_PI;
            //if (a_error < -M_PI) a_error += 2*M_PI;
            if (fabs(a_error) <= M_PI/2.0)
            {
                vel_sign = 1;
                dir_sign = 1;
            }
            else
            {
                vel_sign = -1;
                if (a_error <= 0.0)
                {
                    dir_sign = -1;
                    a_error += M_PI;
                }
                else
                {
                    dir_sign = 1;
                    a_error -= M_PI;
                }
            }
            
            d_target = sqrt(pow(curr_loc.x - current_waypoint.x,2) + pow(curr_loc.y - current_waypoint.y,2));
            
            double vel_pid = get_pid_distance(d_target,d_prev);
            double ang_pid = get_pid_angle(a_error, a_prev);
            
            //Update error terms for PID
            d_prev = d_target;
            a_prev = a_error;
            
            speed = vel_sign * vel_pid;
            direction = vel_sign * ang_pid;
            //ROS_INFO("d_target: %f, speed: %f, a_error: %f, direction: %f", d_target, vel_pid, a_error, ang_pid);
            
            speed = fmax(fmin(speed, MAX_SPEED), -MAX_SPEED);
            direction = fmax(fmin(direction, MAX_ANGLE), -MAX_ANGLE);
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
        
        printrate.sleep();
    }
    positionFile.close();
}

void getWP(const geometry_msgs::PoseStamped& stamped_point)
{
    geometry_msgs::Point point = stamped_point.pose.position;
    
    waypoints.push_back(point);
    
    // wait until we get the final point
    if(stamped_point.header.frame_id == "1")
    {
        current_waypoint = waypoints.front();
        gotWP = true;
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
    ros::Subscriber waypoint = n.subscribe("waypoint", 50, getWP);  // second parameter is num of buffered messages

    dir_path = ros::package::getPath("cyphy_car");
    
    // Gets the current time so we can add to data output
    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo;
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 80, "%G%m%dT%H%M%S", timeinfo);
    
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
