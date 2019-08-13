#include <cstdio>
#include <iostream>
#include <cmath>
#include <thread>
#include <csignal>
#include <vector>
#include <mutex>
#include <atomic>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <mavconn/interface.h>

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/String.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/package.h"

#define GPS_RATE 10 //Hz
#define WP_RATE 10 //Hz
#define PRINT_RATE 100 //Hz

#define WP_RADIUS 0.3 //m
#define TAKEOFF_H 0.5 //m
#define LAND_H 0.3 //m
#define TAKEOFF_TIMEOUT 10.0 //s
#define LAND_TIMEOUT 10.0 //s
#define VICON_TIMEOUT 3.0 //s

const double lat0 = 40.116, lon0 = -88.224;  // IRL GPS coords
std::atomic<bool> gotWP_flag (false);

enum Stage { ground, takeoff, flight, land, landing };
std::atomic<Stage> quad_state (ground);

ros::ServiceClient arming_client, takeoff_client, land_client, mode_client, sethome_client;
ros::Publisher postarget_pub, reached_pub;

geometry_msgs::Point current_pos, current_waypoint, takeoff_pos;
geometry_msgs::Twist current_vel;

std::vector<geometry_msgs::Point> waypoints;

mavconn::MAVConnInterface::Ptr ardupilot_link;

GeographicLib::Geoid egm96_5("egm96-5", "", true, true);
GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
GeographicLib::LocalCartesian proj(lat0, lon0, 0, earth);

std::thread gps_thread, print_thread, pos_thread, wp_thread;
std::mutex wp_mutex;

ros::Time pos_time;

void emergencyLand()
{
    std::cout << "Emergency Land" << std::endl;
    quad_state = land;
    wp_mutex.lock();
    current_waypoint = current_pos;
    wp_mutex.unlock();
    gotWP_flag = true;
}

void getPosition(const geometry_msgs::PoseStamped& posestamped)
{
    current_pos = posestamped.pose.position;
    pos_time = ros::Time::now();
}

void getVelocity(const geometry_msgs::TwistStamped& twiststamped)
{
    current_vel = twiststamped.twist;
}

bool takeoff_seq(const geometry_msgs::Point point)
{
    static bool takeoff_flag = false;
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.base_mode = 0;
    mode_msg.request.custom_mode = "GUIDED";
    mode_client.call(mode_msg);

    double lat, lon, h;
    proj.Reverse(point.y, -point.x, point.z, lat, lon, h);

    mavros_msgs::CommandBool arming_msg;
    mavros_msgs::CommandTOL takeoff_msg;
    arming_msg.request.value = true;
    if (arming_client.call(arming_msg)) {
        ROS_INFO("arming cmd success");
    }
    else {
        ROS_INFO("arming cmd failed");
        return false;
    }
        
    takeoff_msg.request.min_pitch = 0; //have no idea about this
    takeoff_msg.request.yaw = 0;
    takeoff_msg.request.latitude = lat; // This is either the first waypoint or whatever the initial position is
    takeoff_msg.request.longitude = lon;
    takeoff_msg.request.altitude = h;

    if (takeoff_client.call(takeoff_msg)) {
        ROS_INFO("takeoff cmd success");
    }
    else {
        ROS_INFO("takeoff cmd failed");
        return false;
    }

    mavros_msgs::CommandHome sethome_msg;
    sethome_msg.request.current_gps = false;
    sethome_msg.request.latitude = lat0;
    sethome_msg.request.longitude = lon0;
    sethome_msg.request.altitude = 0;
    sethome_client.call(sethome_msg);

    if (takeoff_flag == false)
    {
        takeoff_pos = current_pos;
        takeoff_flag = true;
    }

    return true;
}

bool land_seq(const geometry_msgs::Point point)
{
    double lat, lon, h;
    proj.Reverse(point.y, -point.x, point.z, lat, lon, h);
    
    mavros_msgs::CommandBool arming_msg;
    mavros_msgs::CommandTOL land_msg;
    land_msg.request.min_pitch = 0;
    land_msg.request.yaw = 0;
    land_msg.request.latitude = lat;
    land_msg.request.longitude = lon;
    land_msg.request.altitude = 0;
    if (land_client.call(land_msg)) {
        ROS_INFO("landing cmd success");
    }
    else {
        ROS_INFO("landing cmd failed");
        return false;
    }
     
    arming_msg.request.value = false;
    if (arming_client.call(arming_msg)) {
        ROS_INFO("disarming cmd success");
    }
    else {
        ROS_INFO("disarming cmd failed");
        return false;
    }
    
    return true;
}

void sendPosition(const geometry_msgs::Point point)
{
    geometry_msgs::PoseStamped postarget_msg;
    postarget_msg.header.stamp = ros::Time::now();
    postarget_msg.header.frame_id = '0';
    postarget_msg.pose.position.x = point.y - takeoff_pos.y;
    postarget_msg.pose.position.y = -(point.x - takeoff_pos.x);
    postarget_msg.pose.position.z = (point.z - takeoff_pos.z);
    postarget_pub.publish(postarget_msg);
    
    std::cout << "Publishing point x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;                    
}

inline double goalDist(const geometry_msgs::Point point)
{
    return sqrt(pow(current_pos.x - point.x,2) + pow(current_pos.y - point.y,2) + pow(current_pos.z - point.z,2));
}

void printPos()
{
    ros::Rate pr(1);
    while(ros::ok())
    {
        ROS_INFO("x: %f, y: %f, z: %f\n", current_pos.x, current_pos.y, current_pos.z);
        pr.sleep();
    }
    std::cout << "Done print loop" << std::endl;
}

void sendFakeGPS()
{
    mavlink::common::msg::HIL_GPS fix {};
    ros::Rate r(GPS_RATE);

    while(ros::ok())
    {
    
        if (((ros::Time::now() - pos_time).toSec() > VICON_TIMEOUT) && (quad_state == flight))
        {
            emergencyLand();
        }
       
       	geometry_msgs::Point point = current_pos;
         
        double lat, lon, h;
        //ROS_INFO("x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
        proj.Reverse(point.y, -point.x, point.z, lat, lon, h);
        //ROS_INFO("latitude: %f, longitude: %f, altitude: %f", lat, lon, h);

        // compute course over ground (borrowed from mavros)
        double cog;
        if (current_vel.linear.x == 0 && current_vel.linear.y == 0) {
            cog = 0;
        }
        else if (current_vel.linear.x >= 0 && current_vel.linear.y < 0) {
            cog = M_PI * 5 / 2 - atan2(current_vel.linear.x, current_vel.linear.y);
        }
        else {
            cog = M_PI / 2 - atan2(current_vel.linear.x, current_vel.linear.y);
        }

        // populate GPS message
        fix.time_usec = ros::Time::now().toNSec() / 1000;
        fix.lat = lat * 1e7;
        fix.lon = lon * 1e7;
        fix.alt = h * 1e3;
        fix.vel = sqrt(pow(current_vel.linear.x, 2) + pow(current_vel.linear.y, 2) + pow(current_vel.linear.z, 2)) * 100;
        fix.vn = -current_vel.linear.x * 100;
        fix.ve = current_vel.linear.y * 100;
        fix.vd = -current_vel.linear.z * 100;
        fix.cog = cog * 1e2;
        fix.eph = 1;
        fix.epv = 1;
        fix.fix_type = 3;
        fix.satellites_visible = 6;

        // send it
        ardupilot_link.get()->send_message_ignore_drop(fix);
        
        r.sleep();
    }
    std::cout << "Done GPS loop" << std::endl;
}

void sendWP()
{
    ros::Time stage_time;
    ros::Rate r(WP_RATE);

    while(ros::ok())
    {
        if (gotWP_flag)
        {
            switch(quad_state)
            {
                case ground: // currently not flying, takeoff
                {
                    wp_mutex.lock();
                    current_waypoint = waypoints.front(); // Read first point, but only remove it once we tookoff
                    wp_mutex.unlock();
                    
                    if (takeoff_seq(current_waypoint))
                    {
                        quad_state = takeoff;
                        std::cout << "Takeoff stage" << std::endl;
                        stage_time = ros::Time::now();
                    }
                    
                    break;
                }
                case takeoff:
                {
                    if (current_pos.z >= TAKEOFF_H)
                    {
                        // Successfully tookoff, resend first point
                        wp_mutex.lock();
                        waypoints.erase(waypoints.begin()); //delete first element
                        wp_mutex.unlock();
                        sendPosition(current_waypoint);
                      
                        std::cout << "Flight stage" << std::endl;
                        quad_state = flight;
                        
                    }
                    else if ( (ros::Time::now() - stage_time).toSec() >= TAKEOFF_TIMEOUT )
                    {
                        // took too long to takeoff, try resending takeoff cmd
                        std::cout << "Didn't takeoff, retrying" << std::endl;
                        quad_state = ground;
                        std::cout << "Flight Ground" << std::endl;
                    }
                    
                    break;
                }
                case flight:
                {
                    if (goalDist(current_waypoint) < WP_RADIUS)
                    {
                        double wp_length;
                        wp_mutex.lock();
                        wp_length = waypoints.size();
                        wp_mutex.unlock();
                        if(wp_length == 0) //reached last point
                        {
                            // tell CyPyHous3 if waypoint is reached
                            // for now we only do that once we reach the final dest
                            std_msgs::String wp_reached;
                            wp_reached.data = "TRUE";
                            reached_pub.publish(wp_reached);
                            
                            gotWP_flag = false;
                            quad_state = flight;
                            std::cout << "******************** End Path **********************" << std::endl;
                        }
                        else
                        {
                            std::cout << "Reached Midpoint" << std::endl;
                            wp_mutex.lock();
                            current_waypoint = waypoints.front();
                            waypoints.erase(waypoints.begin()); //delete first element
                            wp_mutex.unlock();
                            
                            if (current_waypoint.z <= 0.0)
                            {
                                quad_state = land;
                                std::cout << "Land stage" << std::endl;
                            }
                            else
                            {
                                sendPosition(current_waypoint);
                                
                                quad_state = flight;
                            }
                        }
                    }
                    
                    break;
                }
                case land:
                {
                    if (land_seq(current_waypoint)) {
                        quad_state = landing;
                        std::cout << "Landing stage" << std::endl;
                        stage_time = ros::Time::now();
                    }
                    
                    break;
                }
                case landing:
                {
                    if (current_pos.z <= LAND_H)
                    {
                        std_msgs::String wp_reached;
                        wp_reached.data = "TRUE";
                        reached_pub.publish(wp_reached);
                        
                        quad_state = ground;
                        std::cout << "Ground stage" << std::endl;
                        gotWP_flag = false;
                        wp_mutex.lock();
                        waypoints.clear();
                        wp_mutex.unlock();
                    }
                    else if ( (ros::Time::now() - stage_time).toSec() >= LAND_TIMEOUT )
                    {
                         // took too long to land, try resending land cmd
                         std::cout << "Didn't land, retrying" << std::endl;
                         quad_state = land;
                         std::cout << "Land stage" << std::endl;
                    }
                    
                    break;
                }
            }
        }
        
        r.sleep();
    }
    std::cout << "Done WP loop" << std::endl;
}

void printToFile()
{
    // TODO: look into getcwd() to get current directory
    std::ofstream positionFile;
    positionFile.open ("/home/pi/copterpos.txt", std::ios::app);
    
    // Sleep and don't print anything while we are not flying
    while(ros::ok() && (quad_state == ground))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Once we send the takeoff command, start printing
    ros::Rate printrate(PRINT_RATE);
    ros::Time time_start = ros::Time::now();
    while(ros::ok() && (quad_state != ground))
    {
        ros::Duration time_since_start = ros::Time::now() - time_start;
        positionFile << time_since_start.toNSec() / 1000 << ", "; //Print time in useconds
        positionFile << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << "\r\n";
        
        printrate.sleep();
    }
    positionFile.close();
    std::cout << "Done save loop" << std::endl;
}

void getWP(const geometry_msgs::PoseStamped& stamped_point)
{
    geometry_msgs::Point point = stamped_point.pose.position;
    std::string stamp = stamped_point.header.frame_id;
    
    std::cout << "Got point x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
    
    if (stamp == "2")
    {
        wp_mutex.lock();
        waypoints.clear();
        if (quad_state == flight)
        {
            current_waypoint = current_pos;
            waypoints.push_back(current_pos);
        }
        wp_mutex.unlock();
    }
    else if ((point.z <= 0.0) && (quad_state == ground))
    {
        //ignore, already landed
        return;
    }
    else
    {
        wp_mutex.lock();
        waypoints.push_back(point);
        wp_mutex.unlock();

        if (stamp == "1")
        {
            gotWP_flag = true;
            std::cout << "Doing path" << std::endl;
        }
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fakeGPS");
    ardupilot_link = mavconn::MAVConnInterface::open_url("udp://127.0.0.1:14550@");
    ros::NodeHandle n("~");
   
    std::string wp_topic, reached_topic, pos_topic, vicon_obj;
    bool use_deca;
    int queue_size;
    
    n.param<bool>("use_deca", use_deca, false);
    
    n.param<std::string>("device/bot_name", vicon_obj, "cph");
    n.param<std::string>("device/waypoint_topic/topic", wp_topic, "waypoint");
    n.param<std::string>("device/reached_topic/topic", reached_topic, "reached");
    n.param<std::string>("device/positioning_topic/topic", pos_topic, "/vrpn_client_node/");
    n.param<int>("device/queue_size", queue_size, 10);


    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    postarget_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    reached_pub = n.advertise<std_msgs::String>(reached_topic, 1);
    
    pos_time = ros::Time::now();
    ros::Subscriber pos_sub, vel_sub;
    if (!use_deca)
    {
        pos_sub = n.subscribe(pos_topic+vicon_obj+"/pose", 1, getPosition);
        vel_sub = n.subscribe(pos_topic+vicon_obj+"/twist", 1, getVelocity);
    }
    else
    {
        pos_sub = n.subscribe("decaPos", 1, getPosition);
        vel_sub = n.subscribe("decaVel", 1, getVelocity);
    }
    
    ros::Subscriber waypoint = n.subscribe(wp_topic, queue_size, getWP);
    
    sethome_client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    mavros_msgs::CommandHome sethome_msg;
    sethome_msg.request.current_gps = false;
    sethome_msg.request.latitude = lat0;
    sethome_msg.request.longitude = lon0;
    sethome_msg.request.altitude = 0;
    sethome_client.call(sethome_msg);
    
    gps_thread = std::thread(sendFakeGPS);
    pos_thread = std::thread(printPos);
    wp_thread = std::thread(sendWP);
    //print_thread = std::thread(printToFile);

    ros::spin();
    
    gps_thread.join();
    pos_thread.join();
    wp_thread.join();
    //print_thread.join();
    
    std::cout << "Joined all threads" << std::endl;
    return 0;
}

