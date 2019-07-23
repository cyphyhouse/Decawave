#include <cstdio>
#include <iostream>
#include <cmath>
#include <thread>
#include <csignal>
#include <vector>

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

static const double lat0 = 40.116, lon0 = -88.224;	// IRL GPS coords
bool takeoff_flag = false;
bool land_flag = false;
bool gotWP_flag = false;
bool isFlying = false;

int takeoff_num = 0;

std::string bot_num, vicon_obj;
bool use_deca;

ros::ServiceClient arming_client, takeoff_client, land_client, mode_client, sethome_client;

ros::Publisher postarget_pub, reached_pub;

geometry_msgs::Point current_pos;
geometry_msgs::Twist current_vel;
geometry_msgs::Point current_waypoint, takeoff_pos;  // VICON coords

std::vector<geometry_msgs::Point> waypoints;

static mavconn::MAVConnInterface::Ptr ardupilot_link;

static GeographicLib::Geoid egm96_5("egm96-5", "", true, true);
static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
static GeographicLib::LocalCartesian proj(lat0, lon0, 0, earth);

std::thread gps_thread, print_thread, pos_thread;


void getPosition(const geometry_msgs::PoseStamped& posestamped)
{
    current_pos = posestamped.pose.position;
}

void getVelocity(const geometry_msgs::TwistStamped& twiststamped)
{
    current_vel = twiststamped.twist;
}

void printPos()
{
    ros::Rate pr(1);
    while(ros::ok())
    {
        ROS_INFO("x: %f, y: %f, z: %f\n", current_pos.x, current_pos.y, current_pos.z);
        pr.sleep();
    }
}

void sendFakeGPS()
{
    mavlink::common::msg::HIL_GPS fix {};
    ros::Rate r(GPS_RATE);

    while(ros::ok())
    {
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
}

void sendWP()
{
    ros::Rate r(WP_RATE);

    while(ros::ok())
    {
        if (!isFlying)    
        {
            if (gotWP_flag) // currently not flying, takeoff
            {
                mavros_msgs::SetMode mode_msg;
                mode_msg.request.base_mode = 0;
                mode_msg.request.custom_mode = "GUIDED";
                mode_client.call(mode_msg);
                
                double lat, lon, h;
                proj.Reverse(current_waypoint.y, -current_waypoint.x, current_waypoint.z, lat, lon, h);
                
                mavros_msgs::CommandBool arming_msg;
                mavros_msgs::CommandTOL takeoff_msg;
                arming_msg.request.value = true;
                if (arming_client.call(arming_msg))
                    ROS_INFO("arming cmd success");
                else
                    ROS_INFO("arming cmd failed");
                    
                takeoff_msg.request.min_pitch = 0; //have no idea about this
                takeoff_msg.request.yaw = 0;
                takeoff_msg.request.latitude = lat; // This is either the first waypoint or whatever the initial position is
                takeoff_msg.request.longitude = lon;
                takeoff_msg.request.altitude = h;

                if (takeoff_client.call(takeoff_msg))
                    ROS_INFO("takeoff cmd success");
                else
                    ROS_INFO("takeoff cmd failed");

                takeoff_flag = true;
                isFlying = true;
                

                mavros_msgs::CommandHome sethome_msg;
                sethome_msg.request.current_gps = false;
                sethome_msg.request.latitude = lat0;
                sethome_msg.request.longitude = lon0;
                sethome_msg.request.altitude = 0;
                sethome_client.call(sethome_msg);
	
                if (takeoff_num == 0) takeoff_pos = current_pos;

	            takeoff_num++;
	        }
        }
        else
        {
            // Have to resend first waypoint
            if (takeoff_flag && (current_pos.z >= 0.5))
            {
                geometry_msgs::PoseStamped postarget_msg;
                postarget_msg.header.stamp = ros::Time::now();
                postarget_msg.header.frame_id = '0';
                postarget_msg.pose.position.x = current_waypoint.y - takeoff_pos.y;
                postarget_msg.pose.position.y = -(current_waypoint.x - takeoff_pos.x);
                postarget_msg.pose.position.z = (current_waypoint.z - takeoff_pos.z);
                postarget_pub.publish(postarget_msg);
                takeoff_flag = false;
            }
            
            if (land_flag && (current_pos.z <= 0.3))
            {
                land_flag = false;
            }
            
            

            // Acknowledge that we reached the desired waypoint
            if (gotWP_flag)
            {
                
                if (sqrt(pow(current_pos.x - current_waypoint.x,2) + pow(current_pos.y - current_waypoint.y,2) + pow(current_pos.z - current_waypoint.z,2)) < 0.3)
                {
                    waypoints.erase(waypoints.begin()); //delete first element
                    
                    if(waypoints.size() == 0) //reached last point
                    {
                        // tell CyPyHous3 if waypoint is reached
                        // for now we only do that once we reach the final dest
                        std_msgs::String wp_reached;
                        wp_reached.data = "TRUE";
                        gotWP_flag = false;
                        reached_pub.publish(wp_reached);
                    }
                    else
                    {
                        // else, set the next point as the next destination
                        current_waypoint = waypoints.front();
                        
                        if (current_waypoint.z <= 0.0)
                        {
                            double lat, lon, h;
                            proj.Reverse(current_waypoint.y, -current_waypoint.x, current_waypoint.z, lat, lon, h);
                            
                            mavros_msgs::CommandBool arming_msg;
                            mavros_msgs::CommandTOL land_msg;
                            land_msg.request.min_pitch = 0;
                            land_msg.request.yaw = 0;
                            land_msg.request.latitude = lat;
                            land_msg.request.longitude = lon;
                            land_msg.request.altitude = 0;
                            if (land_client.call(land_msg))
                                ROS_INFO("landing cmd success");
                            else
                                ROS_INFO("landing cmd failed");
                                
                            arming_msg.request.value = false;
                            if (arming_client.call(arming_msg))
                                ROS_INFO("disarming cmd success");
                            else
                                ROS_INFO("disarming cmd failed");
                                
                            land_flag = true;
                        }
                        else
                        {
                            geometry_msgs::PoseStamped postarget_msg;
                            postarget_msg.header.stamp = ros::Time::now();
                            postarget_msg.header.frame_id = '0';
                            postarget_msg.pose.position.x = (current_waypoint.y - takeoff_pos.y);
                            postarget_msg.pose.position.y = -(current_waypoint.x - takeoff_pos.x);
                            postarget_msg.pose.position.z = (current_waypoint.z - takeoff_pos.z);

                            // publish it
                            postarget_pub.publish(postarget_msg);
                        }
                    }
                    
                }
            }
        }
        
        r.sleep();
    }
}

void printToFile()
{
    // TODO: look into getcwd() to get current directory
    std::ofstream positionFile;
    positionFile.open ("/home/pi/copterpos.txt", std::ios::app);
    
    // Sleep and don't print anything while we are not flying
    while(ros::ok() && !isFlying)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // Once we send the takeoff command, start printing
    ros::Rate printrate(PRINT_RATE);
    ros::Time time_start = ros::Time::now();
    while(ros::ok() && isFlying)
    {
        ros::Duration time_since_start = ros::Time::now() - time_start;
        positionFile << time_since_start.toNSec() / 1000 << ", "; //Print time in useconds
        positionFile << current_pos.x << ", " << current_pos.y << ", " << current_pos.z << "\r\n";
        
        printrate.sleep();
    }
    positionFile.close();
}

void getWP(const geometry_msgs::PoseStamped& stamped_point)
{
    geometry_msgs::Point point = stamped_point.pose.position;
    std::string stamp = stamped_point.header.frame_id;
    
    std::cout << "Got point x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
    
    if (waypoints.size() == 0)
    {
        current_waypoint.x = point.x;
        current_waypoint.y = point.y;
        current_waypoint.z = point.z;
    }
    
    waypoints.push_back(point);
    
    if (stamp == "1")
    {
        gotWP_flag = true;
        std::cout << "Doing path" << std::endl;
    }
    
    
}

int main(int argc, char **argv)
{
    current_waypoint.x = current_waypoint.y = current_waypoint.z = 0;
    ros::init(argc, argv, "fakeGPS");
    ardupilot_link = mavconn::MAVConnInterface::open_url("udp://127.0.0.1:14550@");
    ros::NodeHandle n("~");
    
    n.param<std::string>("vicon_obj", vicon_obj, "cyphyhousecopter");
    n.param<bool>("use_deca", use_deca, false);
    
    std::string waypoint_topic, reached_topic;
    if (!n.getParam("motion_automaton/waypoint_topic", waypoint_topic))
    {
        std::cout << "Error reading waypoint_topic" << std::endl;
    }
    if (!n.getParam("motion_automaton/reached_topic", reached_topic))
    {
	std::cout << "Error reading reached_topic" << std::endl;
    }

    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    postarget_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    reached_pub = n.advertise<std_msgs::String>(reached_topic, 1);
    
    ros::Subscriber pos_sub, vel_sub;
    if (!use_deca)
    {
        pos_sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getPosition);
        vel_sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/twist", 1, getVelocity);
    }
    else
    {
        pos_sub = n.subscribe("decaPos", 1, getPosition);
        vel_sub = n.subscribe("decaVel", 1, getVelocity);
    }
    
    ros::Subscriber waypoint = n.subscribe(waypoint_topic, 1, getWP);
    

    sethome_client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    mavros_msgs::CommandHome sethome_msg;
    sethome_msg.request.current_gps = false;
    sethome_msg.request.latitude = lat0;
    sethome_msg.request.longitude = lon0;
    sethome_msg.request.altitude = 0;
    sethome_client.call(sethome_msg);
    
    gps_thread = std::thread(sendFakeGPS);
    pos_thread = std::thread(printPos);
    //print_thread = std::thread(printToFile);

    ros::spin();
    
    gps_thread.join();
    pos_thread.join();
    //print_thread.join();
    return 0;
}

