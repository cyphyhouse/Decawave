#include <cstdio>
#include <iostream>
#include <cmath>
#include <thread>
#include <csignal>

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
#define PRINT_RATE 100 //Hz

static const double lat0 = 40.116, lon0 = -88.224;	// IRL GPS coords
bool takeoff_flag = false;
bool starl_flag = false;
bool isFlying = false;

std::string bot_num, vicon_obj;
bool use_deca;

ros::ServiceClient arming_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient land_client;
ros::ServiceClient mode_client;
ros::ServiceClient sethome_client;

ros::Publisher postarget_pub;
ros::Publisher reached_pub;

geometry_msgs::Point deca_position, vicon_position, deca_vel, vicon_vel;
geometry_msgs::Point current_waypoint, takeoff_pos;  // VICON coords

static mavconn::MAVConnInterface::Ptr ardupilot_link;

static GeographicLib::Geoid egm96_5("egm96-5", "", true, true);
static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
static GeographicLib::LocalCartesian proj(lat0, lon0, 0, earth);

std::thread gps_thread, print_thread, pos_thread;

int stopflag = 1;
void sigHandler(int signum)
{
    if(signum == SIGINT)
    {
        printf("Stopping fakegps\n");
	stopflag = 0;
    }
}

void getDecaPosition(const geometry_msgs::Point& point)
{
    deca_position = point;
}

void getDecaVelocity(const geometry_msgs::Point& velocity)
{
    deca_vel.x = velocity.x;
    deca_vel.y = velocity.y;
    deca_vel.z = velocity.z;
}

void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_position = pose.pose.position;
}

void getViconVelocity(const geometry_msgs::TwistStamped& twist)
{
    vicon_vel.x = twist.twist.linear.x;
    vicon_vel.y = twist.twist.linear.y;
    vicon_vel.z = twist.twist.linear.z;
}

void printPos()
{
    ros::Rate pr(1);
    while(ros::ok() && stopflag)
    {
        ROS_INFO("x: %f, y: %f, z: %f\n", vicon_position.x, vicon_position.y, vicon_position.z);
        pr.sleep();
    }
}

void sendFakeGPS()
{
    mavlink::common::msg::HIL_GPS fix {};
    ros::Rate r(GPS_RATE);

    while(ros::ok() && stopflag)
    {
        geometry_msgs::Point point, current_vel;
        if(use_deca)
        {
            point = deca_position;
            current_vel = deca_vel;
        }
        else
        {
            point = vicon_position;
            current_vel = vicon_vel;
        }
         
        // Have to resend first waypoint
        if (sqrt(pow(point.z - current_waypoint.z,2)) < 0.3 && takeoff_flag)
        {
            geometry_msgs::PoseStamped postarget_msg;
            postarget_msg.header.stamp = ros::Time::now();
            postarget_msg.header.frame_id = '0';
            postarget_msg.pose.position.x = current_waypoint.y;
            postarget_msg.pose.position.y = -current_waypoint.x;
            postarget_msg.pose.position.z = current_waypoint.z;
            postarget_pub.publish(postarget_msg);
            takeoff_flag = false;
        }

        // Acknowledge that we reached the desired waypoint
        if (starl_flag)
        {
            if (sqrt(pow(point.x - current_waypoint.x,2) + pow(point.y - current_waypoint.y,2) + pow(point.z - current_waypoint.z,2)) < 0.3)
            {
                // tell STARL if waypoint is reached
                std_msgs::String wp_reached;
                wp_reached.data = "TRUE";
                starl_flag = false;
                reached_pub.publish(wp_reached);
            }
        }

        double lat, lon, h;
        //ROS_INFO("x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
        proj.Reverse(point.y, -point.x, point.z, lat, lon, h);
        //ROS_INFO("latitude: %f, longitude: %f, altitude: %f", lat, lon, h);


        // compute course over ground (borrowed from mavros)
        double cog;
        if (current_vel.x == 0 && current_vel.y == 0) {
            cog = 0;
        }
        else if (current_vel.x >= 0 && current_vel.y < 0) {
            cog = M_PI * 5 / 2 - atan2(current_vel.x, current_vel.y);
        }
        else {
            cog = M_PI / 2 - atan2(current_vel.x, current_vel.y);
        }

        // populate GPS message
        fix.time_usec = ros::Time::now().toNSec() / 1000;
        fix.lat = lat * 1e7;
        fix.lon = lon * 1e7;
        fix.alt = h * 1e3;
        fix.vel = sqrt(current_vel.x * current_vel.x + current_vel.y * current_vel.y + current_vel.z * current_vel.z) * 100;
        fix.vn = -current_vel.x * 100;
        fix.ve = current_vel.y * 100;
        fix.vd = -current_vel.z * 100;
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
        positionFile << vicon_position.x << ", " << vicon_position.y << ", " << vicon_position.z << ", ";
        positionFile << deca_position.x << ", " << deca_position.y << ", " << deca_position.z << "\r\n";
        
        printrate.sleep();
    }
    positionFile.close();
}

void sendWP(const geometry_msgs::PoseStamped& stamped_point)
{
    double lat, lon, h;
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.base_mode = 0;
    mode_msg.request.custom_mode = "GUIDED";
    mode_client.call(mode_msg);
    geometry_msgs::Point point = stamped_point.pose.position;
    std::string stamp = stamped_point.header.frame_id;
    proj.Reverse(point.y, -point.x, point.z, lat, lon, h);
    starl_flag = true;
    std::cout << "Going to point x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
    if (stamp == "0")    // takeoff
    {
        mavros_msgs::CommandBool arming_msg;
        mavros_msgs::CommandTOL takeoff_msg;
        arming_msg.request.value = true;
        if (arming_client.call(arming_msg))
            ROS_INFO("arming success");
        else
            ROS_INFO("arming failed");
        takeoff_msg.request.min_pitch = 0; //have no idea about this
        takeoff_msg.request.yaw = 0;
        takeoff_msg.request.latitude = lat; // This is either the first waypoint or whatever the initial position is
        takeoff_msg.request.longitude = lon;
        takeoff_msg.request.altitude = h;

        if (takeoff_client.call(takeoff_msg))
            ROS_INFO("takeoff success");
        else
            ROS_INFO("takeoff failed");
        takeoff_flag = true;
        isFlying = true;

        mavros_msgs::CommandHome sethome_msg;
        sethome_msg.request.current_gps = false;
        sethome_msg.request.latitude = lat0;
        sethome_msg.request.longitude = lon0;
        sethome_msg.request.altitude = 0;
        sethome_client.call(sethome_msg);
	
	takeoff_pos = vicon_position;
    }
    else if (stamp == "2")   // land
    {
        mavros_msgs::CommandBool arming_msg;
        mavros_msgs::CommandTOL land_msg;
        land_msg.request.min_pitch = 0;
        land_msg.request.yaw = 0;
        land_msg.request.latitude = lat;
        land_msg.request.longitude = lon;
        land_msg.request.altitude = 0;
        if (land_client.call(land_msg))
            ROS_INFO("landing success");
        else
            ROS_INFO("landing failed");
        arming_msg.request.value = false;
        if (arming_client.call(arming_msg))
            ROS_INFO("disarming success");
        else
            ROS_INFO("disarming failed");
    }
    else
    {
        geometry_msgs::PoseStamped postarget_msg;
        postarget_msg.header.stamp = ros::Time::now();
        postarget_msg.header.frame_id = '0';
        postarget_msg.pose.position.x = (point.y - takeoff_pos.y);
        postarget_msg.pose.position.y = -(point.x - takeoff_pos.x);
        postarget_msg.pose.position.z = (point.z - takeoff_pos.z);

        // publish it
        postarget_pub.publish(postarget_msg);
    }
    current_waypoint.x = (point.x - takeoff_pos.x);
    current_waypoint.y = (point.y - takeoff_pos.y);
    current_waypoint.z = (point.z - takeoff_pos.z);
}

int main(int argc, char **argv)
{
    current_waypoint.x = current_waypoint.y = current_waypoint.z = 0;
    ros::init(argc, argv, "fakeGPS");
    ardupilot_link = mavconn::MAVConnInterface::open_url("udp://127.0.0.1:14550@");
    ros::NodeHandle n("~");
    
    n.param<std::string>("vicon_obj", vicon_obj, "cyphyhousecopter");
    n.param<bool>("use_deca", use_deca, false);

    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    postarget_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);
    reached_pub = n.advertise<std_msgs::String>("reached", 1);
 
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getViconPosition);
    ros::Subscriber vel_sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/twist", 1, getViconVelocity);
    ros::Subscriber deca_pos = n.subscribe("decaPos", 1, getDecaPosition);
    ros::Subscriber deca_vel = n.subscribe("decaVel", 1, getDecaVelocity);
    
    ros::Subscriber waypoint = n.subscribe("waypoint", 1, sendWP);
    
    signal(SIGINT, &sigHandler);

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
