#include <cstdio>
#include <iostream>
#include <cmath>

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

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/package.h"

static const double lat0 = 40.116, lon0 = -88.224;	// IRL GPS coords
double current_lat, current_lon, current_h = 0;

ros::Time old_stamp = ros::Time(0.0);

ros::ServiceClient arming_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient land_client;
ros::ServiceClient mode_client;

ros::Publisher postarget_pub;

geometry_msgs::Vector3 current_vel;
geometry_msgs::Point deca_position;

static mavconn::MAVConnInterface::Ptr ardupilot_link;

static GeographicLib::Geoid egm96_5("egm96-5", "", true, true);
static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
static GeographicLib::LocalCartesian proj(lat0, lon0, 0, earth);

void getVelocity(const geometry_msgs::TwistStamped& twist)
{
    current_vel.x = twist.twist.linear.x;
    current_vel.y = twist.twist.linear.y;
    current_vel.z = twist.twist.linear.z;
}

void getDecaPosition(const geometry_msgs::Point& point)
{
    deca_position = point;
}

void sendFakeGPS(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    geometry_msgs::Point point = pose->pose.position;
    mavlink::common::msg::HIL_GPS fix {};
    ros::Time stamp = ros::Time::now();

    if ((stamp - old_stamp) < ros::Duration(0.1))   // throttle incoming messages to 10 Hz
        return;

    double lat, lon, h;
    // ROS_INFO("x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
    proj.Reverse(point.y, -point.x, point.z, lat, lon, h);
    // ROS_INFO("latitude: %f, longitude: %f, altitude: %f", lat, lon, h);

    std::ofstream positionFile;
    positionFile.open ("/home/pi/copterpos.txt", std::ios::app);
    positionFile << stamp.toNSec() / 1000 << ", ";
    positionFile << point.x << ", " << point.y << ", " << point.z << "\n";
    positionFile << deca_position.x << ", " << deca_position.y << ", " << deca_position.z << ", ";
    positionFile.close();

    current_lat = lat;
    current_lon = lon;
    current_h = h;

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
    fix.time_usec = stamp.toNSec() / 1000;
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
}

void sendWP(const geometry_msgs::PointStamped& stamped_point)
{
    mavros_msgs::SetMode mode_msg;
    mode_msg.request.base_mode = 0;
    mode_msg.request.custom_mode = "GUIDED";
    mode_client.call(mode_msg);
    geometry_msgs::Point point = stamped_point.point;
    std::string stamp = stamped_point.header.frame_id;
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
        takeoff_msg.request.latitude = current_lat; // This is either the first waypoint or whatever the initial position is
        takeoff_msg.request.longitude = current_lon;
        takeoff_msg.request.altitude = current_h + 1;

        if (takeoff_client.call(takeoff_msg))
            ROS_INFO("takeoff success");
        else
            ROS_INFO("takeoff failed");
    }
    else if (stamp == "2")   // land
    {
        mavros_msgs::CommandBool arming_msg;
        mavros_msgs::CommandTOL land_msg;
        land_msg.request.min_pitch = 0;
        land_msg.request.yaw = 0;
        land_msg.request.latitude = current_lat;
        land_msg.request.longitude = current_lon;
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
        postarget_msg.pose.position.x = point.y;
        postarget_msg.pose.position.y = -point.x;
        postarget_msg.pose.position.z = point.z;

        // publish it
        postarget_pub.publish(postarget_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fakeGPS");
    ardupilot_link = mavconn::MAVConnInterface::open_url("udp://127.0.0.1:14550@");
    ros::NodeHandle n;

    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    land_client = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    postarget_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

    ros::Subscriber vel_sub = n.subscribe("/vrpn_client_node/cyphyhousecopter/twist", 1, getVelocity);
    ros::Subscriber deca_pos = n.subscribe("/decaPos", 1, getDecaPosition);
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/cyphyhousecopter/pose", 1, sendFakeGPS);
    ros::Subscriber waypoint = n.subscribe("/starl/waypoints", 1, sendWP);  // second parameter is num of buffered messages

    ros::ServiceClient sethome_client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");
    mavros_msgs::CommandHome sethome_msg;
    sethome_msg.request.current_gps = false;
    sethome_msg.request.latitude = lat0;
    sethome_msg.request.longitude = lon0;
    sethome_msg.request.altitude = 0;
    sethome_client.call(sethome_msg);

    ros::spin();
    return 0;
}
