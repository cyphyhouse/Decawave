#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <mavconn/interface.h>
#include <mavros_msgs/WaypointPush.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "ros/package.h"

static const double lat0 = 40.116, lon0 = -88.224;	// IRL GPS coords
ros::Time old_stamp = ros::Time(0.0);
ros::ServiceClient client;
geometry_msgs::Vector3 current_vel;
geometry_msgs::Point vicon_position;
static GeographicLib::Geoid egm96_5("egm96-5", "", true, true);

static mavconn::MAVConnInterface::Ptr ardupilot_link;

static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
static GeographicLib::LocalCartesian proj(lat0, lon0, 0, earth);

void getVelocity(const geometry_msgs::Point& velocity)
{
    current_vel.x = velocity.x;
    current_vel.y = velocity.y;
    current_vel.z = velocity.z;
}

void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_position = pose.pose.position;
}

void sendFakeGPS(const geometry_msgs::Point& point)
{
    mavlink::common::msg::HIL_GPS fix {};
    ros::Time stamp = ros::Time::now();

    if ((stamp - old_stamp) < ros::Duration(0.1))   // throttle incoming messages to 10 Hz
        return;

    double lat, lon, h;
    ROS_INFO("x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
    proj.Reverse(point.y, -point.x, point.z, lat, lon, h);
    ROS_INFO("latitude: %f, longitude: %f, altitude: %f", lat, lon, (h + GeographicLib::Geoid::ELLIPSOIDTOGEOID * egm96_5(lat, lon)));

    std::ofstream positionFile;
    positionFile.open ("/home/pi/copterpos.txt", std::ios::app);
    positionFile << stamp.toNSec() / 1000 << ", ";
    positionFile << vicon_position.x << ", " << vicon_position.y << ", " << vicon_position.z << ", ";
    positionFile << point.x << ", " << point.y << ", " << point.z << "\n";
    positionFile.close();

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
    fix.alt = (h + GeographicLib::Geoid::ELLIPSOIDTOGEOID * egm96_5(lat, lon)) * 1e3;
    // fix.alt = h * 1e3;
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

void sendWP(const geometry_msgs::Point& point)
{
    mavros_msgs::WaypointPush waypoint_msg {};
    mavros_msgs::Waypoint waypoint {};
    double lat, lon, h;
    proj.Reverse(point.y, -point.x, point.z, lat, lon, h);
    waypoint_msg.request.start_index = 0;
    waypoint.frame = 0; // global frame, not sure if that important
    waypoint.command = 16;  // enum MAV_CMD_NAV_WAYPOINT
    waypoint.is_current = true;
    waypoint.autocontinue = true;   // autocontinue to next waypoint
    waypoint.param1 = 1;    // hold time in secs
    waypoint.param2 = 0.5;  // acceptance radius in meters
    waypoint.param3 = 0;    // pass through waypoint
    waypoint.param4 = NAN;  // yaw angle (don't need it so NaN)
    waypoint.x_lat = lat;
    waypoint.y_long = lon;
    waypoint.z_alt = h;

    // populate waypoint table with waypoint message (it's a vector)
    waypoint_msg.request.waypoints.push_back(waypoint);

    // send it using service
    client.call(waypoint_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fakeGPS");
    ardupilot_link = mavconn::MAVConnInterface::open_url("udp://127.0.0.1:14550@");
    ros::NodeHandle n;
    client = n.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
    ros::Subscriber vel_sub = n.subscribe("/decaVel", 1, getVelocity);
    ros::Subscriber vicon_pos = n.subscribe("/vrpn_client_node/cyphyhousecopter/pose", 1, getViconPosition);
    ros::Subscriber sub = n.subscribe("/decaPos", 1, sendFakeGPS);
    ros::Subscriber waypoint = n.subscribe("/starl/waypoints", 1, sendWP);  // second parameter is num of buffered messages
    ros::spin();
    return 0;
}
