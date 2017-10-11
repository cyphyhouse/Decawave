#include <cstdio>
#include <iostream>
#include <cmath>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <mavconn/interface.h>


#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "ros/package.h"

static const double lat0 = 0, lon0 = 0;
static double old_stamp = ros::Time(0.0).toSec();
static Eigen::Vector3d old_ecef(0,0,0);

static mavconn::MAVConnInterface::Ptr ardupilot_link;

static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
static GeographicLib::LocalCartesian proj(lat0, lon0, 0, earth);

void sendFakeGPS(const geometry_msgs::Point& point)
{
    mavlink::common::msg::HIL_GPS fix {};
    ros::Time stamp = ros::Time::now();
    Eigen::Vector3d current_ecef(point.x, point.y, point.z);

    double lat, lon, h;
    ROS_INFO("x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
    proj.Reverse(point.x, point.y, point.z, lat, lon, h);
    ROS_INFO("latitude: %f, longitude: %f, altitude: %f", lat, lon, h);

    // compute velocity (borrowed from mavros)
    Eigen::Vector3d vel = ((old_ecef - current_ecef) / (stamp.toSec() - old_stamp)) * 1e2;

    // compute course over ground (borrowed from mavros)
    double cog;
    if (vel.x() == 0 && vel.y() == 0) {
        cog = 0;
    }
    else if (vel.x() >= 0 && vel.y() < 0) {
        cog = M_PI * 5 / 2 - atan2(vel.x(), vel.y());
    }
    else {
        cog = M_PI / 2 - atan2(vel.x(), vel.y());
    }

    fix.time_usec = stamp.toNSec() / 1000;
    fix.lat = lat * 1e7;
    fix.lon = lon * 1e7;
    fix.alt = h * 1e3;
    fix.vel = vel.block<2, 1>(0, 0).norm();
    fix.vn = vel.x();
    fix.ve = vel.y();
    fix.vd = vel.z();
    fix.cog = cog * 1e2;
    fix.eph = 2.0 * 1e2;
    fix.epv = 2.0 * 1e2;
    fix.fix_type = 3;
    fix.satellites_visible = 5;

    old_stamp = stamp.toSec();
    old_ecef = current_ecef;

    // send it
    ardupilot_link.get()->send_message_ignore_drop(fix);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fakeGPS");
    ardupilot_link = mavconn::MAVConnInterface::open_url("udp://:14650@");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("decaPos", 1, sendFakeGPS);
    ros::spin();
    return 0;
}
