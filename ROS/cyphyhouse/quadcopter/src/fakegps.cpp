#include <cstdio>
#include <iostream>
#include <cmath>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "ros/package.h"

static const double lat0 = 0, lon0 = 0;

static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
static GeographicLib::LocalCartesian proj(lat0, lon0, 0, earth);

void sendFakeGPS(const geometry_msgs::Point& point)
{
    double lat, lon, h;
    ROS_INFO("x: %f, y: %f, z: %f\n", point.x, point.y, point.z);
    proj.Reverse(point.x, point.y, point.z, lat, lon, h);
    ROS_INFO("latitude: %f, longitude: %f, altitude: %f", lat, lon, h);
    
    double phi, theta;
    double r = 6371000;
    theta = 90-atan2(r,point.y)*180/3.14159265359;
    phi = 90-acos(point.x/r)*180/3.14159265359;
    ROS_INFO("Approx lat: %f, Approx lon: %f", theta, phi);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fakeGPS");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("decaPos", 1, sendFakeGPS);
    ros::spin();
    return 0;
}
