#include <cstdio>
#include <iostream>
#include <cmath>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "ros/package.h"

static const double lat0 = 40 + 7/60.0, lon0 = -88 - 14/60.0; // CSL GPS coords

static GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
static GeographicLib::LocalCartesian proj(lat0, lon0, 0, earth);

void sendFakeGPS(const geometry_msgs::Point& point)
{
    double lat, lon, h;
    ROS_INFO("x: %f\ny: %f\nz: %f\n", point.x, point.y, point.z);
    proj.Reverse(point.x, point.y, point.z, lat, lon, h);
    ROS_INFO("latitude: %f\nlongitude: %f\naltitude: %f", lat, lon, h);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fakeGPS");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("decaPos", 1, sendFakeGPS);
    ros::spin();
    return 0;
}
