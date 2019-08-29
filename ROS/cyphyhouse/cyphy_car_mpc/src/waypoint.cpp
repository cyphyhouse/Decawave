#include <cmath>
#include <cstdbool>
#include <queue>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "MPC.h"

#define WP_RATE 50.0 //Hz
#define PRINT_RATE 50.0 //Hz

#define DELTA_DIRECTION  0.01
#define DELTA_SPEED      0.25
#define EPSILON_RADIUS   0.25
#define EPSILON_ANGLE    0.1

namespace {  // File local variables

std::queue<geometry_msgs::Point> waypoints;

geometry_msgs::Point vicon_position;
geometry_msgs::Quaternion quat; //Get orientation

}

void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_position = pose.pose.position;
    quat = pose.pose.orientation;
}

void getWP(const geometry_msgs::PoseStamped& stamped_point)
{
    const geometry_msgs::Point& point = stamped_point.pose.position;
    waypoints.push(point);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n("~");

    std::string vicon_obj;
    n.param<std::string>("vicon_obj", vicon_obj, "hotdec_car");

    ros::Publisher reached_pub;
    reached_pub = n.advertise<std_msgs::String>("reached", 1);
    ros::Publisher drive_pub;
    drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd", 1);

    ros::Subscriber sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getViconPosition);
    ros::Subscriber waypoint = n.subscribe("waypoint", 10, getWP);  // second parameter is num of buffered messages

    ROS_INFO("Starting waypoint follower");

    ros::Rate r(WP_RATE);

    MPC mpc;

    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        if(waypoints.empty())  // No waypoint to go to
        {
            continue;  // Wait for next waypoint
        }

        const geometry_msgs::Point& curr_loc = vicon_position;
        const geometry_msgs::Point& current_waypoint = waypoints.front();
        // Acknowledge that we reached the desired waypoint
        if (sqrt(pow(curr_loc.x - current_waypoint.x,2) + pow(curr_loc.y - current_waypoint.y,2)) < EPSILON_RADIUS)
        {
            waypoints.pop();  //delete first element

            // tell STARL if waypoint is reached
            // for now assume we only do that once we reach the final dest
            std_msgs::String wp_reached;
            wp_reached.data = "TRUE";
            reached_pub.publish(wp_reached);
            continue;  // Continue to process or wait for next waypoint
        }
        // else  // Compute and publish Ackermann message
        double curr_ang = atan2(
                2 * (quat.x * quat.y + quat.w * quat.z),
                pow(quat.w,2) + pow(quat.x,2) - pow(quat.y,2) - pow(quat.z,2));
        Eigen::VectorXd state(3);
        state << curr_loc.x, curr_loc.y, curr_ang;

        std::vector<double> solution = mpc.Solve(state, current_waypoint);
        const double& direction = solution.at(0);
        const double& speed = solution.at(1);

        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = direction;
        drive_pub.publish(drive_msg);
    }

    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.drive.speed = 0;
    drive_msg.drive.steering_angle = 0;
    drive_pub.publish(drive_msg);

    return 0;
}

