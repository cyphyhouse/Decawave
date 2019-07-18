#define _USE_MATH_DEFINES
#include <cstdio>
#include <iostream>
#include <cmath>
#include <thread>


#include <eigen_conversions/eigen_msg.h>
#include <mavros/mavros_plugin.h>

#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Thrust.h>
#include <std_msgs/String.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/package.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define GPS_RATE 10 //Hz
#define PRINT_RATE 100 //Hz
#define CONTROLLER_RATE 100 // Hz

bool takeoff_flag = false;
bool starl_flag = false;
bool isFlying = false;

std::string bot_num, vicon_obj;
bool use_deca;

ros::ServiceClient arming_client;
ros::ServiceClient mode_client;

ros::Publisher atttarget_pub, thrusttarget_pub;
ros::Publisher reached_pub;

geometry_msgs::Pose vicon_pose;
geometry_msgs::Twist vicon_vel;
geometry_msgs::Point current_waypoint;  // VICON coords


std::thread gps_thread, print_thread, pos_thread;

class PID
{
public:
    // Contructor
    PID(double Kp, double Ki, double Kd, double Rate, double outLimit)
    {
        kp = Kp;
        ki = Ki;
        kd = Kd;
        dt = 1/Rate;
        samplingRate = Rate;
        integralLimit = outLimit;
        outputLimit = outLimit;
        lastMeasured = 0;
        integ = 0;
        deriv = 0;
    }
    
    double pidUpdate(double measured)
    {
        error = setpoint - measured;
        double dInput = (measured - lastMeasured)/dt;
        lastMeasured = measured;
	deriv = 0.9 * dInput + 0.1 * deriv;
        integ += error*dt;
        integ = fmax(fmin(integ, integralLimit), -integralLimit);
        
        output = kp*error + kd*dInput + ki*integ;
        return fmax(fmin(output, outputLimit), -outputLimit);
    }
    
    void setSetpoint(double _setpoint)
    {
        setpoint = _setpoint;
    }
    
    void reset()
    {
        integ = 0;
        deriv = 0;
    }
    
    double getSetpoint() { return setpoint; }
    double getError() { return error; }
    double getOutput() { return output; }
    double getInteg() { return integ; }
    double getDeriv() { return deriv; }
    double getKP() { return kp; }

private:
    //variables
    double kp, ki, kd, dt, samplingRate, integralLimit, outputLimit;
    double lastMeasured, integ, deriv, setpoint, error, output;
};

// For now, x&y PIDs generate desired roll and pitch, in the future they will generate desired velocities, which in turn generate desired roll/pitch
PID pidX(1, 0, 0, CONTROLLER_RATE, 0.3); // Generate desired pitch
PID pidY(1, 0, 0, CONTROLLER_RATE, 0.3); // Generate desired roll

PID pidZ(1, 0.5, 0, CONTROLLER_RATE, 1); // Generate desired thrust

PID pidRoll(0.15, 0.1, 0.004, CONTROLLER_RATE, 100.0); // Generate desired Roll rate
PID pidPitch(0.15, 0.1, 0.004, CONTROLLER_RATE, 100.0); // Generate desired Pitch rate
PID pidYaw(0.2, 0.1, 0.004, CONTROLLER_RATE, 100.0); // Generate desired Yaw rate


void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_pose = pose.pose;
}

void getViconVelocity(const geometry_msgs::TwistStamped& twist)
{
    vicon_vel = twist.twist;
}

void printPos()
{
    ros::Rate pr(1);
    while(ros::ok())
    {
        ROS_INFO("x: %f, y: %f, z: %f\n", vicon_pose.position.x, vicon_pose.position.y, vicon_pose.position.z);
        /*Eigen::Quaterniond q = Eigen::Quaterniond(vicon_pose.orientation.w, vicon_pose.orientation.x, vicon_pose.orientation.y, vicon_pose.orientation.z);
        Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
	//if (rpy(0) > M_PI) rpy(0) -= 2*M_PI;
	//if (rpy(1) > M_PI) rpy(1) -= 2*M_PI; 
	//if (rpy(2) > M_PI) rpy(2) -= 2*M_PI; 
	Eigen::Vector3d rpyBody;
        rpyBody(0) = -(rpy(1) * cos(rpy(2))) + (rpy(0) * sin(rpy(2)));
        rpyBody(1) = -(rpy(0) * cos(rpy(2))) - (rpy(1) * sin(rpy(2)));
	rpyBody(2) = rpy(2);

	ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f\n", rpyBody(0), rpyBody(1), rpyBody(2));
        */
	pr.sleep();
    }
}

void sendAttitude()
{
    ros::Rate r(CONTROLLER_RATE);

    while(ros::ok())
    {
        geometry_msgs::Point point = vicon_pose.position;
        
        // Resent first point after takeoff
        if (takeoff_flag && sqrt(pow(point.z - current_waypoint.z, 2)) < 0.3)
        {
            pidX.setSetpoint(point.x);
            pidY.setSetpoint(point.y);
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

        if(isFlying)
        {
            Eigen::Quaterniond q = Eigen::Quaterniond(vicon_pose.orientation.w, vicon_pose.orientation.x, vicon_pose.orientation.y, vicon_pose.orientation.z);
            Eigen::Vector3d rpyVicon = q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
            
            geometry_msgs::Vector3 rpySetpoint, rpyRateSetpoint, rpyBody;
            double thrust;
            rpySetpoint.x = pidX.pidUpdate(vicon_pose.position.x);
            rpySetpoint.y = pidY.pidUpdate(vicon_pose.position.y);
            
            thrust = pidZ.pidUpdate(vicon_pose.position.z);
            
            rpyBody.x = -(rpySetpoint.y * cos(rpyVicon(2))) + (rpySetpoint.x * sin(rpyVicon(2)));
            rpyBody.y = -(rpySetpoint.x * cos(rpyVicon(2))) - (rpySetpoint.y * sin(rpyVicon(2)));
            
            pidRoll.setSetpoint(rpyBody.y);
            pidPitch.setSetpoint(rpyBody.x);
            pidYaw.setSetpoint(0);
            
            rpyRateSetpoint.x = pidRoll.pidUpdate(rpyVicon(0));
            rpyRateSetpoint.y = pidPitch.pidUpdate(rpyVicon(1));
            rpyRateSetpoint.z = pidYaw.pidUpdate(rpyVicon(2));
            
            geometry_msgs::TwistStamped velmsg;
            velmsg.header.stamp = ros::Time::now();
            velmsg.twist.angular = rpyRateSetpoint;
            atttarget_pub.publish(velmsg);
            
            mavros_msgs::Thrust thrustmsg;
            thrustmsg.header.stamp = ros::Time::now();
            thrustmsg.thrust = thrust;
            thrusttarget_pub.publish(thrustmsg);
	    //ROS_INFO("Thrust: %f, T setpoint: %f, Error: %f, Output: %f\n", thrust, pidZ.getSetpoint(), pidZ.getError(), pidZ.getOutput());
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
        positionFile << vicon_pose.position.x << ", " << vicon_pose.position.y << ", " << vicon_pose.position.z << "\r\n";
        
        printrate.sleep();
    }
    positionFile.close();
}

void sendWP(const geometry_msgs::PoseStamped& stamped_point)
{
    geometry_msgs::Point point = stamped_point.pose.position;
    std::string stamp = stamped_point.header.frame_id;
    starl_flag = true;
    std::cout << "Going to point x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
    if (stamp == "0")    // takeoff
    {
        mavros_msgs::CommandBool arming_msg;
        arming_msg.request.value = true;
        if (arming_client.call(arming_msg))
            ROS_INFO("arming success");
        else
            ROS_INFO("arming failed");
            
        isFlying = true;
        
        pidX.setSetpoint(vicon_pose.position.x);
        pidY.setSetpoint(vicon_pose.position.y);
        pidZ.setSetpoint(point.z);
    }
    else if (stamp == "2")   // land
    {
        mavros_msgs::CommandBool arming_msg;
        arming_msg.request.value = false;
        if (arming_client.call(arming_msg))
            ROS_INFO("disarming success");
        else
            ROS_INFO("disarming failed");
        
        pidX.setSetpoint(vicon_pose.position.x);
        pidY.setSetpoint(vicon_pose.position.y);
        pidZ.setSetpoint(point.z);
    }
    else
    {
            pidX.setSetpoint(point.x);
            pidY.setSetpoint(point.y);
            pidZ.setSetpoint(point.z);
    }
    
    pidX.reset();
    pidY.reset();
    pidZ.reset();
    
    current_waypoint.x = point.x;
    current_waypoint.y = point.y;
    current_waypoint.z = point.z;
}

int main(int argc, char **argv)
{
    current_waypoint.x = current_waypoint.y = current_waypoint.z = 0;
    ros::init(argc, argv, "posHold");
    ros::NodeHandle n("~");
    
    n.param<std::string>("vicon_obj", vicon_obj, "cyphyhousecopter");
    n.param<bool>("use_deca", use_deca, false);

    arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    atttarget_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 1);
    thrusttarget_pub = n.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 1);
    reached_pub = n.advertise<std_msgs::String>("reached", 1);
 
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getViconPosition);
    ros::Subscriber vel_sub = n.subscribe("/vrpn_client_node/"+vicon_obj+"/twist", 1, getViconVelocity);
    
    ros::Subscriber waypoint = n.subscribe("waypoint", 1, sendWP);
    
    gps_thread = std::thread(sendAttitude);
    pos_thread = std::thread(printPos);
    //print_thread = std::thread(printToFile);

    ros::spin();
    
    gps_thread.join();
    pos_thread.join();
    //print_thread.join();
    return 0;
}
