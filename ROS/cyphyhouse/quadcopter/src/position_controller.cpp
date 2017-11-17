#include <cstdio>
#include <iostream>
#include <cmath>

#include <eigen_conversions/eigen_msg.h>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/package.h"

Eigen::Vector3d current_pos, velSetpoint;


class PID
{
public:
    
    // Contructor
    PID(double Kp, double Ki, double Kd, double Rate, double outLimit)
	{
		kp = kp;
		ki = ki;
		kd = kd;
		dt = 1/Rate;
		samplingRate = Rate;
		integralLimit = 5000.0;
		outputLimit = outLimit;
		lastMeasured = 0;
		integ = 0;
	}
	
	double pidUpdate(double measured)
	{
		double error = setpoint - measured;
		double dInput = (measured - lastMeasured)*dt;
		integ += error*dt;
		integ = fmax(fmin(integ, integralLimit), -integralLimit);
		
		double output = kp*error + kd*dInput + ki*integ;
		output = fmax(fmin(output, outputLimit), -outputLimit);
	}
	void setSetpoint(double _setpoint)
	{
		setpoint = _setpoint;
	}
private:
    //variables
    double kp, ki, kd, dt, samplingRate, integralLimit, outputLimit;
	double lastMeasured, integ;
    double setpoint;
};

PID pidX(2,0,0,10,1.1);
PID pidY(2,0,0,10,1.1);
PID pidZ(2,0.5,0,10,1.1);
PID pidVX(25,1,0,10,22);
PID pidVY(25,1,0,10,22);
PID pidVZ(25,15,0,10,1);

void getPosition(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    current_pos.x = point.x;
	current_pos.y = point.y;
	current_pos.z = point.z;
}

void getWaypoint(const geometry_msgs::Point& point)
{
    pidX.setSetpoint(point.x);
	pidY.setSetpoint(point.y);
	pidZ.setSetpoint(point.z);
}

void sendAttitude()
{
	//Do stuff
	velSetpoint.x = pidX.pidUpdate(current_pos.x);
	velSetpoint.y = pidY.pidUpdate(current_pos.y);
	velSetpoint.z = pidZ.pidUpdate(current_pos.z);
	
	pidVX.setSetpoint(velSetpoint.x);
	pidVY.setSetpoint(velSetpoint.y);
	pidVZ.setSetpoint(velSetpoint.z);
	
	rollRaw = pidVX.pidUpdate(current_vel.x);
	pitchRaw = pidVY.pidUpdate(current_vel.y);
	
	thrustRaw = pidVZ.pidUpdate(current_vel.z);
	
	//q = AngleAxisf(rollRaw, Vector3f::UnitX())* AngleAxisf(pitchRaw, Vector3f::UnitY())* AngleAxisf(0, Vector3f::UnitZ());
	
	geometry_msgs::PoseStamped attmsg;
	attmsg.header.stamp = ros::Time::now().toNSec() / 1000;
	attmsg.pose.pose.x = fmax(fmin(rollRaw,20),-20);
	attmsg.pose.pose.y = fmax(fmin(pitchRaw,20),-20);
	attmsg.pose.pose.z = 0;
	attPub.publish(attmsg);
	std_msgs::Float64 thrustmsg = thrustRaw;
	thrustPub.publish(thrustmsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fakeGPS");
    ardupilot_link = mavconn::MAVConnInterface::open_url("udp://127.0.0.1:14550@");
    ros::NodeHandle n;
    //client = n.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");
    ros::Subscriber sub = n.subscribe("/vrpn_client_node/cyphyhousecopter/pose", 1, getPosition);
    ros::Subscriber waypoint = n.subscribe("/starl/waypoints", 1, getWaypoint);  // second parameter is num of buffered messages
	
	ros::Publisher attPub = n.advertise<geometry_msgs::PoseStamped>("attitude",1);
	ros::Publisher thrustPub = n.advertise<std_msgs::Float64>("att_throttle ",1);
	
	//TODO: Add arming, takeoff, and land commands
	
	ros::Rate loop_rate(10);
	
	while(ros::ok())
	{
		sendAttitude();
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}
