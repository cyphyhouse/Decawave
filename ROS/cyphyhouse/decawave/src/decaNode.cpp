#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "ros/package.h"

#include "Eigen/Dense"
#include "tdoa.h"
#include "serial/serial.h"


#define DEVICE        "/dev/ttyACM0"
#define SPEED         115200
#define MSG_SIZE      9

#define MSG_TYPE_BYTE 0
#define ANCN_BYTE     1
#define ANCR_BYTE     2
#define DATA_BYTE     3
#define CS_BYTE       7


#define TYPE_TDOA     0xAA

#define SERIAL_BUF_SIZE 32

#define SLEEP_TIME_MICROS 10

uint8_t ignoring_flag = 0;
uint8_t RX_idx;
uint8_t An, Ar, dist_recv;
float tdoaDistDiff;
uint8_t serial_msg[SERIAL_BUF_SIZE];

Eigen::MatrixXf P;
Eigen::MatrixXf A;
Eigen::MatrixXf Q;


float to_float(uint8_t* buff)
{
    uint16_t char_for_move[2];
    char_for_move[1] = (buff[0]<<8) | (buff[1]);
    char_for_move[0] = (buff[2]<<8) | (buff[3]);
    return(*(float*)(char_for_move));
}

/* Fletcher checksum, borrowed from wikipedia */
uint16_t serial_checksum(const uint8_t *data, size_t len)
{
    uint16_t sum1 = 0xff, sum2 = 0xff;

    while (len) {
        unsigned tlen = len > 21 ? 21 : len;
        len -= tlen;
        do {
            sum1 += *data++;
            sum2 += sum1;
        } while (--tlen);
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
    }
    /* Second reduction step to reduce sums to 16 bits */
    sum1 = (sum1 & 0xff) + (sum1 >> 8);
    sum2 = (sum2 & 0xff) + (sum2 >> 8);
    return sum2 << 8 | sum1;
}

void initAnchors(TDOA &ekf)
{
    std::string path = ros::package::getPath("decawave");
    std::ifstream file( path+"/config/anchorPos.txt");
    std::string str;
    float x, y, z;
    int i = 0;
    while (std::getline(file, str))
    {
        sscanf(str.c_str(), "%f, %f, %f", &x,&y,&z);
        ekf.setAncPosition(i, x, y, z);
        i++;
    }
}

void initRobotMatrices(std::string type);

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "decaNode");
    ros::NodeHandle nh;
    ros::NodeHandle private_handle("~");
    ros::Publisher decaPos_pub = nh.advertise<geometry_msgs::Point>("decaPos", 1);
	ros::Publisher decaVel_pub = nh.advertise<geometry_msgs::Point>("decaVel", 1);
    
    std::string device_port;
    private_handle.getParam("deca_port", device_port);
    serial::Serial my_serial(device_port, SPEED, serial::Timeout::simpleTimeout(1000));

    int bytes_avail;

    RX_idx = 0;
    dist_recv = 0;
    
    std::string robot_type;
    private_handle.getParam("robot_type", robot_type);
    initRobotMatrices(robot_type);
    
    vec3d_t initial_position = {0,0,0};
    
    TDOA deca_ekf(A, P, Q, initial_position);
    
    initAnchors(deca_ekf);
    
    ros::Time last_pub = ros::Time::now();

    while(ros::ok())
    {
        bytes_avail = my_serial.available();
        if(bytes_avail > 0)
        {
            my_serial.read(&serial_msg[RX_idx], 1);
            if(RX_idx == MSG_TYPE_BYTE)
            {
                if((serial_msg[0] == TYPE_TDOA))
                {
                    ignoring_flag = 0;
                }
                else
                {
                    //printf("Unknown msg type: 0x%02X\n", serial_msg[0]);
                    ignoring_flag = 1;
                    RX_idx = 0;
                }
            }

            if(ignoring_flag == 0)
            {
                RX_idx++;
                if(RX_idx == MSG_SIZE)
                {
                    uint16_t cs = (serial_msg[MSG_SIZE-2] << 8) | serial_msg[MSG_SIZE-1];
                    if(cs == serial_checksum(serial_msg, MSG_SIZE-2))
                    {
                        An = serial_msg[ANCN_BYTE];
                        Ar = serial_msg[ANCR_BYTE];
                        tdoaDistDiff = to_float(&serial_msg[DATA_BYTE]);

			            ros::Time t_now = ros::Time::now();
                        ros::Duration t = t_now - last_pub;
                        if(t.toSec() >= 0.01)
                        {
                            vec3d_t pos = deca_ekf.getLocation();
                            geometry_msgs::Point pos_msg;
                            pos_msg.x = pos.x;
                            pos_msg.y = pos.y;
                            pos_msg.z = pos.z;
                            decaPos_pub.publish(pos_msg);
                            
                            vec3d_t vel = deca_ekf.getVelocity();
                            geometry_msgs::Point vel_msg;
                            vel_msg.x = vel.x;
                            vel_msg.y = vel.y;
                            vel_msg.z = vel.z;
                            decaVel_pub.publish(vel_msg);
                            
                            last_pub = t_now;
                            deca_ekf.stateEstimatorPredict(t.toSec());
			            }
			            
			            deca_ekf.stateEstimatorAddProcessNoise();
			            
			            deca_ekf.scalarTDOADistUpdate(An, Ar, tdoaDistDiff);
			            
			            deca_ekf.stateEstimatorFinalize();
			            

		            }
                    
                    else
                    {
                        //printf("Error: Wrong checksum. Dump: ");
                        //for(int i = 0; i<RX_idx; i++)
                        //    printf("0x%02X ", serial_msg[i]);
                        //printf(". Expected %04X", serial_checksum(serial_msg, MSG_SIZE-2));
                        //printf("\n");
                    }
                    RX_idx = 0;
                }
            }
        }
	else
	{
		std::this_thread::sleep_for(std::chrono::microseconds(SLEEP_TIME_MICROS));
	}
        
    }
    my_serial.close();
    std::cout << "Closed serial" << std::endl;
}

void initRobotMatrices(std::string type)
{
    if (type == "car")
    {
        A.setIdentity(6,6);
        A(2,2) = 0;
        A(5,5) = 0;
        
        P.setZero(6,6);
        P(0,0) = 10000;
        P(1,1) = 10000;
        P(2,2) = 0;
        P(3,3) = 1e-4;
        P(4,4) = 1e-4;
        P(5,5) = 0;
        
        Q.setZero(6,6);
    }
    else if (type == "quadcopter")
    {
        A.setIdentity(6,6);
        
        P.setZero(6,6);
        P(0,0) = 10000;
        P(1,1) = 10000;
        P(2,2) = 100;
        P(3,3) = 1e-4;
        P(4,4) = 1e-4;
        P(5,5) = 1e-4;
        
        Q.setZero(6,6);
    }
    else
    {
        A.setIdentity(6,6);
        
        P.setZero(6,6);
        P(0,0) = 10000;
        P(1,1) = 10000;
        P(2,2) = 100;
        P(3,3) = 1e-4;
        P(4,4) = 1e-4;
        P(5,5) = 1e-4;
        
        Q.setZero(6,6);
    }
}

