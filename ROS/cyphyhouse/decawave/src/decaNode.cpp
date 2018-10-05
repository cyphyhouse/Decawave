#include <cstdio>
#include <cstdint>
#include <cstdbool>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>

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

#define SLEEP_TIME_MICROS 10 //us

#define PUB_RATE 100 //Hz

uint8_t An, Ar, dist_recv;
float tdoaDistDiff;

TDOA deca_ekf;
std::mutex ekf_mutex;
std::thread serial_thread;

Eigen::MatrixXf P;
Eigen::MatrixXf A;
Eigen::MatrixXf Q;

std::string device_port, robot_type;

ros::Publisher decaPos_pub, decaVel_pub;

//Function prototypes
void initRobotMatrices(std::string type);


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

void serial_comm()
{
    uint8_t RX_idx = 0;
    uint8_t serial_msg[SERIAL_BUF_SIZE];
    bool ignoring_flag = false;
    int bytes_avail;

    serial::Serial my_serial(device_port, SPEED, serial::Timeout::simpleTimeout(1000));

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
                    ignoring_flag = false;
                }
                else
                {
                    ignoring_flag = true;
                    RX_idx = 0;
                }
            }

            if(ignoring_flag == false)
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

			            ekf_mutex.lock();
			            deca_ekf.scalarTDOADistUpdate(An, Ar, tdoaDistDiff);
                        //deca_ekf.stateEstimatorFinalize(); //Commented out because it doesnt do anything right now
			            ekf_mutex.unlock();

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

void pub_state(const vec3d_t p, const vec3d_t v)
{
    geometry_msgs::Point pos_msg, vel_msg;
    pos_msg.x = p.x;
    pos_msg.y = p.y;
    pos_msg.z = p.z;
    decaPos_pub.publish(pos_msg);
    
    vel_msg.x = v.x;
    vel_msg.y = v.y;
    vel_msg.z = v.z;
    decaVel_pub.publish(vel_msg);
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "decaNode");
    ros::NodeHandle nh("~");
    decaPos_pub = nh.advertise<geometry_msgs::Point>("decaPos", 1);
	decaVel_pub = nh.advertise<geometry_msgs::Point>("decaVel", 1);
    
    nh.param<std::string>("deca_port", device_port, "/dev/ttyACM0");
    nh.param<std::string>("robot_type", robot_type, "quadcopter");

    

    initRobotMatrices(robot_type);
    
    vec3d_t initial_position = {0,0,0};
    
    //TDOA deca_ekf(A, P, Q, initial_position);
    deca_ekf.setPredictionMat(P);
    deca_ekf.setTransitionMat(A);
    deca_ekf.setCovarianceMat(Q);
    
    initAnchors(deca_ekf);
    
    serial_thread = std::thread(serial_comm);
    
    ros::Rate r(PUB_RATE);

    while(ros::ok())
    {
        ekf_mutex.lock();
        deca_ekf.stateEstimatorPredict(1./PUB_RATE);
        deca_ekf.stateEstimatorAddProcessNoise();
        deca_ekf.stateEstimatorFinalize();
		
        vec3d_t pos = deca_ekf.getLocation();
        vec3d_t vel = deca_ekf.getVelocity();
        ekf_mutex.unlock();
        
        pub_state(pos, vel);
        
        r.sleep();
    }
    
    serial_thread.join();
}

void initRobotMatrices(std::string type)
{
    if (type == "car")
    {
        A.setIdentity(6,6);
        A(5,5) = 0; //No z velocity
        
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

