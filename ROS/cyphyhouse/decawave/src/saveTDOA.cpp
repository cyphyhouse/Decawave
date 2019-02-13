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
#include "geometry_msgs/PoseStamped.h"
#include "ros/package.h"


#include "serial/serial.h"


#define DEVICE        "/dev/ttyACM0"
#define SPEED         1152000
#define MSG_SIZE      9

#define MSG_TYPE_BYTE 0
#define ANCR_BYTE     1
#define ANCN_BYTE     2
#define DATA_BYTE     3
#define CS_BYTE       7


#define TYPE_TDOA     0xAA

#define SERIAL_BUF_SIZE 32

#define SLEEP_TIME_MICROS 10 //us

bool is_init;
uint8_t An, Ar, tdoa_recv;
float tdoaDistDiff;
float tdoaVec[8];

std::thread serial_thread;

std::ofstream positionFile;
ros::Time time_start;

std::string device_port, robot_type, vicon_obj;

geometry_msgs::Point vicon_position;

//Function prototypes


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
            //std::cout << bytes_avail << std::endl;
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
                        Ar = serial_msg[ANCR_BYTE]; //prev_anc
                        An = serial_msg[ANCN_BYTE]; //curr_anc
                        tdoaDistDiff = to_float(&serial_msg[DATA_BYTE]);
                        
                        
                        if (!is_init && (An == 0))
                        {
                            is_init = true;
                        }
                        
                        if (is_init)
                        {
                            if (((Ar+1) & 0x7) == An)  //Check if sequential
                            {
                                tdoaVec[An] = tdoaDistDiff;
                                if(An == 7) //got last anchor
                                {
                                    ros::Duration time_since_start = ros::Time::now() - time_start;
                                    positionFile << time_since_start.toNSec() / 1000 << ", "; //Print time in useconds
                                    positionFile << std::setprecision(8) << vicon_position.x << ", " << vicon_position.y << ", " << vicon_position.z << ", ";
                                    positionFile << std::setprecision(8) << tdoaVec[0] << ", " << tdoaVec[1] << ", " <<  tdoaVec[2] << ", " <<  tdoaVec[3] << ", " <<  tdoaVec[4] << ", " <<  tdoaVec[5] << ", " <<  tdoaVec[6] << ", " <<  tdoaVec[7] << "\r\n";
                                }
                            }
                            else
                            {
                                //Lost a package, delete all data so far
                                memset(tdoaVec, 0, sizeof(tdoaVec));
                                is_init = false;
                                std::cout << "Lost package " << +Ar << ", " << +An << std::endl;
                            }
                        }
			            
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

void getViconPosition(const geometry_msgs::PoseStamped& pose)
{
    vicon_position = pose.pose.position;
}


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "decaNode");
    ros::NodeHandle nh("~");
    
    nh.param<std::string>("deca_port", device_port, "/dev/ttyACM0");
    nh.param<std::string>("robot_type", robot_type, "quadcopter");
    nh.param<std::string>("vicon_obj", vicon_obj, "cyphyhousecopter");

    ros::Subscriber sub = nh.subscribe("/vrpn_client_node/"+vicon_obj+"/pose", 1, getViconPosition);
    
    // Gets the current time so we can add to data output
    char time_buffer[80];
    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo;
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 80, "%G%m%dT%H%M%S", timeinfo);
    
    is_init = false;

    std::cout << "Starting" << std::endl;

    positionFile.open (std::string("/home/pi/tdoaData_")+time_buffer+".txt", std::ios::app);
    time_start = ros::Time::now();
    
    serial_thread = std::thread(serial_comm);
    
    ros::spin();
    
    serial_thread.join();
    
    positionFile.close();
}

