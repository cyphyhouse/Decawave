#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <sstream>

#include "tdoa.h"
#include "serial/serial.h"


#define DEVICE        "/dev/ttyACM0"
#define SPEED         B115200
#define MSG_SIZE      9

#define MSG_TYPE_BYTE 0
#define ANCN_BYTE     1
#define ANCR_BYTE     2
#define DATA_BYTE     3
#define CS_BYTE       7


#define TYPE_TDOA     0xAA

#define SERIAL_BUF_SIZE 32


uint8_t ignoring_flag = 0;
uint8_t RX_idx;
uint8_t An, Ar, dist_recv;
float tdoaDistDiff;
uint8_t serial_msg[SERIAL_BUF_SIZE];


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

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "decaNode");
    ros::NodeHandle nh;
    ros::NodeHandle private_handle("~");
    ros::Publisher decaPos_pub = nh.advertise<geometry_msgs::Point>("decaPos", 1);
    
    std::string device_port;
    private_handle.getParam("deca_port", device_port);
    serial::Serial my_serial(device_port, SPEED, serial::Timeout::simpleTimeout(1000));

    int bytes_avail;

    RX_idx = 0;
    dist_recv = 0;
    
    TDOA deca_ekf;

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
                    printf("Unknown msg type: 0x%02X\n", serial_msg[0]);
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
                        dist_recv = 1;
                    }
                    else
                    {
                        printf("Error: Wrong checksum. Dump: ");
                        for(int i = 0; i<RX_idx; i++)
                            printf("0x%02X ", serial_msg[i]);
                        printf(". Expected %04X", serial_checksum(serial_msg, MSG_SIZE-2));
                        printf("\n");
                    }
                    RX_idx = 0;
                }
            }
        }
        
        if(dist_recv) //Received distance measurement. Can calculate stuff now
        {
            if(Ar == 0)
            {
                deca_ekf.stateEstimatorPredict();
                vec3d_t pos = deca_ekf.getLocation();
                geometry_msgs::Point pos_msg;
                pos_msg.x = pos.x;
                pos_msg.y = pos.y;
                pos_msg.z = pos.z;
                decaPos_pub.publish(pos_msg);
            }
            
            deca_ekf.stateEstimatorAddProcessNoise();
            
            deca_ekf.scalarTDOADistUpdate(An, Ar, tdoaDistDiff);
            
            deca_ekf.stateEstimatorFinalize();
        }
    }
    
    my_serial.close();
    std::cout << "Closed serial" << std::endl;
}
