#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <iostream>

#include "tdoa.h"
#include "serial/serial.h"


#define DEVICE "/dev/ttyACM0"
#define SPEED    115200
#define MSG_SIZE 9

#define MSG_TYPE_BYTE 0
#define ANCN_BYTE     1
#define ANCR_BYTE     2
#define DATA_BYTE     3
#define CS_BYTE       7


#define TYPE_TDOA     0xAA

#define SERIAL_BUF_SIZE 32



static volatile int stopflag = 1;

uint8_t ignoring_flag = 0;
uint8_t RX_idx;
uint8_t An, Ar, dist_recv;
float tdoaDistDiff;
float x_hat[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
uint8_t msg[SERIAL_BUF_SIZE];

uint8_t buf[SERIAL_BUF_SIZE];

void sigHandler(int signum)
{
    if(signum == SIGINT)
    {
        printf("Stopping\n");
        stopflag = 0;
    }
}

float to_float(uint8_t* buf)
{
    uint16_t char_for_move[2];
    char_for_move[1] = (buf[0]<<8) | (buf[1]);
    char_for_move[0] = (buf[2]<<8) | (buf[3]);
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

int main(int arc, char *argv[])
{
    serial::Serial my_serial(DEVICE, SPEED, serial::Timeout::simpleTimeout(1000));

    signal(SIGINT, &sigHandler);

    int n, bytes_avail;

    RX_idx = 0;
    dist_recv = 0;
    
    TDOA deca_pos;

    while(stopflag)
    {
        bytes_avail = my_serial.available();
        if(bytes_avail > 0)
        {
            n = my_serial.read(&buf[RX_idx], 1);
            if(RX_idx == MSG_TYPE_BYTE)
            {
                if((buf[0] == TYPE_TDOA))
                {
                    ignoring_flag = 0;
                }
                else
                {
                    printf("Unknown msg type: 0x%02X\n", buf[0]);
                    ignoring_flag = 1;
                    RX_idx = 0;
                }
            }

            if(ignoring_flag == 0)
            {
                RX_idx++;
                if(RX_idx == MSG_SIZE)
                {
                    uint16_t cs = (buf[MSG_SIZE-2] << 8) | buf[MSG_SIZE-1];
                    if(cs == serial_checksum(buf, MSG_SIZE-2))
                    {
                        An = buf[ANCN_BYTE];
                        Ar = buf[ANCR_BYTE];
                        tdoaDistDiff = to_float(&buf[DATA_BYTE]);
                        dist_recv = 1;
                    }
                    else
                    {
                        printf("Error: Wrong checksum. Dump: ");
                        for(int i = 0; i<RX_idx; i++)
                            printf("0x%02X ", buf[i]);
                        printf(". Expected %04X", serial_checksum(buf, MSG_SIZE-2));
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
                deca_pos.stateEstimatorPredict();
                deca_pos.getLocation();
            }
            
            deca_pos.stateEstimatorAddProcessNoise();
            
            deca_pos.scalarTDOADistUpdate(An, Ar, tdoaDistDiff);
            
            deca_pos.stateEstimatorFinalize();
        }
    }
    
    my_serial.close();
}
