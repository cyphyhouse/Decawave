/* 
 *
 */
#ifndef _TDOA_TAG_H_
#define _TDOA_TAG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "deca_types.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "port_deca.h"

#define NR_OF_ANCHORS        6
#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT          (0x00FFFFFFFFFFUL)  // DW1000 counter is 40 bits
#define MASK_TXDTS          (0x00FFFFFFFE00UL)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.
#define PACKET_TYPE_RANGE    0x21
#define MAX_DISTANCE_DIFF   (10.0f)

typedef union dwTime_u {
	uint8 raw[5];
	uint64_t full;
	struct {
		uint32 low32;
		uint8 high8;
	}__attribute__((packed));
	struct {
		uint8 low8;
		uint32 high32;
	}__attribute__((packed));
} dwTime_t;

typedef struct {
	uint8 prevAnc;
	uint8 currAnc;
	float distanceDiff;
} usb_msg_t;

typedef struct rangePacket_s {
	uint8 type;
	uint8 txMaster[5];				//TX time at master
	uint8 timestamps[NR_OF_ANCHORS][5];	//Relevant time for anchors
}__attribute__((packed)) rangePacket_t;

typedef struct packet_s {
	union {
		uint16_t fcf;
		struct {
			uint16_t type:3;
			uint16_t security:1;
			uint16_t framePending:1;
			uint16_t ack:1;
			uint16_t ipan:1;
			uint16_t reserved:3;
			uint16_t destAddrMode:2;
			uint16_t version:2;
			uint16_t srcAddrMode:2;
		} fcf_s;
	};

	uint8_t seq;
	uint16_t pan;
	uint8_t destAddress[8];
	uint8_t sourceAddress[8];

	uint8_t payload[64];
} __attribute__((packed)) packet_t;

uint8 anc_prf, anc_chan;
uint32 tx_failed_count;

usb_msg_t usbData;
volatile uint8 usbDataReady;

void tdoa_init(uint8 s1switch, dwt_config_t *config);

void rx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);


#define CIR_PWR_OFFSET	0x06
#define TWOPOWER17		131072.0f // 2^17
#define DISTANCE_OF_RADIO_INV 213.139451293f
void dwCorrectTimestamp(dwTime_t* timestamp);
float dwGetReceivePower(void);
float calculatePower(float base, float N);

#ifdef __cplusplus
}
#endif

#endif
