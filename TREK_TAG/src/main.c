#include "compiler.h"
#include "port_deca.h"

#include "deca_types.h"
#include "deca_device_api.h"

#include "sleep.h"
#include "lcd.h"
#include "deca_spi.h"
#include "tdoa_tag.h"



extern void usb_run(void);
extern int usb_init(void);
extern void usb_printconfig(int, uint8*, int);
extern void send_usbmessage(uint8*, int);

#define SWS1_SHF_MODE 0x02	//short frame mode (6.81M)
#define SWS1_CH5_MODE 0x04	//channel 5 mode
#define SWS1_ANC_MODE 0x08  //anchor mode
#define SWS1_A1A_MODE 0x10  //anchor/tag address A1
#define SWS1_A2A_MODE 0x20  //anchor/tag address A2
#define SWS1_A3A_MODE 0x40  //anchor/tag address A3

#define LCD_BUFF_LEN (80)
uint8 dataseq[LCD_BUFF_LEN];

uint8 s1switch = 0;
int dr_mode = 0;
int chan, tagaddr, ancaddr;

typedef struct
{
    uint8 channel ;
    uint8 prf ;
    uint8 datarate ;
    uint8 preambleCode ;
    uint8 preambleLength ;
    uint8 pacSize ;
    uint8 nsSFD ;
    uint16 sfdTO ;
} chConfig_t ;


//Configuration for DecaRangeRTLS TREK Modes (4 default use cases selected by the switch S1 [2,3] on EVB1000, indexed 0 to 3 )
dwt_config_t chConfig[4] ={
	//mode 1 - S1: 2 off, 3 off
	{
		2,              // channel
		DWT_PRF_16M,    // Pulse repetition frequency.
		DWT_PLEN_1024,  // preambleLength
		DWT_PAC32,      // pacSize
		4,				// TX preambleCode
		4,				// RX preambleCode
		1,				// non-standard SFD
		DWT_BR_110K,    // datarate
		DWT_PHRMODE_STD,// PHY header mode
		(1025 + 64 - 32)// SFD timeout (preamble length + 1 + SFD length - PAC size)
	},
	//mode 2 - S1: 2 on, 3 off
	{
		2,              // channel
		DWT_PRF_64M,    // prf
		DWT_PLEN_128,   // preambleLength
		DWT_PAC8,       // pacSize
		9,				// TX preambleCode
		9,				// RX preambleCode
		0,				// non-standard SFD
		DWT_BR_6M8,		// datarate
		DWT_PHRMODE_STD,// PHY header mode
		(129 + 8 - 8)	// SFD timeout (preamble length + 1 + SFD length - PAC size)
	},
	//mode 3 - S1: 2 off, 3 on
	{
		5,              // channel
		DWT_PRF_16M,    // prf
		DWT_PLEN_1024,  // preambleLength
		DWT_PAC32,      // pacSize
		4,              // TX preambleCode
		4,				// RX preambleCode
		1,				// non-standard SFD
		DWT_BR_110K,    // datarate
		DWT_PHRMODE_STD,// PHY header mode
		(1025 + 64 - 32)// SFD timeout (preamble length + 1 + SFD length - PAC size)
	},
	//mode 4 - S1: 2 on, 3 on
	{
		5,              // channel
		DWT_PRF_64M,    // prf
		DWT_PLEN_128,   // preambleLength
		DWT_PAC8,       // pacSize
		9,				// TX preambleCode
		9,				// RX preambleCode
		0,				// non-standard SFD
		DWT_BR_6M8,		// datarate
		DWT_PHRMODE_STD,// PHY header mode
		(129 + 8 - 8)	// SFD timeout (preamble length + 1 + SFD length - PAC size)
	}
};

void initLCD(void)
{
    uint8 initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38 /*0x3C*/, 0x0C, 0x01, 0x06 };
    uint8 command = 0x0;
    int j = 100000;

    writetoLCD( 9, 0,  initseq); //init seq
    while(j--);

    command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
    command = 0x1 ;  //clear screen
    writetoLCD( 1, 0,  &command);
}


uint32 init_deca(uint8 s1switch)
{
    uint32 devID ;

    /* Install DW1000 IRQ handler. */
    port_set_deca_isr(dwt_isr);

	//reset the DW1000 by driving the RSTn line low
    reset_DW1000();
    spi_set_rate_low();  //max SPI before PLLs configured is ~4M


	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        lcd_display_str("INIT FAILED");
        while (1)
        { };
    }
	spi_set_rate_high();

    devID = dwt_readdevid() ;

    if (DWT_DEVICE_ID != devID)   // Means it is NOT DW1000 device
    {
        // SPI not working or Unsupported Device ID
        return(-1) ;
    }

    //instance_anchaddr = (((s1switch & SWS1_A1A_MODE) << 2) + (s1switch & SWS1_A2A_MODE) + ((s1switch & SWS1_A3A_MODE) >> 2)) >> 4;

    int sw_mode = 1;
	dwt_configure(&chConfig[sw_mode]);

	dwt_setleds(1);

    return devID;
}

uint16_t serial_checksum(uint8_t* data, int len)
{
	uint16 sum1 = 0xFF, sum2 = 0xFF;

	while (len)
	{
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
	sum1 = (sum1 & 0xFF) + (sum1 >> 8);
	sum2 = (sum2 & 0xFF) + (sum2 >> 8);
	return sum2 << 8 | sum1;
}


/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/


/*
 * @fn      main()
 * @brief   main entry point
**/

//#pragma GCC optimize ("O3")
int main(void)
{
	int i;

	led_off(LED_ALL); //turn off all the LEDs

	peripherals_init();

	memset(dataseq, 0, LCD_BUFF_LEN);
	memcpy(dataseq, (const uint8 *) "DECAWAVE        ", 16);
	writetoLCD( 40, 1, dataseq); //send some data
	memcpy(dataseq, (const uint8 *) "TDOA TAG        ", 16); // Also set at line #26 (Should make this from single value !!!)
	writetoLCD( 16, 1, dataseq); //send some data

	sleep_ms(1000);

	usb_init();

	sleep_ms(1000);

	s1switch = is_button_low(0) << 1 // is_switch_on(TA_SW1_2) << 2
			| is_switch_on(TA_SW1_3) << 2
			| is_switch_on(TA_SW1_4) << 3
			| is_switch_on(TA_SW1_5) << 4
			| is_switch_on(TA_SW1_6) << 5
			| is_switch_on(TA_SW1_7) << 6
			| is_switch_on(TA_SW1_8) << 7;

	port_DisableEXT_IRQ(); //disable ScenSor IRQ until we configure the device


    //run TDOA application for TREK

	led_off(LED_ALL);

	if(init_deca(s1switch) == (uint32)-1)
	{
		led_on(LED_ALL); //to display error....
		lcd_display_str("  INIT FAIL ");
		return 0; //error
	}

	/* Display application name on LCD. */
	char lcd_str[16] = "TDOA v0.45 TAG:x";
	lcd_str[15] = ((((s1switch & 0x10) << 2) + (s1switch & 0x20) + ((s1switch & 0x40) >> 2)) >> 4) + 0x30; //converts to ASCII number
	lcd_display_str(lcd_str);

	sleep_ms(1000);

	int sw_mode = 1;
	tdoa_init(s1switch, &chConfig[sw_mode]);

	//sleep for 5 seconds displaying last LCD message and flashing LEDs
	i=30;
	while(i--)
	{
		if (i & 1) led_off(LED_ALL);
		else    led_on(LED_ALL);

		sleep_ms(200);
	}
	i = 0;
	led_off(LED_ALL);

	port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting

    // Enable RX so we can start receiving messages
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // main loop
	while(1)
	{
		// Check if we have data ready
		if(usbDataReady == 1)
		{
			uint8 str_to_send[9];
			str_to_send[0] = 0xAA;
			str_to_send[1] = usbData.prevAnc;
			str_to_send[2] = usbData.currAnc;
			uint32_t asInt = *((int32_t*)&usbData.distanceDiff);
			str_to_send[3] = (asInt>>24);
			str_to_send[4] = (asInt>>16);
			str_to_send[5] = (asInt>>8);
			str_to_send[6] = (asInt);
 			uint16_t cs = serial_checksum(&str_to_send[0], 7);
			str_to_send[7] = (cs>>8);
			str_to_send[8] = (cs);
			send_usbmessage(str_to_send, 9);
			usb_run();
			usbDataReady = 0;
		}
	}

}
