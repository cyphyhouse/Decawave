/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   main loop for the DecaRanging application
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
/* Includes */

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <assert.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include "deca_device_api.h"
#include "deca_regs.h"
#include "sleep.h"
#include "lcd.h"
#include "port_deca.h"
#include "tdoa_anc.h"



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

    return devID;
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

static uint32_t timeout;
static SemaphoreHandle_t irqSemaphore;

static void uwbTask(void* parameters)
{
	while(1)
	{
		if (xSemaphoreTake(irqSemaphore, timeout))
		{
			do{
				dwt_isr();
			} while(port_CheckEXT_IRQ() == 1);
		}
		else
		{
			rx_to_cb(NULL);
			//timeout = 0xffffffffUL;
			//timeout = 10000;
			timeout = portMAX_DELAY;
		}
	}
}

//#pragma GCC optimize ("O3")
static void main_task(void *pvParameters)
{
    int i;

    led_off(LED_ALL); //turn off all the LEDs

    peripherals_init();

    static StaticSemaphore_t irqSemaphoreBuffer;
	irqSemaphore = xSemaphoreCreateBinaryStatic(&irqSemaphoreBuffer);

//	sleep_ms(1000); //wait for LCD to power on

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
		return; //error
	}

	/* Display application name on LCD. */
	//char lcd_str[16];
	//sprintf(lcd_str, "TDOA v0.51 Anc:%d", (((s1switch & 0x10) << 2) + (s1switch & 0x20) + ((s1switch & 0x40) >> 2)) >> 4);
	//char lcd_str[16] = {'T','D','O','A',' ','v','0','.','5','2',' ','A','n','c',':','9'};
	char lcd_str[16] = "TDOA v0.57 Anc:x";
	lcd_str[15] = ((((s1switch & 0x10) << 2) + (s1switch & 0x20) + ((s1switch & 0x40) >> 2)) >> 4) + 0x30; //converts to ASCII number
	lcd_display_str(lcd_str);

//	sleep_ms(1000);

	int sw_mode = 1;
	tdoa_init(s1switch, &chConfig[sw_mode]);

#ifdef USB_SUPPORT //this is defined in the port.h file
	// Configure USB for output, (i.e. not USB to SPI)
	usb_printconfig(16, (uint8 *)SOFTWARE_VER_STRING, s1switch);
#endif

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

    static StaticTask_t uwbStaticTask;
	static StackType_t uwbStaticStack[2*configMINIMAL_STACK_SIZE];

	xTaskCreateStatic(uwbTask, "uwb", 2*configMINIMAL_STACK_SIZE, NULL,configMAX_PRIORITIES - 1, uwbStaticStack, &uwbStaticTask);

    // main loop
    while(1)
    {
		//Do something

#ifdef USB_SUPPORT //this is set in the port.h file

        //led_on(LED_PC7);
        usb_run();
        //led_off(LED_PC7);
#endif
    }

}

static StaticTask_t xMainTask;
static StackType_t ucMainStack[configMINIMAL_STACK_SIZE];

int main()
{
	clock_init();

	xTaskCreateStatic( main_task, "main", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, ucMainStack, &xMainTask );

	vTaskStartScheduler();

	while(1);

	return 0;
}

// Freertos required callbacks
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
	static StaticTask_t xTimerTaskTCB;
	static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vAssertCalled( unsigned long ulLine, const char * const pcFileName )
{
	printf("Assert failed at %s:%lu", pcFileName, ulLine);
	while(1);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn EXTI9_5_IRQHandler()
 *
 * @brief Handler for DW1000 IRQ.
 *
 * @param none
 *
 * @return none
 */
void EXTI9_5_IRQHandler(void)
{
	BaseType_t higherPriorityTaskWoken = pdFALSE;
    /*do
    {
        port_deca_isr();
    } while (port_CheckEXT_IRQ() == 1);*/
	xSemaphoreGiveFromISR(irqSemaphore, &higherPriorityTaskWoken);
    /* Clear EXTI Line 5 Pending Bit */
    EXTI_ClearITPendingBit(DECAIRQ_EXTI);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
