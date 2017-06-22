/*! ----------------------------------------------------------------------------
 * @file	deca_usb.c
 * @brief	DecaWave USB application state machine for USB to SPI feature
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
#include "deca_usb.h"

#include "deca_types.h"
#include "deca_device_api.h"
#include "deca_spi.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usb_conf.h"
#include "usbd_desc.h"

#include "string.h"
#include "sleep.h"

/** @defgroup USB_VCP_Private_Variables
  * @{
  */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;


//USB to SPI data buffers
int local_buff_length = 0;
uint8_t local_buff[8000];
uint16_t local_buff_offset = 0;
int tx_buff_length = 0;
uint8_t tx_buff[8000];
int local_have_data = 0;

int version_size;
uint8* version;
int s1configswitch;
extern uint32_t APP_Rx_length;
extern void setLCDline1(uint8 s1switch);


uint16_t DW_VCP_Init     (void) { return USBD_OK; }
uint16_t DW_VCP_DeInit   (void) { return USBD_OK; }
uint16_t VCP_COMConfig(uint8_t Conf) { return USBD_OK; }

/**
  * @brief  DW_VCP_Ctrl
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the opeartion (USBD_OK in all cases)
  */
uint16_t DW_VCP_Ctrl (uint32_t Cmd, uint8_t* Buf, uint32_t Len)
{
   return USBD_OK;
}

//example functions to interface to USB VCOM
/**
  * @brief  DW_VCP_DataTx
  *         CDC received data to be send over USB IN endpoint are managed in
  *         this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
  */
#pragma GCC optimize ("O3")
uint16_t DW_VCP_DataTx (uint8_t* Buf, uint32_t Len)
{
	int i = 0;

	i = APP_Rx_ptr_in ;

	if ((APP_Rx_length+Len) < APP_RX_DATA_SIZE)
	{
		int l;
		/* Get the data to be sent */
		for (l = 0; l < Len; l++)
		{
			APP_Rx_Buffer[i++] = Buf[l];
			/* Increment the in pointer */
			if (i>=APP_RX_DATA_SIZE)
				i=0;
		}
	}

	APP_Rx_ptr_in = i;

	return USBD_OK;
}


/**
  * @brief  DW_VCP_DataRx
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         until exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (i.e. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
//#pragma GCC optimize ("O3")
uint16_t DW_VCP_DataRx (uint8_t* Buf, uint32_t Len)
{
  uint32_t i;

  // ZS: This is where PC (USB Tx) data is received
  for (i = 0; i < Len; i++)
  {
	  if((i + local_buff_offset) < 10000)
	  {
		  local_buff[i + local_buff_offset] = Buf[i];
	  }
  }

  local_buff_length = Len + local_buff_offset;
  //

  if((local_buff[2] + (local_buff[3]<<8)) == local_buff_length)
  {
	  local_buff_offset = 0;
	  local_have_data = 1;
  }
  else
  {
	   if((local_buff_length == 5) && (local_buff[0] == 100))
	   {
		   local_buff_offset = 0;
		   local_have_data = 1;
	   }
	   else if((Len == 5) && (Buf[0] == 100)) //reset the buffer
	   {
		   local_buff_offset = 0;
		   local_have_data = 1;
		   for (i = 0; i < Len; i++)
		   {
			   local_buff[i + local_buff_offset] = Buf[i];
		   }
	   }
	   //we have received a antenna calibration or tx power configuration command
	   else if(local_buff_length == 6)
	   {
		   if(((local_buff[0] == 0x5) && (local_buff[5] == 0x5))
			   || ((local_buff[0] == 0x7) && (local_buff[5] == 0x7)))
		   {
			   local_buff_offset = 0;
			   local_have_data = 1;
		   }
	   }
	   //we have received a S1 configuration command
	   else if((local_buff_length == 3) && (local_buff[0] == 0x6) && (local_buff[2] == 0x6))
	   {
	       local_buff_offset = 0;
	   	   local_have_data = 1;
	   }
	   else
	   {
		   local_buff_offset += Len;
	   }
  }



  return USBD_OK;
}

#pragma GCC optimize ("O3")
int process_usbmessage(void)
{
	int result = 0;
	return result;
}
#pragma GCC optimize ("O3")
void send_usbmessage(uint8 *string, int len)
{
	if(local_have_data == 0)
	{
		memcpy(&tx_buff[0], string, len);
		tx_buff[len] = '\r';
		tx_buff[len+1] = '\n';
		tx_buff_length = len + 2;

		local_have_data = 2;
	}
}
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/


int usb_init(void)
{
	memset(local_buff, 0, 10000);
	led_off(LED_ALL); //to display error....

	// enable/initialise the USB functionality
	USBD_Init(&USB_OTG_dev,USB_OTG_FS_CORE_ID,&USR_desc,&USBD_CDC_cb,&USR_cb);

    return 0;
}

#pragma GCC optimize ("O3")
void usb_run(void)
{
	// Do nothing in foreground -- allow USB application to run

	if(local_have_data == 1)
	{
		local_have_data = process_usbmessage();
	}
	else if(local_have_data == 2) //have data to send (over USB)
	{
		DW_VCP_DataTx(tx_buff, tx_buff_length);
		local_have_data = 0;
	}
}



