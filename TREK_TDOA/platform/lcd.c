/*! ----------------------------------------------------------------------------
 * @file    lcd.c
 * @brief   EVB1000 LCD screen access functions
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include <string.h>

#include "sleep.h"
#include "port_deca.h"
#include "lcd.h"

#if (EVB1000_LCD_SUPPORT == 1)

void writetoLCD
(
    uint32       bodylength,
    uint8        rs_enable,
    const uint8 *bodyBuffer
)
{

    int i = 0;
    int sleep = 0;
    //int j = 10000;

    if(rs_enable)
    {
        port_LCD_RS_set();
    }
    else
    {
        if(bodylength == 1)
        {
            if(bodyBuffer[0] & 0x3) //if this is command = 1 or 2 - exsecution time is > 1ms
                sleep = 1 ;
        }
        port_LCD_RS_clear();
    }

    port_SPIy_clear_chip_select();  //CS low


    //while(j--); //delay

    for(i=0; i<bodylength; i++)
    {
        port_SPIy_send_data(bodyBuffer[i]); //send data on the SPI

        while (port_SPIy_no_data()); //wait for rx buffer to fill

        port_SPIy_receive_data(); //this clears RXNE bit
    }

    //j = 10000;

    port_LCD_RS_clear();

    //while(j--); //delay

    port_SPIy_set_chip_select();  //CS high

    if(sleep)
        sleep_ms(2);
} // end writetoLCD()

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn lcd_display_str()
 *
 * @brief Display a string on the LCD screen.
 * /!\ The string must be 16 chars long maximum!
 *
 * @param  string  the string to display
 *
 * @return none
 */
void lcd_display_str(const char *string)
{
    uint8 command;
    /* Return cursor home and clear screen. */
    command = 0x2;
    writetoLCD(1, 0, &command);
    command = 0x1;
    writetoLCD(1, 0, &command);
    /* Write the string to display. */
    writetoLCD(strlen(string), 1, (const uint8 *)string);
}

#endif
