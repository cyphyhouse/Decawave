/*! ----------------------------------------------------------------------------
 * @file    lcd.h
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

#ifndef _LCD_H_
#define _LCD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "deca_types.h"

#define EVB1000_LCD_SUPPORT             (1)

#if (EVB1000_LCD_SUPPORT == 1)
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetoLCD()
 *
 * Low level abstract function to write data to the LCD display via SPI2 peripheral
 * Takes byte buffer and rs_enable signals
 * or returns -1 if there was an error
 */
void writetoLCD
(
    uint32       bodylength,
    uint8        rs_enable,
    const uint8 *bodyBuffer
);
#else
#define writetoLCD(x)
#endif

#if (EVB1000_LCD_SUPPORT == 1)
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
void lcd_display_str(const char *string);
#else
#define lcd_display_str(x)
#endif

#ifdef __cplusplus
}
#endif

#endif /* _LCD_H_ */
