/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "1602_driver.h"



/**
 * @brief Function for main application entry.
 */
int main(void)
{
    //printf("\n\rTWI sensor example\r\n");

		lcd_i2c_io_initial(LCD_ADDR, 4, 5, 6, 0, 1, 2, 3, 7, NEGATIVE);  // Set the LCD I2C address
		lcd_i2c_begin(20,4, LCD_5x8DOTS);
		
		lcd_setCursor(0,2);
		lcd_write('K');
		lcd_write('i');
		lcd_write('r');
		lcd_write('b');
		lcd_write('y');
		
    while(true)
    {
				

			
    }
}



/** @} */
