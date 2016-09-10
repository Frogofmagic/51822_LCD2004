 nRF51822 -> I2C -> LCD2004
 SCL P0.07
 SDA P0.30
 Nordic SDK11
 Path C:\Users\Kirby\Desktop\nRF5_SDK_11\examples\peripheral\
 driver file name:1602_driver.c/.h

1. Most of code is from http://arduino-info.wikispaces.com/LCD-Blue-I2C.
   But it's for Arduino.

2. My LCD2004 only can work with the example called "I2C LCD DISPLAY VERSION 2:".

3. Change I2C address in 1602_driver.h  
   define LCD_ADDR LCD_2004_ADDR

