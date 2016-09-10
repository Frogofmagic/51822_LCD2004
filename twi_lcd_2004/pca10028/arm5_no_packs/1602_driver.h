#include "app_util_platform.h"

// I2C address
#define LCD_ADDR LCD_2004_ADDR
#define LCD_1602_ADDR  0x3F
#define LCD_2004_ADDR  0x20


/*!
 @defined 
 @abstract   All these definitions shouldn't be used unless you are writing 
 a driver.
 @discussion All these definitions are for driver implementation only and
 shouldn't be used by applications.
 */
// LCD Commands
// ---------------------------------------------------------------------------
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// flags for display entry mode
// ---------------------------------------------------------------------------
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off and cursor control
// ---------------------------------------------------------------------------
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
// ---------------------------------------------------------------------------
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// flags for function set
// ---------------------------------------------------------------------------
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00


// Define COMMAND and DATA LCD Rs (used by send method).
// ---------------------------------------------------------------------------
#define COMMAND                 0
#define DATA                    1
#define FOUR_BITS               2


/*!
 @defined 
 @abstract   Defines the duration of the home and clear commands
 @discussion This constant defines the time it takes for the home and clear
 commands in the LCD - Time in microseconds.
 */
#define HOME_CLEAR_EXEC      2000

/*!
    @defined 
    @abstract   Backlight off constant declaration
    @discussion Used in combination with the setBacklight to swith off the
 LCD backlight. @set setBacklight
*/
#define BACKLIGHT_OFF           0

/*!
 @defined 
 @abstract   Backlight on constant declaration
 @discussion Used in combination with the setBacklight to swith on the
 LCD backlight. @set setBacklight
 */
#define BACKLIGHT_ON          255

typedef enum { POSITIVE, NEGATIVE } t_backlighPol;

void lcd_begin(uint8_t cols, uint8_t lines, uint8_t dotsize);
void lcd_clear(void);
void lcd_home(void);
void lcd_setCursor(uint8_t col, uint8_t row);
void lcd_noDisplay(void);
void lcd_display(void);
void lcd_noCursor(void);
void lcd_cursor(void);
void lcd_noBlink(void);
void lcd_blink(void);
void lcd_scrollDisplayLeft(void);
void lcd_scrollDisplayRight(void);
void lcd_leftToRight(void);
void lcd_rightToLeft(void) ;
void lcd_moveCursorRight(void);
void lcd_moveCursorLeft(void);
void lcd_autoscroll(void) ;
void lcd_noAutoscroll(void) ;
void lcd_createChar(uint8_t location, uint8_t charmap[]) ;
void lcd_backlight (void);
void lcd_noBacklight ( void );
void lcd_on (void);
void lcd_off (void);
void lcd_command(uint8_t value);
void lcd_write(uint8_t value);

void lcd_i2c_io_initial(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs, uint8_t d4, uint8_t d5,
                                     uint8_t d6, uint8_t d7, uint8_t backlighPin, 
                                     t_backlighPol pol);
void lcd_i2c_begin(uint8_t cols, uint8_t lines, uint8_t dotsize);
void lcd_i2c_setBacklight(uint8_t value); 
void lcd_i2c_setBacklightPin ( uint8_t value, t_backlighPol pol);
int lcd_i2c_init(void);
void lcd_i2c_config(uint8_t lcd_Addr, uint8_t En, uint8_t Rw, uint8_t Rs, 
                                  uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
void lcd_i2c_send(uint8_t value, uint8_t mode);
void lcd_i2c_write4bits(uint8_t value, uint8_t mode);
void lcd_i2c_pulseEnable(uint8_t data);
