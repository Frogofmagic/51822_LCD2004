#include "nrf_drv_twi.h"
#include "nrf_drv_common.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "1602_driver.h"
#include "pca10028.h"
#include <stdio.h>



//====================I2C==================//
/* Indicates if reading operation from accelerometer has ended. */
static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
static volatile bool m_set_mode_done = false;
/* TWI instance. */
static const nrf_drv_twi_t m_twi_lcd1602 = NRF_DRV_TWI_INSTANCE(0);

bool twi_busy;

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{       
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            twi_busy = false;
            break;
        default:
            break;        
    }   
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    
    const nrf_drv_twi_config_t m_twi_lcd1602_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_lcd1602, &m_twi_lcd1602_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_lcd1602);
		twi_busy = false;
}
//====================I2C END==================//


// CONSTANT  definitions
// ---------------------------------------------------------------------------

// flags for backlight control
/*!
 @defined 
 @abstract   LCD_NOBACKLIGHT
 @discussion NO BACKLIGHT MASK
 */
#define LCD_NOBACKLIGHT 0x00

/*!
 @defined 
 @abstract   LCD_BACKLIGHT
 @discussion BACKLIGHT MASK used when backlight is on
 */
#define LCD_BACKLIGHT   0xFF


// Default library configuration parameters used by class constructor with
// only the I2C address field.
// ---------------------------------------------------------------------------
/*!
 @defined 
 @abstract   Enable bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Enable
 */
#define EN 6  // Enable bit

/*!
 @defined 
 @abstract   Read/Write bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Rw pin
 */
#define RW 5  // Read/Write bit

/*!
 @defined 
 @abstract   Register bit of the LCD
 @discussion Defines the IO of the expander connected to the LCD Register select pin
 */
#define RS 4  // Register select bit

/*!
 @defined 
 @abstract   LCD dataline allocation this library only supports 4 bit LCD control
 mode.
 @discussion D4, D5, D6, D7 LCD data lines pin mapping of the extender module
 */
#define D4 0
#define D5 1
#define D6 2
#define D7 3

	
uint8_t _Addr;             // I2C Address of the IO expander
uint8_t _backlightPinMask; // Backlight IO pin mask
uint8_t _backlightStsMask; // Backlight status mask   
uint8_t _En;               // LCD expander word for enable pin
uint8_t _Rw;               // LCD expander word for R/W pin
uint8_t _Rs;               // LCD expander word for Register Select pin
uint8_t _data_pins[4];     // LCD data lines
//I2CIO   _i2cio;            // I2CIO PCF8574* expansion module driver I2CLCDextraIO

uint8_t _displayfunction;  // LCD_5x10DOTS or LCD_5x8DOTS, LCD_4BITMODE or 
													// LCD_8BITMODE, LCD_1LINE or LCD_2LINE
uint8_t _displaycontrol;   // LCD base control command LCD on/off, blink, cursor
													// all commands are "ored" to its contents.
uint8_t _displaymode;      // Text entry mode to the LCD
uint8_t _numlines;         // Number of lines of the LCD, initialized with begin()
uint8_t _cols;             // Number of columns in the LCD



t_backlighPol _polarity;   // Backlight polarity


void lcd_begin(uint8_t cols, uint8_t lines, uint8_t dotsize) 
{
	 
	
   if (lines > 1) 
   {
      _displayfunction |= LCD_2LINE;
   }
   _numlines = lines;
   _cols = cols;
   
   // for some 1 line displays you can select a 10 pixel high font
   // ------------------------------------------------------------
   if ((dotsize != LCD_5x8DOTS) && (lines == 1)) 
   {
      _displayfunction |= LCD_5x10DOTS;
   }
   
   // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
   // according to datasheet, we need at least 40ms after power rises above 2.7V
   // before sending commands. Arduino can turn on way before 4.5V so we'll wait 
   // 50
   // ---------------------------------------------------------------------------
   nrf_delay_ms (100); // 100ms delay
   
   //put the LCD into 4 bit or 8 bit mode
   // -------------------------------------
   if (! (_displayfunction & LCD_8BITMODE)) 
   {
      // this is according to the hitachi HD44780 datasheet
      // figure 24, pg 46
      
      // we start in 8bit mode, try to set 4 bit mode
      // Special case of "Function Set"
      lcd_i2c_send(0x03, FOUR_BITS);
      nrf_delay_ms(5); // wait min 4.1ms
      
      // second try
      lcd_i2c_send(0x03, FOUR_BITS);
      nrf_delay_us(150); // wait min 100us
      
      // third go!
      lcd_i2c_send( 0x03, FOUR_BITS );
      nrf_delay_us(150); // wait min of 100us
      
      // finally, set to 4-bit interface
      lcd_i2c_send ( 0x02, FOUR_BITS );
      nrf_delay_us(150); // wait min of 100us

   } 
   else 
   {
      // this is according to the hitachi HD44780 datasheet
      // page 45 figure 23
      
      // Send function set command sequence
      lcd_command(LCD_FUNCTIONSET | _displayfunction);
      nrf_delay_ms(5);  // wait more than 4.1ms
      
      // second try
      lcd_command(LCD_FUNCTIONSET | _displayfunction);
      nrf_delay_us(150);
      
      // third go
      lcd_command(LCD_FUNCTIONSET | _displayfunction);
      nrf_delay_us(150);

   }
   
   // finally, set # lines, font size, etc.
   lcd_command(LCD_FUNCTIONSET | _displayfunction);
   nrf_delay_us(60);  // wait more
   
   // turn the display on with no cursor or blinking default
   _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;  
   lcd_display();
   
   // clear the LCD
   lcd_clear();
   
   // Initialize to default text direction (for romance languages)
   _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
   // set the entry mode
   lcd_command(LCD_ENTRYMODESET | _displaymode);

   lcd_backlight();

}

// Common LCD Commands
// ---------------------------------------------------------------------------
void lcd_clear()
{
   lcd_command(LCD_CLEARDISPLAY);             // clear display, set cursor position to zero
   nrf_delay_us(HOME_CLEAR_EXEC);    // this command is time consuming
}

void lcd_home()
{
   lcd_command(LCD_RETURNHOME);             // set cursor position to zero
   nrf_delay_us(HOME_CLEAR_EXEC);  // This command is time consuming
}

void lcd_setCursor(uint8_t col, uint8_t row)
{
   const uint8_t row_offsetsDef[]   = { 0x00, 0x40, 0x14, 0x54 }; // For regular LCDs
   const uint8_t row_offsetsLarge[] = { 0x00, 0x40, 0x10, 0x50 }; // For 16x4 LCDs
   
   if ( row >= _numlines ) 
   {
      row = _numlines-1;    // rows start at 0
   }
   
   // 16x4 LCDs have special memory map layout
   // ----------------------------------------
   if ( _cols == 16 && _numlines == 4 )
   {
      lcd_command(LCD_SETDDRAMADDR | (col + row_offsetsLarge[row]));
   }
   else 
   {
      lcd_command(LCD_SETDDRAMADDR | (col + row_offsetsDef[row]));
   }
   
}

// Turn the display on/off
void lcd_noDisplay() 
{
   _displaycontrol &= ~LCD_DISPLAYON;
   lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcd_display() 
{
   _displaycontrol |= LCD_DISPLAYON;
   lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void lcd_noCursor() 
{
   _displaycontrol &= ~LCD_CURSORON;
   lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void lcd_cursor() 
{
   _displaycontrol |= LCD_CURSORON;
   lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns on/off the blinking cursor
void lcd_noBlink() 
{
   _displaycontrol &= ~LCD_BLINKON;
   lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcd_blink() 
{
   _displaycontrol |= LCD_BLINKON;
   lcd_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void lcd_scrollDisplayLeft(void) 
{
   lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void lcd_scrollDisplayRight(void) 
{
   lcd_command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_leftToRight(void) 
{
   _displaymode |= LCD_ENTRYLEFT;
   lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void lcd_rightToLeft(void) 
{
   _displaymode &= ~LCD_ENTRYLEFT;
   lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This method moves the cursor one space to the right
void lcd_moveCursorRight(void)
{
   lcd_command(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVERIGHT);
}

// This method moves the cursor one space to the left
void lcd_moveCursorLeft(void)
{
   lcd_command(LCD_CURSORSHIFT | LCD_CURSORMOVE | LCD_MOVELEFT);
}


// This will 'right justify' text from the cursor
void lcd_autoscroll(void) 
{
   _displaymode |= LCD_ENTRYSHIFTINCREMENT;
   lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void lcd_noAutoscroll(void) 
{
   _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
   lcd_command(LCD_ENTRYMODESET | _displaymode);
}

// Write to CGRAM of new characters
void lcd_createChar(uint8_t location, uint8_t charmap[]) 
{
   location &= 0x7;            // we only have 8 locations 0-7
   
   lcd_command(LCD_SETCGRAMADDR | (location << 3));
   nrf_delay_us(30);
   
   for (uint8_t i = 0; i < 8; i++)
   {
      lcd_write(charmap[i]);      // call the virtual write method
      nrf_delay_us(40);
   }
}


//
// Switch on the backlight
void lcd_backlight ( void )
{
   lcd_i2c_setBacklight(255);
}

//
// Switch off the backlight
void lcd_noBacklight ( void )
{
   lcd_i2c_setBacklight(0);
}

//
// Switch fully on the LCD (backlight and LCD)
void lcd_on ( void )
{
   lcd_display();
   lcd_backlight();
}

//
// Switch fully off the LCD (backlight and LCD) 
void lcd_off ( void )
{
   lcd_noBacklight();
   lcd_noDisplay();
}

// General LCD commands - generic methods used by the rest of the commands
// ---------------------------------------------------------------------------
void lcd_command(uint8_t value) 
{
   lcd_i2c_send(value, COMMAND);
}

void lcd_write(uint8_t value)
{
   lcd_i2c_send(value, DATA);
}



//---------------------------------------------------------------------------//
//----------------------------I2C driver function----------------------------//
//---------------------------------------------------------------------------//
void lcd_i2c_io_initial(uint8_t lcd_Addr, uint8_t En, uint8_t Rw,
                                     uint8_t Rs, uint8_t d4, uint8_t d5,
                                     uint8_t d6, uint8_t d7, uint8_t backlighPin, 
                                     t_backlighPol pol)
{
   lcd_i2c_config(lcd_Addr, En, Rw, Rs, d4, d5, d6, d7);
   lcd_i2c_setBacklightPin(backlighPin, pol);
}


// begin
void lcd_i2c_begin(uint8_t cols, uint8_t lines, uint8_t dotsize) 
{
   
   lcd_i2c_init();     // Initialise the I2C expander interface
   lcd_begin( cols, lines, dotsize );   
}


// User commands - users can expand this section
//----------------------------------------------------------------------------
// Turn the (optional) backlight off/on

//
// setBacklight
void lcd_i2c_setBacklight( uint8_t value ) 
{
   // Check if backlight is available
   // ----------------------------------------------------
   if ( _backlightPinMask != 0x0 )
   {
      // Check for polarity to configure mask accordingly
      // ----------------------------------------------------------
      if  (((_polarity == POSITIVE) && (value > 0)) || 
           ((_polarity == NEGATIVE ) && ( value == 0 )))
      {
         _backlightStsMask = _backlightPinMask & LCD_BACKLIGHT;
      }
      else 
      {
         _backlightStsMask = _backlightPinMask & LCD_NOBACKLIGHT;
      }
      //_i2cio.write( _backlightStsMask );
   }
}

//
// setBacklightPin
void lcd_i2c_setBacklightPin ( uint8_t value, t_backlighPol pol)
{
   _backlightPinMask = ( 1 << value );
   _polarity = pol;
   lcd_i2c_setBacklight(BACKLIGHT_OFF);
}

// PRIVATE METHODS
// ---------------------------------------------------------------------------

//
// init
int lcd_i2c_init()
{
   int status = 0;
   
   // initialize the backpack IO expander
   // and display functions.
   // ------------------------------------------------------------------------
//   if ( _i2cio.begin ( _Addr ) == 1 )
//   {
//      _i2cio.portMode ( OUTPUT );  // Set the entire IO extender to OUTPUT
//      _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
//      status = 1;
//      _i2cio.write(0);  // Set the entire port to LOW
//   }
	 twi_init(); 
	 uint8_t i2c_data; 
	 ret_code_t err_code;
	
	 i2c_data = 0;
	 while(twi_busy){};
	 err_code = nrf_drv_twi_tx(&m_twi_lcd1602, LCD_ADDR, &i2c_data, sizeof(i2c_data), true); 
	 APP_ERROR_CHECK(err_code);
	 twi_busy = true;	
	 _displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
   status = 1;
   return ( status );
}

//
// config
void lcd_i2c_config (uint8_t lcd_Addr, uint8_t En, uint8_t Rw, uint8_t Rs, 
                                uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7 )
{
   _Addr = lcd_Addr;
   
   _backlightPinMask = 0;
   _backlightStsMask = LCD_NOBACKLIGHT;
   _polarity = POSITIVE;
   
   _En = ( 1 << En );
   _Rw = ( 1 << Rw );
   _Rs = ( 1 << Rs );
   
   // Initialise pin mapping
   _data_pins[0] = ( 1 << d4 );
   _data_pins[1] = ( 1 << d5 );
   _data_pins[2] = ( 1 << d6 );
   _data_pins[3] = ( 1 << d7 );   
}



// low level data pushing commands
//----------------------------------------------------------------------------

//
// send - write either command or data
void lcd_i2c_send(uint8_t value, uint8_t mode) 
{
   // No need to use the delay routines since the time taken to write takes
   // longer that what is needed both for toggling and enable pin an to execute
   // the command.
   
   if ( mode == FOUR_BITS )
   {
      lcd_i2c_write4bits( (value & 0x0F), COMMAND );
   }
   else 
   {
      lcd_i2c_write4bits( (value >> 4), mode );
      lcd_i2c_write4bits( (value & 0x0F), mode);
   }
}

//
// write4bits
void lcd_i2c_write4bits ( uint8_t value, uint8_t mode ) 
{
   uint8_t pinMapValue = 0;
   
   // Map the value to LCD pin mapping
   // --------------------------------
   for ( uint8_t i = 0; i < 4; i++ )
   {
      if ( ( value & 0x1 ) == 1 )
      {
         pinMapValue |= _data_pins[i];
      }
      value = ( value >> 1 );
   }
   
   // Is it a command or data
   // -----------------------
   if ( mode == DATA )
   {
      mode = _Rs;
   }
   
   pinMapValue |= mode | _backlightStsMask;
   lcd_i2c_pulseEnable ( pinMapValue );
}

//
// pulseEnable
void lcd_i2c_pulseEnable (uint8_t data)
{
	 uint8_t i2c_data[2]; 
	 ret_code_t err_code;
	
	 i2c_data[0] = data | _En;
	 i2c_data[1] = data & ~_En;
	 //while(twi_busy){};
	 err_code = nrf_drv_twi_tx(&m_twi_lcd1602, LCD_ADDR, i2c_data, sizeof(i2c_data), true); 
	 APP_ERROR_CHECK(err_code);
	 //twi_busy = true;

   //_i2cio.write (data | _En);   // En HIGH
   //_i2cio.write (data & ~_En);  // En LOW
}


