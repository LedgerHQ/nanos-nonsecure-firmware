
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"

// for varargs support in printf
#include <stdarg.h>
#include <string.h>

#include "bagl.h"

#ifdef HAVE_GLO091

// framebuffer
#define SCREEN_HEIGHT 32
#define SCREEN_WIDTH 128
char screen_framebuffer[(SCREEN_WIDTH*SCREEN_HEIGHT)/8];
unsigned int screen_changed; // to avoid screen update for nothing

#ifdef HAVE_PRINTF
char newline_requested = 0;
unsigned char current_x = 0;
#define PRINTF_ZONE_HEIGHT (32)
#define PRINTF_FONT_HEIGHT 8
#define PRINTF_FONT_WIDTH 5
#define PRINTF_FONT_ID BAGL_FONT_LUCIDA_CONSOLE_8
#define PRINTF_LINE_CHAR_LENGTH (SCREEN_WIDTH/PRINTF_FONT_WIDTH)
#define PRINTF_LINE_COUNT (PRINTF_ZONE_HEIGHT/PRINTF_FONT_HEIGHT)
unsigned char screen_charbuffer[PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT];
#endif // HAVE_PRINTF


//#define REG_SET(reg, mask, value) reg = ((reg) & ~mask) | value 
void REG_SET(volatile unsigned int * reg, unsigned int pin, unsigned int value) {
  volatile v = 1+pin;
  *reg = (*reg & ~(1<<pin)) | (value<<pin); 
}
#define BB_OUT(port, pin, value) REG_SET(&(port)->ODR, pin, value) 
//#define BB_OUT(port, pin, value) REG_SET((port)->ODR, pin, value) 

#define DISP_PWR(x)    // BB_OUT(GPIOA, 5, x)
/* F042
SPI_SCK =    PA5
SPI_MOSI = PA7
Disp_Reset  PA2
Disp_CS         PA1
Disp_DC        PA0
*/

#define DISP_NCS(x)    BB_OUT(GPIOA, 1, x) // BB_OUT(GPIOA, 6, x)
#define DISP_NRST(x)   BB_OUT(GPIOA, 2, x) // BB_OUT(GPIOA, 7, x)
#define DISP_DC(x)     BB_OUT(GPIOA, 0, x) // BB_OUT(GPIOB, 6, x) 
#define DISP_D6_SCL(x) BB_OUT(GPIOA, 5, x) // BB_OUT(GPIOC, 7, x)
#define DISP_D7_SI(x)  BB_OUT(GPIOA, 7, x) // BB_OUT(GPIOA, 9, x)

void screen_init_pins(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  // configure GPIO  
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*
  // configure GPIO  
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  */
}

void screen_poweroff(void)
{
  /*
  GPIO_InitTypeDef GPIO_InitStruct;
 
  // configure GPIO  
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MOf042 test screen (low clockDE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  */
}

void lcd_data(unsigned char control_DATA, unsigned char d) {
  unsigned char c=8;
  // set command
  DISP_DC(control_DATA);
  // activate the data interface
  DISP_NCS(0);
  // msb to lsb bit transfer
  while(c--) {
    // prepare the clock
    DISP_D6_SCL(0);
    // output the line bit value
    volatile unsigned int bit_value = (d & 0x80);
    DISP_D7_SI( (bit_value)?1:0);
    // raise the clock
    DISP_D6_SCL(1);
    // prepare next bit
    d <<= 1;
  }
  // low clock
  DISP_D6_SCL(0);
  // deactivate the data interface
  DISP_NCS(1);
  // disable A0 (just in case)
  DISP_DC(0);
}
        
void lcd_control(unsigned char control)
{
  lcd_data(0, control);
}
        
void lcd_write(char data)
{
  lcd_data(1, data);
}



const unsigned char screen_init_commands[] = {
  0xAE,              // Display off

  0x00,              // <- column address low = 0
  0x10,              // <- column address high = 0
  0x00,              // <- row address low = 0
  0xB0,              // <- page address = 0
  0x81,              // <- Contract control
  0x80,              // Max contrast 256, orig 128

  //0xA0,              // <- ADC Direction 0-127
  0xA1,              // <- ADC Direction 127-0

  0xA6,              // <- Normal display
  0xA8,              // Multiplex ratio 
  0x1F,              // duty = 1/32
  //0xC0,              // <- SHL Direction 0-31
  0xC8,              // <- SHL Direction 31-0
  0xD3,              // Set display offset
  0x00,              // 0
  0xD9,              // set pre-charge period
  0x1F,              // 
  0xDA,              // set com pins
  0x00,
  0xDB,              // set vcomh
  0x40,
  0x8D,              // set charge pump enable
  0x14,              
  //0xAF,              // <- turn display on
  0xA4,              // <- Display all points to OFF
  0x2E,
  0x21,
  0x0,  // first column
  0x7F, // last column
  0x22,
  0x0, // first page
  0x3, // last page
  0x40, // initial line

  // 0xC0,              // <- SHL Direction 0-31
  // 0xA7,              // <- Reverse display
};

void screen_init(void)
{      
  unsigned int i; 
  screen_init_pins();

  // first, force a hardware reset
  // there isn't a spec for how long this requires 
  // - it's suggested to tie it to the CPU reset -
  // but it has to wait until power is stable
  // so we'll delay for 100ms
  DISP_DC(0);
  DISP_NCS(1);
  DISP_D6_SCL(0);

  // set the ports to be outputs
  DISP_PWR(1);
  DISP_NRST(0);
  // assert delay before starting requests
  //for (volatile int i = 0; i < 1000; i++);
  HAL_Delay(10);
  DISP_NRST(1);
  //for (volatile int i = 0; i < 1000; i++);
  HAL_Delay(10);
  
  for (i=0; i < sizeof(screen_init_commands); i++) {
    lcd_control(screen_init_commands[i]);
  } 
  
  /*
  lcd_control(0xAE);              // Display off

  lcd_control(0x00);              // <- column address low = 0
  lcd_control(0x10);              // <- column address high = 0

  lcd_control(0x00);              // <- row address low = 0
  
  lcd_control(0xB0);              // <- page address = 0

  lcd_control(0x81);              // <- Contract control
  lcd_control(0xFF);              // Max contrast 256, orig 128
  
  lcd_control(0xA1);              // <- ADC Direction 128-0
  
  lcd_control(0xA6);              // <- Normal display
  //lcd_control(0xA7);              // <- Reverse display

  lcd_control(0xA8);              // Multiplex ratio 
  lcd_control(0x1F);              // duty = 1/32

  lcd_control(0xC8);              // <- SHL Direction 31-0
  //lcd_control(0xC0);              // <- SHL Direction 0-31
  
  lcd_control(0xD3);              // Set display offset
  lcd_control(0x00);              // 0

  lcd_control(0xD9);              // set pre-charge period
  lcd_control(0x1F);              // 

  lcd_control(0xDA);              // set com pins
  lcd_control(0x00);
  
  lcd_control(0xDB);              // set vcomh
  lcd_control(0x40);

  lcd_control(0x8D);              // set charge pump enable
  lcd_control(0x14);              
  

//#define TESTLCD
#ifdef TESTLCD
  lcd_control(0xA5);              // <- Display all points to ON
  //for(;;);
  volatile unsigned int q=0x40000;while(q--);
#endif
  lcd_control(0xA4);              // <- Display all points to OFF

  lcd_control(0x2E);

  lcd_control(0x21);
  lcd_control(0x0);  // first column
  lcd_control(0x7F); // last column

  lcd_control(0x22);
  lcd_control(0x0); // first page
  lcd_control(0x3); // last page

  lcd_control(0x40); // initial line
  */

  
//#define TESTLCD
#ifdef TESTLCD
  lcd_control(0xA5);              // <- Display all points to ON
  //for(;;);
  volatile unsigned int q=0x40000;while(q--);
#endif

#ifdef HAVE_PRINTF
  // clear the char buffer
  current_x = 0;
  newline_requested = 0;
  memset(((char*)screen_charbuffer),' ',sizeof(screen_charbuffer));
#endif // HAVE_PRINTF
  
  // clear screen frame buffer
  memset(screen_framebuffer, 0, sizeof(screen_framebuffer));
  screen_changed = 1; // make sure to refresh
  screen_update();

  lcd_control(0xAF);              // <- turn display on
}

void screen_invert(unsigned int inverted) {
  if (inverted) {
    lcd_control(0xA7);
  }
  else {
    lcd_control(0xA6);
  }
  //wait for general status // screen_update(); // immediate effect
}

void screen_rotation(unsigned int degree) {
  switch(degree) {
    default:
    case 0:
      lcd_control(0xC0);              // <- SHL Direction 0-31
      lcd_control(0xA0);              // <- ADC Direction 0-127
      break;
    case 180:
      lcd_control(0xC8);              // <- SHL Direction 31-0
      lcd_control(0xA1);              // <- ADC Direction 127-0
      break;
  }
  //wait for general status // screen_update(); // immediate effect
}

void screen_brightness(unsigned int percentage) {
  lcd_control(0x81); // <- Contract control
  lcd_control(percentage*255/100); // level value
}

/*-------------
refresh the screen, need to be called by application every time the screen's content changes
this method is screen dependent, as it depends on the technology and transport.
--------------*/
void screen_update(void)
{
  unsigned int _x;
  unsigned int _y;

  if (!screen_changed) {
    return;
  }
  // flush the char buffer into the driver and update the screen

  _x = 0;
  _y = 0;

  for (_y=0; _y<(SCREEN_HEIGHT/8); _y++) {
    lcd_control(0xB0|_y);              // <- page address
    for (_x=0; _x<SCREEN_WIDTH; _x++) {
      lcd_write(screen_framebuffer[_y*SCREEN_WIDTH+_x]);
    }
  }
}

void screen_clear(void) {
  memset(screen_framebuffer, 0, sizeof(screen_framebuffer)); 
  screen_changed = 1;
}

int screen_draw_x;
int screen_draw_y;
unsigned int screen_draw_width;
unsigned int screen_draw_height;
int screen_draw_YX;
int screen_draw_YXlinemax;
int screen_draw_Ybitmask;
unsigned int* screen_draw_colors;

void bagl_hal_draw_bitmap_within_rect_internal(unsigned int bit_per_pixel, unsigned char* bitmap, unsigned int bitmap_length_bits) {
  unsigned int i;
  unsigned int xx;  
  //unsigned int pixel_mask = (1<<bit_per_pixel)-1;
  #define pixel_mask 1 // 2 colors only

  int x = screen_draw_x;
  xx = x;
  int y = screen_draw_y;
  unsigned int width = screen_draw_width;
  unsigned int height = screen_draw_height;
  int YX = screen_draw_YX;
  int YXlinemax = screen_draw_YXlinemax;
  int Ybitmask = screen_draw_Ybitmask;
  unsigned int* colors = screen_draw_colors;

  screen_changed=1;

  while(bitmap_length_bits) {
    // horizontal scan transformed into vertical
    unsigned int ch = *bitmap++;
    // draw each pixel (at most 256 index color bitmap support)
    for (i = 0; i < 8 && bitmap_length_bits; bitmap_length_bits -= bit_per_pixel, i += bit_per_pixel) {

      // grab LSB to MSB bits
      // 2 colors only
      // bit    colorsarray     painted
      // 0      [0]=0               0
      // 1      [1]=0               0
      // 0      [0]=1               1
      // 1      [1]=1               1
      // 
      if (y>=0 && xx>=0) { // else we're out of screen
        if (colors[((ch>>i) & pixel_mask)] != 0) {
          screen_framebuffer[YX] |= Ybitmask;
        }
        else {
          screen_framebuffer[YX] &= ~Ybitmask;
        }
      }

      xx++;
      YX++;
      if (YX >= YXlinemax) {
        y++;
        height--;
        // update fast bit operation variables
        YX = (y/8)*SCREEN_WIDTH + x;
        xx = x;
        YXlinemax = YX + width;
        Ybitmask = 1<<(y%8);
      }

      if (height == 0) {
        goto end;
      }
    }
  }

  // save for continue
end:
  screen_draw_x = x;
  screen_draw_y = y;
  screen_draw_width = width;
  screen_draw_height = height;
  screen_draw_YX = YX;
  screen_draw_YXlinemax = YXlinemax;
  screen_draw_Ybitmask = Ybitmask;
  return;
}

void bagl_hal_draw_bitmap_within_rect(int x, int y, unsigned int width, unsigned int height, unsigned int color_count, unsigned int *colors, unsigned int bit_per_pixel, unsigned char* bitmap, unsigned int bitmap_length_bits) {

  // horizontal scan
  if (x>= SCREEN_WIDTH || y >= SCREEN_HEIGHT) {
    return;
  }

  if (x+width > SCREEN_WIDTH) {
    width = SCREEN_WIDTH-x;
  }

  if (y+height > SCREEN_HEIGHT) {
    height = SCREEN_HEIGHT-y;
  }

  int YX = (y/8)*SCREEN_WIDTH + x;
  int Ybitmask = 1<<(y%8); 
  int YXlinemax = YX + width;

  // run bitmap draw
  screen_draw_x = x;
  screen_draw_y = y;
  screen_draw_width = width;
  screen_draw_height = height;
  screen_draw_YX = YX;
  screen_draw_YXlinemax = YXlinemax;
  screen_draw_Ybitmask = Ybitmask;
  screen_draw_colors = colors;
  bagl_hal_draw_bitmap_within_rect_internal(bit_per_pixel, bitmap, bitmap_length_bits);
}

void bagl_hal_draw_bitmap_continue(unsigned int bit_per_pixel, unsigned char* bitmap, unsigned int bitmap_length_bits) {
  bagl_hal_draw_bitmap_within_rect_internal(bit_per_pixel, bitmap, bitmap_length_bits);
}

// draw a simple rect
void bagl_hal_draw_rect(unsigned int color, int x, int y, unsigned int width, unsigned int height) {
  unsigned int i;

  if (x+width > SCREEN_WIDTH) {
    return;
  }
  if (y+height > SCREEN_HEIGHT) {
    return;
  }

  unsigned int YX = (y/8)*SCREEN_WIDTH + x;
  unsigned int Ybitmask = 1<<(y%8); 
  unsigned int YXlinemax = YX + width;

  screen_changed=1;

  i = width*height;
  while(i--) {
    // 2 colors only
    if (color) {
      screen_framebuffer[YX] |= Ybitmask;
    }
    else {
      screen_framebuffer[YX] &= ~Ybitmask;
    }

    YX++;
    if (YX >= YXlinemax) {
      y++;
      height--;
      // update fast bit operation variables
      YX = (y/8)*SCREEN_WIDTH + x;
      YXlinemax = YX + width;
      Ybitmask = 1<<(y%8);
    }

    if (height == 0) {
      break;
    }
  }
}

#ifdef HAVE_PRINTF
void screen_printc(const char ch) {

  if (newline_requested) {
    newline_requested=0;
    current_x=0;
    // rotate n-1 line
    memmove(screen_charbuffer, ((char*)screen_charbuffer)+PRINTF_LINE_CHAR_LENGTH, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT-PRINTF_LINE_CHAR_LENGTH);
    // erase last line
    memset(((char*)screen_charbuffer)+PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT-PRINTF_LINE_CHAR_LENGTH,' ', PRINTF_LINE_CHAR_LENGTH);

    //bagl_hal_draw_rect(0xF9F9F9, 0, SCREEN_HEIGHT- PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT);
    // redraw the whole zone (all chars are printed, no problem with)
    /*
    bagl_draw_string(PRINTF_FONT_ID, 
                   0x000000,
                   0xF9F9F9,
                   0, SCREEN_HEIGHT-PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT,
                   screen_charbuffer, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT,
                   BAGL_ENCODING_LATIN1);
    */
  }

  if(ch==0xA){
    // don't move screen yet, upon next print yes
    newline_requested = 1;

    // refresh surface
    // useless // bagl_hal_draw_rect(0xF9F9F9, 0, SCREEN_HEIGHT- PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT);
    bagl_draw_string(PRINTF_FONT_ID, 
                   0x000000,
                   0xF9F9F9,
                   0, SCREEN_HEIGHT-PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT,
                   screen_charbuffer, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT,
                   BAGL_ENCODING_LATIN1);
  }
  else if(ch=='\r') {
    // erase last line (but don't move text around)
    memset(((char*)screen_charbuffer)+PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT-PRINTF_LINE_CHAR_LENGTH,' ', PRINTF_LINE_CHAR_LENGTH);
  }
  else if (ch >= 0x20 && ch <= 0x7F) {
    // always write at the bottom of the screen
    screen_charbuffer[PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT-PRINTF_LINE_CHAR_LENGTH+current_x] = ch;
    current_x++;
    if (current_x >= PRINTF_LINE_CHAR_LENGTH-1) {
      newline_requested = 1;
    }
  }

  /*
  // rotate n-1 line
  memmove(screen_charbuffer, ((char*)screen_charbuffer)+1, sizeof(screen_charbuffer)-1);
  // erase last line
  screen_charbuffer[sizeof(screen_charbuffer)-1] = ch;
  if (ch == '\n') {
    bagl_hal_draw_rect(0xF9F9F9, 0, SCREEN_HEIGHT- PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT);
    bagl_draw_string(PRINTF_FONT_ID, 
                   0x000000,
                   0xF9F9F9,
                   0, SCREEN_HEIGHT-PRINTF_ZONE_HEIGHT, SCREEN_WIDTH, PRINTF_ZONE_HEIGHT,
                   screen_charbuffer, PRINTF_LINE_CHAR_LENGTH*PRINTF_LINE_COUNT,
                   BAGL_ENCODING_LATIN1);
  }
  */

}

void screen_prints(const char* str, unsigned int charcount) {
  while(charcount--) {
    screen_printc(*str++);
  }
}


/**
 * Common printf code, relies on 2 FAL:
 * - screen_prints
 * - screen_printc
 */

static const char * const g_pcHex = "0123456789abcdef";
static const char * const g_pcHex_cap = "0123456789ABCDEF";

void screen_printf(const char* format, ...) {
    /* dummy version
    unsigned short i;
    unsigned short len = strlen(str);
    for(i=0;i<len;i++){

        screen_printc(str[i]);
    }
    */

    unsigned long ulIdx, ulValue, ulPos, ulCount, ulBase, ulNeg, ulStrlen, ulCap;
    char *pcStr, pcBuf[16], cFill;
    va_list vaArgP;
    char cStrlenSet;

    //
    // Check the arguments.
    //
    if(format == 0) {
      return;
    }

    //
    // Start the varargs processing.
    //
    va_start(vaArgP, format);

    //
    // Loop while there are more characters in the string.
    //
    while(*format)
    {
        //
        // Find the first non-% character, or the end of the string.
        //
        for(ulIdx = 0; (format[ulIdx] != '%') && (format[ulIdx] != '\0');
            ulIdx++)
        {
        }

        //
        // Write this portion of the string.
        //
        screen_prints(format, ulIdx);

        //
        // Skip the portion of the string that was written.
        //
        format += ulIdx;

        //
        // See if the next character is a %.
        //
        if(*format == '%')
        {
            //
            // Skip the %.
            //
            format++;

            //
            // Set the digit count to zero, and the fill character to space
            // (i.e. to the defaults).
            //
            ulCount = 0;
            cFill = ' ';
            ulStrlen = 0;
            cStrlenSet = 0;
            ulCap = 0;
            ulBase = 10;

            //
            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
            //
again:

            //
            // Determine how to handle the next character.
            //
            switch(*format++)
            {
                //
                // Handle the digit characters.
                //
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    //
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    //
                    if((format[-1] == '0') && (ulCount == 0))
                    {
                        cFill = '0';
                    }

                    //
                    // Update the digit count.
                    //
                    ulCount *= 10;
                    ulCount += format[-1] - '0';

                    //
                    // Get the next character.
                    //
                    goto again;
                }

                //
                // Handle the %c command.
                //
                case 'c':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Print out the character.
                    //
                    screen_prints((char *)&ulValue, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %d command.
                //
                case 'd':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    //
                    if((long)ulValue < 0)
                    {
                        //
                        // Make the value positive.
                        //
                        ulValue = -(long)ulValue;

                        //
                        // Indicate that the value is negative.
                        //
                        ulNeg = 1;
                    }
                    else
                    {
                        //
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        //
                        ulNeg = 0;
                    }

                    //
                    // Set the base to 10.
                    //
                    ulBase = 10;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle ths %.*s command
                // special %.*H or %.*h format to print a given length of hex digits (case: H UPPER, h lower)
                //
                case '.':
                {
                  // ensure next char is '*' and next one is 's'
                  if (format[0] == '*' && (format[1] == 's' || format[1] == 'H' || format[1] == 'h')) {
                    
                    // skip '*' char
                    format++;
                    
                    ulStrlen = va_arg(vaArgP, unsigned long);
                    cStrlenSet = 1;
                    
                    // interpret next char (H/h/s)
                    goto again;
                  }
                  
                  // does not support %.2x for example
                  goto error;
                }
                
                case '*':
                {
                  if (*format == 's' ) {                    
                    
                    ulStrlen = va_arg(vaArgP, unsigned long);
                    cStrlenSet = 2;
                    goto again;
                  }
                  
                  goto error;
                }
                
                case '-': // -XXs
                {
                  cStrlenSet = 0;
                  // read a number of space to post pad with ' ' the string to display
                  goto again;
                }

                //
                // Handle the %s command.
                // %H and %h also
                case 'H':
                  ulCap = 1; // uppercase base 16
                  ulBase = 16;
                  goto case_s;
                case 'h':
                  ulBase = 16; // lowercase base 16
                  goto case_s;
                case 's':
                case_s:
                {
                    //
                    // Get the string pointer from the varargs.
                    //
                    pcStr = va_arg(vaArgP, char *);

                    //
                    // Determine the length of the string. (if not specified using .*)
                    //
                    switch(cStrlenSet) {
                      // compute length with strlen
                      case 0:
                        for(ulIdx = 0; pcStr[ulIdx] != '\0'; ulIdx++)
                        {
                        }
                        break;
                        
                      // use given length
                      case 1:
                        ulIdx = ulStrlen;
                        break;
                        
                      // printout prepad
                      case 2:
                        // if string is empty, then, ' ' padding
                        if (pcStr[0] == '\0') {
                        
                          // padd ulStrlen white space
                          do {
                            screen_prints(" ", 1);
                          } while(ulStrlen-- > 0);
                        
                          goto s_pad;
                        }
                        goto error; // unsupported if replicating the same string multiple times
                      case 3:
                        // skip '-' still buggy ...
                        goto again;
                    }

                    //
                    // Write the string.
                    //
                    switch(ulBase) {
                      default:
                        screen_prints(pcStr, ulIdx);
                        break;
                      case 16: {
                        unsigned char nibble1, nibble2;
                        for (ulCount = 0; ulCount < ulIdx; ulCount++) {
                          nibble1 = (pcStr[ulCount]>>4)&0xF;
                          nibble2 = pcStr[ulCount]&0xF;
                          switch(ulCap) {
                            case 0:
                              screen_printc(g_pcHex[nibble1]);
                              screen_printc(g_pcHex[nibble2]);
                              break;
                            case 1:
                              screen_printc(g_pcHex_cap[nibble1]);
                              screen_printc(g_pcHex_cap[nibble2]);
                              break;
                          }
                        }
                        break;
                      }
                    }

s_pad:
                    //
                    // Write any required padding spaces
                    //
                    if(ulCount > ulIdx)
                    {
                        ulCount -= ulIdx;
                        while(ulCount--)
                        {
                            screen_prints(" ", 1);
                        }
                    }
                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %u command.
                //
                case 'u':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // Set the base to 10.
                    //
                    ulBase = 10;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ulNeg = 0;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %x and %X commands.  Note that they are treated
                // identically; i.e. %X will use lower case letters for a-f
                // instead of the upper case letters is should use.  We also
                // alias %p to %x.
                //
                case 'X':
                    ulCap = 1;
                case 'x':
                case 'p':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // Set the base to 16.
                    //
                    ulBase = 16;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ulNeg = 0;

                    //
                    // Determine the number of digits in the string version of
                    // the value.
                    //
convert:
                    for(ulIdx = 1;
                        (((ulIdx * ulBase) <= ulValue) &&
                         (((ulIdx * ulBase) / ulBase) == ulIdx));
                        ulIdx *= ulBase, ulCount--)
                    {
                    }

                    //
                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    //
                    if(ulNeg)
                    {
                        ulCount--;
                    }

                    //
                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    //
                    if(ulNeg && (cFill == '0'))
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ulPos++] = '-';

                        //
                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        //
                        ulNeg = 0;
                    }

                    //
                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    //
                    if((ulCount > 1) && (ulCount < 16))
                    {
                        for(ulCount--; ulCount; ulCount--)
                        {
                            pcBuf[ulPos++] = cFill;
                        }
                    }

                    //
                    // If the value is negative, then place the minus sign
                    // before the number.
                    //
                    if(ulNeg)
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ulPos++] = '-';
                    }

                    //
                    // Convert the value into a string.
                    //
                    for(; ulIdx; ulIdx /= ulBase)
                    {
                        if (!ulCap) {
                          pcBuf[ulPos++] = g_pcHex[(ulValue / ulIdx) % ulBase];
                        }
                        else {
                          pcBuf[ulPos++] = g_pcHex_cap[(ulValue / ulIdx) % ulBase];
                        }
                    }

                    //
                    // Write the string.
                    //
                    screen_prints(pcBuf, ulPos);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %% command.
                //
                case '%':
                {
                    //
                    // Simply write a single %.
                    //
                    screen_prints(format - 1, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

error:
                //
                // Handle all other commands.
                //
                default:
                {
                    //
                    // Indicate an error.
                    //
                    screen_prints("ERROR", 5);

                    //
                    // This command has been handled.
                    //
                    break;
                }
            }
        }
    }

    //
    // End the varargs processing.
    //
    va_end(vaArgP);
}
#endif // HAVE_PRINTF



void screen_test(void) {
  // initialize screen session and charge pump
  screen_init();
  // modify screen "image"  
  screen_clear();
  
  /*
  screen_printf("Test\nString\n value ");
  screen_printf("azertyuiopqsdfghjklmwxcvbn,;:!?./????*%%??12345678901234567890)=??+~#{[|`\\^$@]}ABCDEFGHIJKLMNOPQRSTUVWXYZ\n%04d\nBOOT SCREEN TEST\n",2763);
  
  screen_printf("%.*H\n", 8, g_pcHex);
  */
  // force refresh and close screen session
  screen_printf("Ledger BLE/NFC $itcoin Wallet\n");
  screen_update();
  
  /*
  screen_printf("lastline\n");
  
  screen_update();
  */
}



#endif // HAVE_GLO091 
