/*******************************************************************************
*   Ledger Nano S - Non secure firmware
*   (c) 2016 Ledger
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
********************************************************************************/


/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx.h"

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h" 

#include <stdio.h>

#include "bagl.h"

// for N_bootsector variable
#include "bootsector.h"
// for USB state
#include "seproxyhal.h"

#include "nvm.h"


#define backlight_enable(x) 
#define backlight_is_enabled(x) 1

// not used for now
#define THROW(x) for(;;);

#define SYSTICK_MS 8

#define UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES 25

#define ASYNCH_DISPLAY_PROCESSED_EVENT

//#define DEMO_LOGO

//#define TEST_KEYBOARD

const uint32_t CLOCK_SECOND = 1000/SYSTICK_MS;

unsigned int frequency_hz;

volatile unsigned char G_io_apdu_protocol_enabled;
volatile unsigned char G_io_apdu_buffer[260];
volatile unsigned short G_io_apdu_length;

#ifdef HAVE_SE
USART_HandleTypeDef G_io_se_usart;

volatile unsigned char G_io_se_powered = 0;
extern unsigned char G_io_se_atr[40];
extern unsigned short G_io_se_atr_length;
#endif // HAVE_SE

volatile io_button_t G_io_button;

#ifdef DEBUG_BUTTON_ALWAYS_PUSHED
#define BUTTON_PRESS_DURATION_BOOT_POWER_OFF         50 // long enough to avoid people being troubled when trying to go bootloader
#define BUTTON_PRESS_DURATION_BOOT_CTRL_BOOTLOADER   20
#define BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY       10
#define BUTTON_PRESS_DURATION_BOOT_POWER_ON          10
#define BUTTON_PRESS_DURATION_POWER_OFF            1000
#ifdef DEBUG_BUTTON_FLASHBACK
#error FLASHBACK not supported with ALWAYS PUSHED
#endif // DEBUG_BUTTON_FLASHBACK
#else // DEBUG_BUTTON_ALWAYS_PUSHED
#define BUTTON_PRESS_DURATION_BOOT_POWER_OFF       10000 // long enough to avoid people being troubled when trying to go bootloader
#define BUTTON_PRESS_DURATION_BOOT_CTRL_BOOTLOADER  3000
#define BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY      3000

// the L4 button when pressed a long time switch to a verbose mode
#ifdef DEBUG_BUTTON_LINK_DEBUG
#define BUTTON_PRESS_DURATION_LINK_DEBUG           (BUTTON_PRESS_DURATION_POWER_OFF-2000)
#endif //DEBUG_BUTTON_LINK_DEBUG

#endif // DEBUG_BUTTON_ALWAYS_PUSHED

extern unsigned char _signed;
extern unsigned char _esigned;
extern unsigned char _esignature;

// dummy in case no screen HW declared
__weak void screen_init(unsigned char reinit) {}
__weak void screen_clear(void) {}
__weak void screen_poweroff(void) {}
__weak void screen_update(void) {}
__weak void screen_printf(const char* format,...) {}
__weak void screen_xy(unsigned short x, unsigned short y, unsigned short rotation) {}

#define SEPROXYHAL_EVENT_TIMEOUT_MS 10000
unsigned int G_seproxyhal_event_timeout_enable;
unsigned int G_seproxyhal_event_timeout;
unsigned char G_seproxyhal_event_timeout_header[3];

/*
unsigned int backlight_level;
unsigned int backlight_is_enabled(void) {
  #ifdef BACKLIGHT_AUTOOFF_MS
  return G_backlight_autooff_ms < BACKLIGHT_AUTOOFF_MS;
  #else // if no auto off, the backlight is always on
  return 1;
  #endif //
}
*/


void harakiri(void) {

  __asm volatile("cpsid i");

  // + reset in case protoboard
  for (;;) {
    NVIC_SystemReset();
  }
}

/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

void SYSTICK_power(unsigned char powered) {
  if (powered) {
    /*Configure the SysTick to have interrupt in 1ms time basis*/
    // SYSTICK_MS/1000 = 1/(frequency_hz/systickreload)
    // systickreload = (SYSTICK_MS/1000)*(frequency_hz)
    /*
#if ((frequency_hz/1000)*SYSTICK_MS) <= 0 || ((frequency_hz/1000)*SYSTICK_MS) > (1<<24)
#error invalid ticking interval
#endif 
    */
/*
    volatile unsigned int v = frequency_hz;
    v /= 1000;
    v *= SYSTICK_MS;
    HAL_SYSTICK_Config(v);
    */
    HAL_SYSTICK_Config(((frequency_hz/1000UL)*SYSTICK_MS));
  }  
  else {
    SysTick->CTRL = 0;
  }
}

/* ==========================================================================================
 * ==========================================================================================
 * ========================================================================================== 
 */

/**
* @brief This function handles USB OTG FS global interrupt.
*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
void USB_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/**
  * @brief  SysTick_Handler This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern __IO uint32_t uwTick;
void SysTick_Handler(void)
{
  // increment the systick of the number of milliseconds elapsed since previous tick
  // NOTE: HAX of Hal_IncTick, to support all HAl_Delay callers (shall be avoided in the end)
  // TODO: remove me to detect HAL_Delay loops
  uwTick+=SYSTICK_MS;
  
#ifdef HAVE_SEPROXYHAL
  // update the ticker event
  if (G_io_seproxyhal_ticker_enabled) {
    G_io_seproxyhal_ticker_current_ms += SYSTICK_MS;
    if (G_io_seproxyhal_ticker_current_ms >= G_io_seproxyhal_ticker_interval_ms) {
      G_io_seproxyhal_events |= SEPROXYHAL_EVENT_TICKER;
      G_io_seproxyhal_ticker_current_ms = 0;
    }
  }

  if (G_seproxyhal_event_timeout_enable) {
    G_seproxyhal_event_timeout += SYSTICK_MS;
    #ifdef DEBUG_BUTTON_LINK_DEBUG
    if (G_io_button.link_debug && G_seproxyhal_event_timeout > SEPROXYHAL_EVENT_TIMEOUT_MS && G_seproxyhal_event_timeout_enable == 1) {
      G_seproxyhal_event_timeout_enable = 2;
      screen_printf("Timeout waiting status for event: %.*H...\n", 3, G_seproxyhal_event_timeout_header);
    }
    #endif // DEBUG_BUTTON_LINK_DEBUG
  }
#endif // HAVE_SEPROXYHAL


  if (G_io_button.pressed) {
    // compute total press duration
    G_io_button.duration_ms += SYSTICK_MS;
  }
}


/* Private function prototypes -----------------------------------------------*/
void User_Process(void);

void clock_low(void) {
  //RCC->ICSCR = ((RCC->ICSCR) & 0xFFFF1FFF) | 0x00000000;
}

void clock_high(void) {

  // MSI
  //RCC->ICSCR = ((RCC->ICSCR) & 0xFFFF1FFF) | (MSIRANGE<<13);
  // HSI
}



/**
 * @brief  Main function to show how to use the BlueNRG Bluetooth Low Energy 
 *          shield to exchange data between two Nucleo baords with their
 *          respective BlueNRG shields.
 *          One board will act as Server-Peripheral and the other as 
 *          Client-Central.
 *          After connection has been established, by pressing the USER button
 *          on one board, the LD2 LED on the other one gets toggled and
 *          viceversa.
 *          The communication is done using a vendor specific profile.
 * @param  None
 * @retval None
 */

USBD_HandleTypeDef USBD_Device;
PCD_HandleTypeDef hpcd_USB_OTG_FS;


void clock_config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);


  // compute systick clock at once
  frequency_hz = 48000000;

}


#ifdef HAVE_BL
#define CLA 0xE0

#define INS_SECUINS 0
#define INS_GET_VERSION 0x01
#define INS_RESET 2

#define INS_VALIDATE_TARGET_ID 4

// entirely replied (length of Lc)
#define INS_ECHO 0xFF

// graphic debug for NBI
#define INS_DRAW_BMP4BPP          0x40
#define INS_DRAW_BMP4BPP_CONTINUE 0x41

// 0xE0 0xFE 0x00 0x00 0x06 <U4BE(@)> <U2BE(len)>
#define INS_DUMP 0xFE
#define INS_MCU_CMD 0xFD

#define SECUREINS_SELECT_SEGMENT 5
#define SECUREINS_LOAD 6
#define SECUREINS_FLUSH 7
#define SECUREINS_CRC 8
// start at given address (app main)
#define SECUREINS_BOOT 9

// within the data field
#define APDU_OFF_CLA 0
#define APDU_OFF_INS 1
#define APDU_OFF_LC 4  
#define APDU_OFF_SECINS 5

#define STATE_ID     0xFF00
#define STATE_UNAUTH 0xEE11
#define STATE_AUTH   0xDD22 

#define os_memcpy memcpy
#define os_memset memset


// --------------------------------------------------------------------------
// -
// --------------------------------------------------------------------------
#ifdef FAST_CRC
static unsigned short const cx_ccitt[] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067, 
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};
#endif // FAST_CRC

// --------------------------------------------------------------------------
// -
// --------------------------------------------------------------------------

unsigned short cx_crc16_update(unsigned short crc, void const *buf, int len) {
  int i;
  unsigned char b;

  for (i = 0; i<len; i++) {
    b = ((unsigned char const *)buf)[i];
#ifdef FAST_CRC
    b = b ^ (crc >> 8);
    crc = cx_ccitt[b] ^ (crc << 8);
#else
    crc ^= b << 8;

    unsigned char bit;
    for (bit = 8; bit > 0; --bit) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc = (crc << 1);
    }
#endif
  }
  return crc;
}


unsigned short  cx_crc16(void const *buf, int len) {
  return cx_crc16_update(0xFFFF, buf, len);
}

void bootloader_erase_appconf(void) {
  // erase the whole page
  nvm_write(&N_bootsector, NULL, NVM_PAGE_SIZE_B);
  // ensure flushed to nvram
  nvm_write_flush();
}

extern unsigned int _text;
extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;

void bootloader_apdu_interp(void) {
  /*volatile*/ unsigned short rx = 0;
  /*volatile*/ unsigned short tx = 0;
  //volatile unsigned char iv[8];
  /*volatile*/ unsigned char flags;
  /*volatile*/ unsigned int load_address;
  /*volatile*/ unsigned short state = STATE_ID;
  // reset variables to avoid subtil attack after returning from the loaded app
  rx = 0;
  tx = 0;
  state = STATE_ID;
  load_address = 0;
  flags = 0;
  //memset(iv, 0, sizeof(iv));

  io_usb_hid_init();

  // DESIGN NOTE: the bootloader ignores the way APDU are fetched. The only goal is to retrieve APDU.
  // When APDU are to be fetched from multiple IOs, like NFC+USB+BLE, make sure the io_event is called with a 
  // switch event, before the apdu is replied to the bootloader. This avoid APDU injection faults.
        
  for (;;) {

    unsigned short sw;

    // process screen touch
    rx = tx;
    while ((rx = io_exchange(CHANNEL_APDU|flags, rx)) == 0 ) {
      // print loader ?
    }

    tx = 0; // ensure no race in catch_other if io_exchange throws an error

    // no apdu received, well, reset the session, and reset the bootloader configuration
    if (rx == 0) {
      sw = 0x6982; goto error;
    }

    if (G_io_apdu_buffer[APDU_OFF_CLA] != CLA) {
      sw = 0x6E00; goto error;
    }

    // unauthenticated instruction
    switch (G_io_apdu_buffer[APDU_OFF_INS]) {
      case INS_GET_VERSION:
        tx = sizeof(VERSION);
        G_io_apdu_buffer[0] = CONFIG_TARGET_ID>>24;
        G_io_apdu_buffer[1] = CONFIG_TARGET_ID>>16;
        G_io_apdu_buffer[2] = CONFIG_TARGET_ID>>8;
        G_io_apdu_buffer[3] = CONFIG_TARGET_ID;
        G_io_apdu_buffer[4] = sizeof(VERSION);
        memcpy(G_io_apdu_buffer + 5, VERSION, sizeof(VERSION));
        sw = 0x9000; goto error;

      case INS_VALIDATE_TARGET_ID:
        if (U4BE(G_io_apdu_buffer, 5) == CONFIG_TARGET_ID) {
          state = STATE_UNAUTH;
          sw = 0x9000; goto error;
        }
        else {
          // id not valid
          state = STATE_ID;
          sw = 0x6A84; goto error;
        }
        break;

#ifdef HAVE_DRAW
      case INS_DRAW_BMP4BPP: {
        unsigned short x = U2BE(G_io_apdu_buffer, 5);
        unsigned short y = U2BE(G_io_apdu_buffer, 7);
        unsigned short width = U2BE(G_io_apdu_buffer, 9);
        unsigned short height = U2BE(G_io_apdu_buffer, 11);
        unsigned short colors_count = G_io_apdu_buffer[13];
        // colors are LE encoded
        //unsigned bit_per_pixel = 4;
        unsigned bit_per_pixel = 1;
        unsigned int colors[2];
        colors [0] = U4BE(G_io_apdu_buffer, 14);
        colors [1] = U4BE(G_io_apdu_buffer, 14+4);
        bagl_hal_draw_bitmap_within_rect(x, 
                                         y, 
                                         width, 
                                         height, 
                                         colors_count, 
                                         colors,
                                         bit_per_pixel, 
                                         G_io_apdu_buffer+14+colors_count*4, 
                                         (rx - 5 - 9 - colors_count*4)*8);
        screen_update();
        sw = 0x9000; goto error;
        break;
      }

      case INS_DRAW_BMP4BPP_CONTINUE: {
        unsigned bit_per_pixel = 1;
        bagl_hal_draw_bitmap_continue(bit_per_pixel, G_io_apdu_buffer+5, (rx-5)*8);
        screen_update();
        break;
      }
#endif // HAVE_DRAW

      case INS_RESET:
        flags |= IO_RESET_AFTER_REPLIED;
        sw = 0x9000; goto error;
        break;

      // proceed to the secure instruction interpreter
      case INS_SECUINS:
        // retrieve data (all raw loader commands are case 2)
        rx = G_io_apdu_buffer[APDU_OFF_LC];

        // loader ID not yet presented
        if (state == STATE_ID) {
          sw = 0x6985 ; goto error;
        }

// SECURITY DEACTIVATED UNTIL CRYPTO IS READY
        
        switch(G_io_apdu_buffer[APDU_OFF_SECINS]) {
          // all data are ciphered with the issuer key, no data is trusted from the
          // SCP as the customer key could be compromised
          case SECUREINS_SELECT_SEGMENT: {
            if (rx != 1+4) {
              sw = 0x6700; goto error;
            }

            load_address = U4BE(G_io_apdu_buffer, 6);
            // nothing to output
            break; 
          }
          case SECUREINS_LOAD: {
            // at least offset (2 bytes) + 1 byte to patch + 1 byte instruction
            if (rx < 1+2+1) {
              sw = 0x6700; goto error;
            }


            unsigned int chunk_address = ((unsigned int)load_address + (unsigned int)U2BE(G_io_apdu_buffer, 6));

            // feil the session, won't accept further command
            if (chunk_address <= &_etext) {
              sw = 0x6984; goto error;
            }

            // avoid loading when a code is ready for booting
            if (N_bootsector.main_magic == MAIN_MAGIC) {
              bootloader_erase_appconf();
            }

            // protect against bootloader self modification
            // align to the next nvm page (could leave a blank page in between if already aligned)
            if (chunk_address >= (unsigned int)&_text && chunk_address < ((unsigned int)&_etext + (unsigned int)&_edata - (unsigned int)&_data)&(~(NVM_PAGE_SIZE_B-1)) + NVM_PAGE_SIZE_B ) {
              sw = 0x6985; goto error;
            }

            nvm_write((unsigned char const *)chunk_address, G_io_apdu_buffer+8, rx-3);
            break; 
          }
          case SECUREINS_FLUSH: {
            nvm_write_flush();
            break;
          }
          case SECUREINS_CRC: {
            if (rx != 1+2+4+2) {
              sw = 0x6700; goto error;
            }
            // check crc, OR security error
            if (cx_crc16((unsigned char const*)((unsigned int)load_address + (unsigned int)U2BE(G_io_apdu_buffer, 6)), U4BE(G_io_apdu_buffer, 8)) != U2BE(G_io_apdu_buffer, 12)) {
              sw = 0x6982; goto error;
            }
            break;
          }

          case SECUREINS_BOOT: {
            if (rx != 1+4) {
              sw = 0x6700; goto error;
            }

            union {
             bootsector_nvram_t ramconf;
            } bootpage;

            memset(&bootpage, 0, sizeof(bootpage));
            bootpage.ramconf.main = (loadedmain_t)U4BE(G_io_apdu_buffer, 6);
            bootpage.ramconf.main_magic = MAIN_MAGIC;
            nvm_write(&N_bootsector, &bootpage, sizeof(bootpage));
            // ensure flushing the boot configuration into nvram
            nvm_write_flush();

            // from now on, the application can boot, boot now
            flags |= IO_RESET_AFTER_REPLIED;

            ///BEGIN WIPE STATE
            // invalidate the IV, make sure nothing can be validated after that
            //os_memset(&(iv[0]), 0xFF, sizeof(iv));
            state = STATE_ID;
            ///END WIPE STATE
            break;
          }
          default:
            sw = 0x6D00; goto error;
            break;
        }
        // ok no problem
        sw = 0x9000; goto error;
        break;

      // ensure INS is empty otherwise (use secure instruction)
      default:
        sw = 0x6D00; goto error;
    }

  error:
    // Unexpected exception => security erase
    G_io_apdu_buffer[tx] = sw>>8;
    G_io_apdu_buffer[tx+1] = sw;
    tx += 2;
    // error detected !!
    if (sw != 0x9000) {
      ///BEGIN WIPE STATE
      // invalidate the IV, make sure nothing can be validated after that
      //os_memset(&(iv[0]), 0xFF, sizeof(iv));
      state = STATE_ID;
      /// END WIPE STATE
    }
  }
}

void bootloader_delegate_boot(uint32_t button_press_duration) {

  // if boot address is valid, then boot it
  if (N_bootsector.main_magic == MAIN_MAGIC) {

    // disable interruption before changing VTOR and delegate to application code
    __asm("cpsid i");

    // jump into the application code
    N_bootsector.main(button_press_duration);

    __asm("cpsid i");

    // jump failed (or loaded code returned) go to bootloader
    nvm_write_init();
    bootloader_erase_appconf();

    // ensure to reset the display of the bootloader prompt
    NVIC_SystemReset();
  }
}
#endif // HAVE_BL

void SystemClock_Config(void) {
  // TODO re apply the best clock scheme according to the current power mode
}

#ifdef HAVE_SCREEN


// Little Endian, Column, not packed, 
unsigned char const C_bitmap_logo[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcc, 
0xce, 0xcf, 0xcf, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfc, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x73, 
0xf3, 0xf3, 0x00, 0x00, 0xf3, 0xf3, 0xf3, 0xf3, 0x03, 0x03, 0xf3, 0xf3, 0x73, 0x33, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char C_bitmap_bootloader[]= {
0xc0, 0xf0, 0xf8, 0xfc, 0xbc, 0x3e, 0x06, 0x06, 0x3e, 0xbc, 0xfc, 0xf8, 0xf0, 0xc0, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x48, 0x48, 0x48, 0xb8, 0x00, 0x00, 0xc0, 0x20, 0x20, 
0x20, 0xc0, 0x00, 0x00, 0xc0, 0x20, 0x20, 0x20, 0xc0, 0x00, 0x20, 0xf0, 0x20, 0x20, 0x00, 0xf8, 
0x00, 0x00, 0xc0, 0x20, 0x20, 0x20, 0xc0, 0x00, 0x00, 0xa0, 0xa0, 0xa0, 0xe0, 0x00, 0x00, 0xc0, 
0x20, 0x20, 0x20, 0xf8, 0x00, 0x00, 0xc0, 0xa0, 0xa0, 0xe0, 0x00, 0x00, 0xe0, 0x20, 0x20, 0x03, 
0x0f, 0x1f, 0x3f, 0x33, 0x73, 0x72, 0x72, 0x73, 0x33, 0x3f, 0x1f, 0x0f, 0x03, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x04, 0x04, 0x04, 0x07, 0x00, 0x00, 0x03, 0x04, 0x04, 0x04, 
0x03, 0x00, 0x00, 0x03, 0x04, 0x04, 0x04, 0x03, 0x00, 0x00, 0x07, 0x04, 0x04, 0x00, 0x07, 0x00, 
0x00, 0x03, 0x04, 0x04, 0x04, 0x03, 0x00, 0x00, 0x07, 0x04, 0x04, 0x07, 0x00, 0x00, 0x03, 0x04, 
0x04, 0x04, 0x07, 0x00, 0x00, 0x03, 0x04, 0x04, 0x04, 0x00, 0x00, 0x07, 0x00, 0x00,
};

const unsigned char C_bitmap_se_recovery[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf0, 0xf8, 0xfc, 0x9c, 0x0e, 0x06, 0x9e, 0x1e, 0x3c, 0x7c, 
0xf8, 0xf0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x88, 0x88, 0x88, 0x70, 
0x00, 0x00, 0xc0, 0xa0, 0xa0, 0xe0, 0x00, 0x00, 0xc0, 0x20, 0x20, 0x20, 0x00, 0xc0, 0x20, 0x20, 
0x20, 0xc0, 0x00, 0x20, 0xc0, 0x00, 0x00, 0xc0, 0x20, 0x00, 0xc0, 0xa0, 0xa0, 0xe0, 0x00, 0x00, 
0xe0, 0x20, 0x20, 0x20, 0xc0, 0x00, 0x00, 0xc0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x03, 0x0f, 0x1f, 0x3f, 0x3f, 0x7f, 0x7e, 0x7f, 0x73, 0x30, 0x38, 0x1f, 
0x0f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x01, 0x06, 0x00, 
0x00, 0x03, 0x04, 0x04, 0x04, 0x00, 0x00, 0x03, 0x04, 0x04, 0x04, 0x00, 0x03, 0x04, 0x04, 0x04, 
0x03, 0x00, 0x00, 0x01, 0x06, 0x06, 0x01, 0x00, 0x00, 0x03, 0x04, 0x04, 0x04, 0x00, 0x00, 0x07, 
0x00, 0x00, 0x20, 0x21, 0x1e, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

extern unsigned char screen_framebuffer[];
void display_ctrl_mode(unsigned int mode) {
  const unsigned char* bmp;
  unsigned int i;
  if (mode == MODE_BOOT) {
    bmp = C_bitmap_logo;
    goto update_bitmap;
  }
  if (mode & MODE_MCU_BOOTLOADER) {
    bmp = C_bitmap_bootloader;
    goto update_bitmap;
  }
  else if (mode & MODE_SE_RECOVERY) {
    bmp = C_bitmap_se_recovery;
    goto update_bitmap;
  }
  /*
  else if (mode & MODE_POWER_ON) {
    bmp = bitmap_pwron;
    goto update_bitmap;
  }
  */
  return;

update_bitmap:
  // half first part
  memcpy(screen_framebuffer+(128*32/4 + 128*32/4/2 - 320)/8, bmp, sizeof(C_bitmap_logo)/2);
  // half second part
  memcpy(screen_framebuffer+(2*128*32/4 + 128*32/4/2 - 320)/8, bmp + sizeof(C_bitmap_logo)/2, sizeof(C_bitmap_logo)/2);

  screen_update();
}
#endif // HAVE_SCREEN

extern unsigned int isr;

extern unsigned int _etext;
extern unsigned int  _data;
extern unsigned int  _edata;
extern unsigned int  _bss;
extern unsigned int  _ebss;
extern unsigned int  _estack;

void main(unsigned int button_press_duration)
{ 
  GPIO_InitTypeDef GPIO_InitStruct;

  G_io_button.displayed_mode = 0;
  G_io_button.pressed = 0;
  G_io_button.duration_ms = 0; 

  #ifdef DEBUG_BUTTON_LINK_DEBUG
  G_io_button.link_debug = 0;
  #endif // DEBUG_BUTTON_LINK_DEBUG

  #ifndef ARM_CORE_HAS_VTOR
  G_bootsector.VTOR = &isr;
  #else
  SCB->VTOR = &isr;
  #endif //! ARM_CORE_HAS_VTOR

  nvm_write_init();

  __PWR_CLK_ENABLE();
  __SYSCFG_CLK_ENABLE(); 

  __GPIOA_CLK_ENABLE();
  // backlight full off during screen init to avoid blink
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);

  __GPIOC_CLK_ENABLE();
  /* PC9 AUX_PWR_enable, PC8 button_enable */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  // keep power on
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0); 

  clock_config();


  // low powering the whole chip by disabling all possible boot pull resistors
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9/*|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14*/ /*|GPIO_PIN_15*/;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|/*GPIO_PIN_8|GPIO_PIN_9|*/GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  clock_high();

  // sort out interrupt priority to avoid complete lockup
  HAL_NVIC_SetPriority(USB_IRQn, 3, 0);
  //HAL_NVIC_SetPriority(PendSV_IRQn, 3, 0);
  //HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  /*
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
  //HAL_NVIC_SetPriority(USART1_IRQn, 0, 0); // avoid overrun error on the SE iso link when receiving
  

  // start the ticker for the tsc to work correctly (warning ARM hal set the systick prio !!)
  SYSTICK_power(1);
  __asm("cpsie i");

  // initialize screen session and charge pump, but no disoplay yet
  screen_init(0);
  screen_brightness(20); // same setting as in Bolos
  
  display_ctrl_mode(MODE_BOOT);

  // check if both buttons are pressed to start in bootloader mode
  // set pullup, connected to ground if pressed
  __GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_15; // on the USB side
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  __GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_7; // on the hole in the mechanic side (SE side)
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  G_io_button.duration_ms = 0; // approx
  G_io_button.displayed_mode = MODE_BOOT;

  G_io_button.pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)?0:1) | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)?0:2); // fake pressed (booting means pressed)
  goto update_pressed;
  while (G_io_button.pressed != 0) {


    if ((G_io_button.pressed == 1) && G_io_button.duration_ms >= BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY && !(G_io_button.displayed_mode & MODE_SE_RECOVERY) ) {
      // display an icon on screen to notify.
      display_ctrl_mode(MODE_SE_RECOVERY);
      G_io_button.displayed_mode |= MODE_SE_RECOVERY;
    }

    if ((G_io_button.pressed == 2) && G_io_button.duration_ms >= BUTTON_PRESS_DURATION_BOOT_CTRL_BOOTLOADER && !(G_io_button.displayed_mode & MODE_MCU_BOOTLOADER) ) {
      // display an icon on screen to notify.
      display_ctrl_mode(MODE_MCU_BOOTLOADER);
      G_io_button.displayed_mode |= MODE_MCU_BOOTLOADER;
    }

    /*
    // too long, sorry
    if (G_io_button.pressed != 0 && G_io_button.duration_ms >= BUTTON_PRESS_DURATION_BOOT_POWER_OFF && !(G_io_button.displayed_mode & MODE_POWER_OFF) ) {
      //pre_harakiri();
      G_io_button.displayed_mode |= MODE_POWER_OFF;
    }
    */

  update_pressed:
    G_io_button.pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)?0:1) | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)?0:2); // fake pressed (booting means pressed)
  }

  /* Disabled, as the nano always startup, only the mode change depending on the button press
  // power off if not pushed long enough,
  if (
   //allow boot in usb without button pressed
  // !usb && 
   !(G_io_button.displayed_mode & MODE_POWER_ON)) {
    harakiri();
  }
  */

  G_io_button.pressed = 0;
  // if any button is not pressed, then boot the code if any currently loaded
  if (! (G_io_button.displayed_mode & MODE_MCU_BOOTLOADER)) {
    bootloader_delegate_boot(G_io_button.duration_ms);
  }

  // let the boot logo for a while
  HAL_Delay(500);

  display_ctrl_mode(MODE_MCU_BOOTLOADER);


  G_io_apdu_protocol_enabled = 1;

  /* Init Device Library */
  USBD_Init(&USBD_Device, &HID_Desc, 0);
  
  /* Register the HID class */
  USBD_RegisterClass(&USBD_Device, &USBD_CUSTOM_HID);

  USBD_CUSTOM_HID_RegisterInterface(&USBD_Device, &USBD_CustomHID_template_fops);
  
  /* Start Device Process */
  USBD_Start(&USBD_Device);

  // run the bootloader main
  bootloader_apdu_interp();

  // shall never be reached
  NVIC_SystemReset();
}

/**
 * @}
 */

/**
 * @}
 */

