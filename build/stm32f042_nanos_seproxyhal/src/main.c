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
#include "seproxyhal.h"

#define backlight_enable(x) 
#define backlight_is_enabled(x) 1

// not used for now
#define THROW(x) for(;;);

#define SYSTICK_MS 8

#define UI_KEYBOARD_BLINK_ON_TOUCH_CYCLES 25

#ifdef HAVE_REVERSED_BUTTON
  #define PB7_BUTTON_IDX 1
  #define PA15_BUTTON_IDX 0
#else
  #define PB7_BUTTON_IDX 0
  #define PA15_BUTTON_IDX 1
#endif

//#define ASYNCH_DISPLAY_PROCESSED_EVENT

//#define DEMO_LOGO

//#define TEST_KEYBOARD

const uint32_t CLOCK_SECOND = 1000/SYSTICK_MS;

unsigned int frequency_hz;

SPI_HandleTypeDef hspi;
volatile unsigned char G_io_apdu_protocol_enabled;
volatile unsigned short G_io_apdu_length;
USART_HandleTypeDef G_io_se_usart;

volatile unsigned char G_io_se_powered = 0;
volatile unsigned char G_io_apdu_buffer[260];
extern unsigned char G_io_se_atr[40];
extern unsigned short G_io_se_atr_length;

volatile unsigned int G_io_seproxyhal_ticker_interval_ms;
volatile unsigned int G_io_seproxyhal_ticker_current_ms;
volatile unsigned int G_io_seproxyhal_ticker_enabled;

volatile io_button_t G_io_button;

#define BUTTON_PRESS_DURATION_BOOT_POWER_OFF       10000 // long enough to avoid people being troubled when trying to go bootloader
#define BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY     3000
#define BUTTON_PRESS_DURATION_POWER_OFF            10000

#ifdef HAVE_TOUCHPANEL
volatile struct touch_state_s G_io_touch;
#endif //HAVE_TOUCHPANEL
volatile struct usb_state_s G_io_usb;
volatile int G_io_seproxyhal_state;
volatile unsigned int G_io_seproxyhal_events;
volatile unsigned char G_io_seproxyhal_buffer[IO_SEPROXYHAL_BUFFER_SIZE_B];

volatile io_unsec_chunk_t G_io_unsec_chunk;

#define MAX_BAGL_ANIM 1
unsigned char bagl_animated_text[MAX_BAGL_ANIM][128]; // up to 128 char of text in a single animated label
bagl_animated_t bagl_animated[MAX_BAGL_ANIM];

// dummy in case no screen HW declared
__weak void screen_init(unsigned char reinit) {}
__weak void screen_clear(void) {}
__weak void screen_poweroff(void) {}
__weak void screen_update(void) {}
__weak void screen_printf(const char* format,...) {}
__weak void screen_xy(unsigned short x, unsigned short y, unsigned short rotation) {}

const char ESC_DEVICE_NAME[] = "$DEVICENAME";
const char DEVICE_NAME_NA[] = "Device Name not available";

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
  
  // update the ticker event
  if (G_io_seproxyhal_ticker_enabled) {
    G_io_seproxyhal_ticker_current_ms += SYSTICK_MS;
    if (G_io_seproxyhal_ticker_current_ms >= G_io_seproxyhal_ticker_interval_ms) {
      __asm volatile("cpsid i");
      G_io_seproxyhal_events |= SEPROXYHAL_EVENT_TICKER;
      __asm volatile("cpsie i");
      G_io_seproxyhal_ticker_current_ms = 0;
    }
  }

  if (G_io_button.pressed) {
    // compute total press duration
    G_io_button.duration_ms += SYSTICK_MS;

    // prepare to generate a new button event
    G_io_button.next_event_ms += SYSTICK_MS;
    if (G_io_button.next_event_ms >= SEPROXYHAL_TAG_BUTTON_PUSH_INTERVAL_MS) {
      __asm volatile("cpsid i");
      G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BUTTON;
      __asm volatile("cpsie i");
      G_io_button.next_event_ms = 0;
    }
#ifdef DEBUG_BUTTON_LINK_DEBUG
    if (G_io_button.duration_ms > BUTTON_PRESS_DURATION_LINK_DEBUG && ! (G_io_button.displayed_mode & MODE_LINK_DEBUG)) {
      G_io_button.link_debug++;
      G_io_button.displayed_mode |= MODE_LINK_DEBUG;
      screen_printf("SE link debug level %d\n", G_io_button.link_debug);
    }
#endif // DEBUG_BUTTON_LINK_DEBUG

#ifdef BUTTON_POWER_OFF_LONG_PRESS
    if (G_io_button.duration_ms > BUTTON_PRESS_DURATION_POWER_OFF && ! (G_io_button.displayed_mode & MODE_BOOT) && ! (G_io_button.displayed_mode & MODE_POWER_OFF)) {
      G_io_button.displayed_mode |= MODE_POWER_OFF;
      PRINTF("Mode power off\n");
    }
#endif // BUTTON_POWER_OFF_LONG_PRESS

  }

  if (G_seproxyhal_event_timeout_enable) {
    G_seproxyhal_event_timeout += SYSTICK_MS;
    #ifdef DEBUG_BUTTON_LINK_DEBUG
    if (G_io_button.link_debug && G_seproxyhal_event_timeout > SEPROXYHAL_EVENT_TIMEOUT_MS && G_seproxyhal_event_timeout_enable == 1) {
      G_seproxyhal_event_timeout_enable = 2;
      screen_printf("Timeout waiting status for event: %.*H...\n", 3, G_seproxyhal_event_timeout_header);
    }
    #endif // DEBUG_BUTTON_LINK_DEBUG
    // very dirty, but very handy
    //bagl_draw_string(BAGL_FONT_OPEN_SANS_REGULAR_11px, 0xFFFFFF, 0x000000, 0, 0, 128, 15, "timeout !", 9, BAGL_ENCODING_LATIN1);
  }

  // trigger the PendingSV handler to process lower tasks
  //SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  //__asm volatile ("dsb"); 
}

/* too triggerish, play within the event loop instead
void PendSV_Handler(void) {
  bagl_animate(&bagl_animated[0], uwTick, SYSTICK_MS);
}
*/

void EXTI4_15_IRQHandler(void) {
  __asm volatile("cpsid i");
  if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_7)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
    if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) {
      G_io_button.duration_ms = 0;
      G_io_button.pressed |= G_io_button.mapping[PB7_BUTTON_IDX];
      G_io_button.next_event_ms = 0;
    }
    else {
      G_io_button.pressed &= ~G_io_button.mapping[PB7_BUTTON_IDX];
    }
  }

  if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_15)) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
    if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
      G_io_button.duration_ms = 0;
      G_io_button.pressed |= G_io_button.mapping[PA15_BUTTON_IDX];
      G_io_button.next_event_ms = 0;

    }
    else {
      G_io_button.pressed &= ~G_io_button.mapping[PA15_BUTTON_IDX];
    }
  }

  // a button event is to be generated
  G_io_seproxyhal_events |= SEPROXYHAL_EVENT_BUTTON;
  __asm volatile("cpsie i");
}

#include "bagl.h"


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


typedef struct bagl_element_e {
  bagl_component_t component;
  const char* text;
} bagl_element_t;



void display_l4_mode_touch(void) {
  // button has been removed
  /*
  if (G_io_touch.ts_last_x >= 80 
    && G_io_touch.ts_last_x <= 80+160
    && G_io_touch.ts_last_y >= 290
    && G_io_touch.ts_last_y <= 290+60) {
    harakiri();
  }
  */
}

#ifdef DEMO_MODE

// a graphic element is an element with defined text and actions depending on user touches
typedef struct bagl_element_e be_t;

typedef struct bagl_screen_flow_s bagl_screen_flow_t;
typedef struct bagl_screen_flow_s {
  be_t const * elements;  
  bagl_screen_flow_t const * btn_1_next_screen;
  bagl_screen_flow_t const * btn_2_next_screen;
  bagl_screen_flow_t const * btn_both_next_screen;
};

bagl_screen_flow_t *current_screen;

typedef unsigned int (*bagl_element_button_press_event_t) (unsigned int buttons_mask);
bagl_element_button_press_event_t btn_handler;


void demo_refresh_current_screen(void) {
  unsigned int i = 0;

  while (current_screen->elements[i].component.type != BAGL_NONE) {
    if (current_screen->elements[i].text != NULL) {
      unsigned char* text;
      text = current_screen->elements[i].text;
      bagl_draw_with_context(&current_screen->elements[i].component, text, strlen(text), BAGL_ENCODING_LATIN1);
    }
    else {
      bagl_draw(&current_screen->elements[i].component);
    }

    // proceed to next screen element
    i++;
  }

  screen_update();
}


extern const bagl_screen_flow_t const screen_boot;

const be_t screen_bitcoin_2_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,     0,  0,  128,   32, 0, 0, BAGL_FILL, 0, 0, 0, 0 }, NULL },
  {{BAGL_ICON                          , 0x00,    25,  9,   14,   14, 0, 0, BAGL_FILL, 1, 0, 0, BAGL_GLYPH_ICON_CHECK_BADGE }, NULL },
  {{BAGL_LABEL                         , 0x00,    50, 10,  128,   32, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_LEFT, 0 }, "Confirmed" }, 
  {0},
};

const bagl_screen_flow_t screen_bitcoin_2 = {
  screen_bitcoin_2_elements,  
  &screen_boot, 
  &screen_boot,
  &screen_boot,
};


const be_t screen_bitcoin_1_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,    0,  0,  128,  32, 0, 0, BAGL_FILL, 0, 0, 0, 0 }, NULL },
  //{{BAGL_RECTANGLE                     , 0x00,    0,  0,  128,  32, 0, 0, BAGL_FILL, 1, 0, 0, 0 }, NULL },
  {{BAGL_ICON                          , 0x00,    3, 12,    8,   8, 0, 0, BAGL_FILL, 1, 0, 0, BAGL_GLYPH_ICON_CROSS }, NULL },
  {{BAGL_LABEL                         , 0x00,    0,  3,  128,  16, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_CENTER, 0 }, "New transaction" },
  {{BAGL_LABEL                         , 0x00,    0, 17,  128,  16, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_EXTRABOLD_11px|BAGL_FONT_ALIGNMENT_CENTER, 0 }, "0.124 BTC" },
  {{BAGL_ICON                          , 0x00, 128-(3+8),  12,   8,   8, 0, 0, BAGL_FILL, 1, 0, 0, BAGL_GLYPH_ICON_CHECK }, NULL }, 
  {0},
};
const bagl_screen_flow_t screen_bitcoin_1 = {
  screen_bitcoin_1_elements,
  &screen_bitcoin_2, 
  &screen_bitcoin_2,
  &screen_bitcoin_2,
};

/*
const be_t screen_fido_4_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,    0,  0,  128,   32, 1, 0, 0, 1, 0, 0, 0 }, NULL },
  // toast
  {{BAGL_RECTANGLE                      , 0x00,  29,  3,   16,   26, 0, 5, BAGL_FILL, 0xFFFFFF, 0x000000, BAGL_FONT_OPEN_SANS_LIGHT_16px|BAGL_FONT_ALIGNMENT_CENTER|BAGL_FONT_ALIGNMENT_MIDDLE, 0   }, "5"},
  {0},
};
const bagl_screen_flow_t screen_fido_4 = {
  screen_fido_4_elements,
  &screen_boot, 
  &screen_boot,
  &screen_boot,
};

const be_t screen_fido_3_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,    0,  0,  128,   32, 0, 0, BAGL_FILL, 1, 0, 0, 0 }, NULL },
  {0},
};
const bagl_screen_flow_t screen_fido_3 = {
  screen_fido_3_elements,
  &screen_fido_4, 
  &screen_fido_4,
  &screen_fido_4,
};
*/

const be_t screen_fido_2_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,    0,  0,  128,   32, 0, 0, BAGL_FILL, 0, 0, 0, 0 }, NULL },
  {{BAGL_ICON                          , 0x00,   16,  9,   14,   14, 0, 0, BAGL_FILL, 1, 0, 0, BAGL_GLYPH_ICON_CHECK_BADGE }, NULL },
  {{BAGL_LABEL                         , 0x00,   40, 10,  128,   32, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_LEFT, 0 }, "Authenticated" }, 
  {0},
};
const bagl_screen_flow_t screen_fido_2 = {
  screen_fido_2_elements,
  &screen_boot, 
  &screen_boot,
  &screen_boot,
/*
  &screen_fido_3, 
  &screen_fido_3,
  &screen_fido_3,
*/
};


const be_t screen_fido_1_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,    0,  0,  128,  32, 0, 0, BAGL_FILL, 0, 0, 0, 0 }, NULL },
  {{BAGL_ICON                          , 0x00,    3, 12,    8,   8, 0, 0, BAGL_FILL, 1, 0, 0, BAGL_GLYPH_ICON_CROSS }, NULL },
  {{BAGL_LABEL                         , 0x00,    0,  3,  128,  16, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_CENTER, 0 }, "Authenticate ?" },
  {{BAGL_LABEL                         , 0x00,    0, 17,  128,  16, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_EXTRABOLD_11px|BAGL_FONT_ALIGNMENT_CENTER, 0 }, "Google" },
  {{BAGL_ICON                          , 0x00, 128-(3+8),  12,   8,   8, 0, 0, BAGL_FILL, 1, 0, 0, BAGL_GLYPH_ICON_CHECK }, NULL }, 
  {0},
};
const bagl_screen_flow_t screen_fido_1 = {
  screen_fido_1_elements,
  &screen_fido_2, 
  &screen_fido_2,
  &screen_fido_2,
};


const be_t screen_boot_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,    0,   0, 128,  32, 0, 0, BAGL_FILL, 0, 0, 0, 0 }, NULL },
  {{BAGL_LABEL                          , 0x00,   0,   0, 128,  32, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_LEFT|BAGL_FONT_ALIGNMENT_MIDDLE, 0 }, "BITCOIN", },
  {{BAGL_LABEL                          , 0x00,   0,   0, 128,  32, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_RIGHT|BAGL_FONT_ALIGNMENT_MIDDLE, 0 }, "FIDO", },
  {0},
};
const bagl_screen_flow_t screen_boot = {
  screen_boot_elements,
  //screen_fido_4_elements, 
  &screen_bitcoin_1,
  &screen_fido_1,
  &screen_boot,
};

unsigned int b;
unsigned int demo_btn_handler(unsigned int btn_mask) {
  switch(btn_mask) {
    case 1:
      current_screen = current_screen->btn_1_next_screen;
      break;
    case 2:
      current_screen = current_screen->btn_2_next_screen;
      break;
    case 3: // both buttons
      //current_screen = current_screen->btn_both_next_screen;
      b = (b+10)%100;
      screen_brightness(b);
      break;
  }
  demo_refresh_current_screen();
}

void demo_init(void) {
  b = 0;
  // display the first screen
  current_screen = &screen_boot;
  demo_refresh_current_screen();
}
#endif // DEMO_MODE

#ifdef PROBER_MODE

// a graphic element is an element with defined text and actions depending on user touches
typedef struct bagl_element_e be_t;

typedef struct bagl_screen_flow_s bagl_screen_flow_t;
typedef struct bagl_screen_flow_s {
  be_t const * elements;  
  bagl_screen_flow_t const * btn_1_next_screen;
  bagl_screen_flow_t const * btn_2_next_screen;
  bagl_screen_flow_t const * btn_both_next_screen;
};

bagl_screen_flow_t *current_screen;

typedef unsigned int (*bagl_element_button_press_event_t) (unsigned int buttons_mask);
bagl_element_button_press_event_t btn_handler;


void prober_refresh_current_screen(void) {
  unsigned int i = 0;

  while (current_screen->elements[i].component.type != BAGL_NONE) {
    if (current_screen->elements[i].text != NULL) {
      unsigned char* text;
      text = current_screen->elements[i].text;
      bagl_draw_with_context(&current_screen->elements[i].component, text, strlen(text), BAGL_ENCODING_LATIN1);
    }
    else {
      bagl_draw(&current_screen->elements[i].component);
    }

    // proceed to next screen element
    i++;
  }

  screen_update();
}

extern const bagl_screen_flow_t const screen_boot;

const be_t screen_empty_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,    0,  0,  128,  32, 0, 0, BAGL_FILL, 0, 0, 0, 0 }, NULL },
  {0},
};

const bagl_screen_flow_t screen_empty = {
  screen_empty_elements,
  &screen_boot,
  &screen_boot,
  &screen_boot,
};

const be_t screen_boot_elements[] = {
  {{BAGL_RECTANGLE                     , 0x00,    0,  0,  128,   32, 0, 0, BAGL_FILL, 1, 0, 0, 0 }, NULL },
  {0},
};

const bagl_screen_flow_t screen_boot = {
  screen_boot_elements,
  &screen_boot,
  &screen_boot,
  &screen_empty,
};

unsigned int b;
unsigned int prober_btn_handler(unsigned int btn_mask) {
  switch(btn_mask) {
    case 1:
      current_screen = current_screen->btn_1_next_screen;
      break;
    case 2:
      current_screen = current_screen->btn_2_next_screen;
      break;
    case 3: // both buttons
      current_screen = current_screen->btn_both_next_screen;
      /*
      b = (b+10)%100;
      screen_brightness(b);
      */
      break;
  }
  prober_refresh_current_screen();
}

void prober_init(void) {
  b = 0;
  // display the first screen
  current_screen = &screen_boot;
  prober_refresh_current_screen();
}
#endif // PROBER_MODE

extern unsigned int g_pfnVectors;

extern unsigned int _etext;
extern unsigned int  _data;
extern unsigned int  _edata;
extern unsigned int  _bss;
extern unsigned int  _ebss;
extern unsigned int  _estack;


#include "bootsector.h"

#define N_bootsector (*((bootsector_nvram_t*)BL_BS_NVRAM))

#ifdef BOOTLOADER_UPGRADE
const unsigned char const upgrade_array[]= {

  // sections:
  // U4BE:       address
  // U4BE:       length
  // length* U1: data

#include "upgrade_content.bytes"

  // END
  // U4 -1
  // U4 -1 
  0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF,
};
#endif // BOOTLOADER_UPGRADE

void main_setup_screen_rotation(unsigned char rotation) {
    // default is inversed
    switch(rotation) {
        // USB ON THE RIGHT
        case 180:
            screen_rotation(0);
            G_io_button.mapping[0] = 1;
            G_io_button.mapping[1] = 2;                
            break;
            
        default:
        // USB ON THE LEFT
        case 0:
            screen_rotation(180);
            G_io_button.mapping[0] = 2;
            G_io_button.mapping[1] = 1;
            break;
    }
}

void main(unsigned int button_press_duration)
{ 
  GPIO_InitTypeDef GPIO_InitStruct;

  // as set by the bootloader
  frequency_hz = 48000000;

  G_io_button.displayed_mode = 0;
  G_io_button.pressed = 0;
  G_io_button.duration_ms = 0; 

  #ifndef ARM_CORE_HAS_VTOR
  G_bootsector.VTOR = &g_pfnVectors;
  #else
  SCB->VTOR = &g_pfnVectors;
  #endif //! ARM_CORE_HAS_VTOR

  #ifdef DEBUG_BUTTON_LINK_DEBUG
  G_io_button.link_debug = DEBUG_BUTTON_LINK_DEBUG_INITIAL;
  #endif // DEBUG_BUTTON_LINK_DEBUG

  //clock_config();

  // stabilize usb before engaged
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


#ifdef BOOTLOADER_UPGRADE

  // only setup the bootsector signature when not already set to avoid tearing during tearing recovery issue
  extern void Reset_Handler(unsigned int button_press_duration);
  if (N_bootsector.bs_magic != BS_MAGIC) {
    bootsector_nvram_t ramconf;
    memset(&ramconf, 0, sizeof(ramconf));

    // ignore other fields.
    // in case of tearing, the old bootloader is left untouched and waits for a code to be loaded
    ramconf.bs_magic = BS_MAGIC;
    ramconf.bs = Reset_Handler;

    // erase the whole page
    nvm_write(&N_bootsector, &ramconf, sizeof(ramconf));
    // ensure flushed to nvram
    nvm_write_flush();
  }

  // --------- START UPGRADE STUFF HERE

  // patch byte arrays
  unsigned int address;
  unsigned int length;
  const unsigned char* pointer = &upgrade_array[0];
  address = U4BE(pointer, 0);
  length = U4BE(pointer, 4);
  while (address != -1UL && length != -1UL) {
    nvm_write((void*)address, (void*)(pointer+8), length);
    pointer += 8+length;
    address = U4BE(pointer, 0);
    length = U4BE(pointer, 4);
  }

  // --------- END UPGRADE STUFF HERE
  
  // erase the upgrade antitearing signature
  {
    bootsector_nvram_t ramconf;
    memset(&ramconf, 0, sizeof(ramconf));

    // erase the whole page
    nvm_write(&N_bootsector, &ramconf, sizeof(ramconf));
    // ensure flushed to nvram
    nvm_write_flush();
  }


#else // BOOTLOADER_UPGRADE

#ifndef TESTLONGLABEL
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0); // avoid overrun error on the SE iso link when receiving

  // initialize the button press duration
  // button_it configuration for press delay determination
  // set pullup, connected to ground if pressed
  __GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  __GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
#endif // TESTLONGLABEL

  __asm("cpsie i");
  
  G_io_seproxyhal_ticker_current_ms = 0;
  G_io_seproxyhal_ticker_enabled = 0;

  // initialize screen session and charge pump, but no display yet
  screen_init(0);

  
  screen_invert(0);
  screen_brightness(20); // default brightness until otherwise changed
  // only clear the screen
  main_setup_screen_rotation(0);
  //screen_clear();//will be done by the SE
  
  G_io_button.duration_ms = 0; // approx
  G_io_button.displayed_mode = 0;
  G_io_button.pressed = 0; 
  G_io_button.next_event_ms = 0;


  // display boot screen as the delay is ok
  //display_l4_boot();

  // only lit screen when logo is on screen
  //screen_update();//will be done by the SE


  G_seproxyhal_event_timeout_enable = 0;

  // let the bootloader screen for awhile
  HAL_Delay(500);

  G_io_apdu_protocol_enabled = 0;

  //clock_config();

  PRINTF("Power up ST31\n");

#ifndef TESTLONGLABEL
  // USART test with hsi clock
  // open iso clock
  SE_iso_power(1);
#endif

#ifdef DEMO_MODE
  demo_init();
  unsigned int last_btn = G_io_button.pressed;
  while(1) {
    if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BUTTON) {
      __asm volatile("cpsid i");
      G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_BUTTON;
      __asm volatile("cpsie i");

      // button action OR'ed
      last_btn |= G_io_button.pressed;

      // only action upon button release
      if (G_io_button.pressed == 0) {
        demo_btn_handler(last_btn);
        // consume buttons
        last_btn = 0;
      }
    }
  }
#endif

#ifdef PROBER_MODE
  prober_init();
  unsigned int last_btn = G_io_button.pressed;
  while(1) {
    if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BUTTON) {
      __asm volatile("cpsid i");
      G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_BUTTON;
      __asm volatile("cpsie i");

      // button action OR'ed
      last_btn |= G_io_button.pressed;

      // only action upon button release
      if (G_io_button.pressed == 0) {
        prober_btn_handler(last_btn);
        // consume buttons
        last_btn = 0;
      }
    }
  }
#endif

#ifdef HAVE_SE_PERSO
  PRINTF("ATR: %.*H\n", 20, G_io_se_atr);

  // ATR for ST31 Ledger BL/BOLOS
  if (G_io_se_atr_length != 5) {   
    unsigned int boot_mode_sent = 0;

    const bagl_element_t elmt = {{BAGL_LABEL                         , 0x00,    0, 10,  128,  0, 10, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_CENTER, 0 }, "SE PERSO MODE" };

    bagl_draw_with_context(&elmt, elmt.text, strlen(elmt.text), BAGL_ENCODING_LATIN1);
    screen_update();


    // request standard ISO apdu transport between the SE and the MCU instead of raw SPI packet (to save overhead)
    G_io_apdu_protocol_enabled = 1;

#ifdef HAVE_BLE
    // ######### ENABLE BLE TRANSPORT
    // await for BLE connection to process APDU through ST ISO bootloader
    const char recovery_name [] = {AD_TYPE_COMPLETE_LOCAL_NAME,'L','e','d','g','e','r',' ', 'B','l','u','e' /*,' ', 'B','o','o','t','l','o','a','d','e','r'*/, '\0'};
    //BLE_power(0, recovery_name);
    BLE_power(1, recovery_name);
#endif // HAVE_BLE

    // ######### ENABLE USB TRANSPORT
    /* Init Device Library */
    USBD_Init(&USBD_Device, &HID_Desc, 0);
    /* Register the HID class */
    USBD_RegisterClass(&USBD_Device, &USBD_CUSTOM_HID);
    USBD_CUSTOM_HID_RegisterInterface(&USBD_Device, &USBD_CustomHID_template_fops);
    /* Start Device Process */
    USBD_Start(&USBD_Device);
    io_usb_hid_init();

    // reset card, as the usb power on somehow messed up the usart
    //SE_iso_power(1);

    while(1) {

#ifdef HAVE_TOUCHPANEL
      screen_update_touch_event();
#endif // HAVE_TOUCHPANEL

#ifdef HAVE_BLE
      // stop ble upon disconnection
      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_DISCONNECT) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_BLE_DISCONNECT;

        BLE_power(0, NULL);
        BLE_power(1, recovery_name);
      }
#endif // HAVE_BLE


      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BUTTON) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_BUTTON;



        // push event is not userful but for awakening
        backlight_enable(1);
      }

#ifdef HAVE_TOUCHPANEL
      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_TOUCH) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_TOUCH;
        PRINTF("TT. ");
      }

      if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_RELEASE) {
        G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_RELEASE;

        // undim
        if (backlight_is_enabled()) {
          backlight_enable(1);
        }

        //display_l4_mode_touch();
      }
#endif // HAVE_TOUCHPANEL

#ifdef HAVE_BLE
      // check for an apdu via ble
      if (G_io_ble.apdu_available) {
        backlight_enable(1);

        // consume apdu availability under critical section
        __asm("cpsid i");
        G_io_ble.apdu_available = 0;
        __asm("cpsie i");
        __asm volatile ("nop"); 
        __asm volatile ("nop"); 

        // special reset command
        if (G_io_apdu_buffer[0] == 0xFE
          && G_io_apdu_buffer[1] == 0xFE
          && G_io_apdu_buffer[2] == 0xFE
          && G_io_apdu_buffer[3] == 0xFE) {
          G_io_apdu_buffer[0] = 0x90;
          G_io_apdu_buffer[1] = 0x00;
          BLE_protocol_send(G_io_apdu_buffer, 2);


          HAL_Delay(200);

          // go back to booloader to load the seproxyhal firm
          return; 


          // just in case
          for (;;) {
            NVIC_SystemReset();
          }
        }
      
        // process and reply the apdu  
        BLE_protocol_send(G_io_apdu_buffer, SE_iso_exchange_apdu(G_io_apdu_buffer, G_io_ble.apdu_length)); 
        
        // process ble activity if any
        if (HCI_require_processing()) {
          SCB->ICSR |= 1<<28;
        }
      }
#endif // HAVE_BLE

      // check for an apdu via usb
      if ((io_exchange(CHANNEL_APDU, 0))) {

        backlight_enable(1);

        // special reset command
        if (G_io_apdu_buffer[0] == 0xFE
          && G_io_apdu_buffer[1] == 0xFE
          && G_io_apdu_buffer[2] == 0xFE
          && G_io_apdu_buffer[3] == 0xFE) {
          G_io_apdu_buffer[0] = 0x90;
          G_io_apdu_buffer[1] = 0x00;
          io_exchange(CHANNEL_APDU|IO_RETURN_AFTER_TX, 2);

          // prepare for seproxyhal flash
          if (boot_mode_sent) {
            HAL_Delay(200);

            // go back to booloader to load the seproxyhal firm
            return; 
          }

          // just in case
          for (;;) {
            NVIC_SystemReset();
          }
        }

        // check if boot user mode or boot user emulation mode has been seen
        if (G_io_apdu_buffer[0] == 0x80 && (G_io_apdu_buffer[1] == 0xA6 || G_io_apdu_buffer[1] == 0xA0)) {
          // need seproxyhal next time
          boot_mode_sent = 1;
        }

        io_exchange(CHANNEL_APDU|IO_RETURN_AFTER_TX, SE_iso_exchange_apdu(G_io_apdu_buffer, G_io_apdu_length));
      }
    }

    // reset the chip and go back bootloader
    return;
  }
  else {
    /*
    const bagl_element_t elmt = {{BAGL_LABEL                         , 0x00,    0, 10,  128,   32, 0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_CENTER, 0 }, "PRESS RECOVERY TO FLASHBACK" };

    bagl_draw_with_context(&elmt, elmt.text, strlen(elmt.text), BAGL_ENCODING_LATIN1);
    screen_update();

    G_io_button.duration_ms = 0;
    G_io_button.displayed_mode = 0;
    G_io_button.pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)?0:1) | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)?0:2); // fake pressed (booting means pressed)
    while(1) {

      if ((G_io_button.pressed == 1) && G_io_button.duration_ms >= BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY && !(G_io_button.displayed_mode & MODE_SE_RECOVERY) ) 
      */
      {
        // perform flashback
        // session start the SE
        G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_SESSION_START_EVENT;
        G_io_seproxyhal_buffer[1] = 0;
        G_io_seproxyhal_buffer[2] = 1+4+1+sizeof(VERSION);

        // flash back request
        G_io_seproxyhal_buffer[3] = 4;

        // compute the feature flags
        unsigned int features = 0;
        G_io_seproxyhal_buffer[4] = (features>>24);
        G_io_seproxyhal_buffer[5] = (features>>16);
        G_io_seproxyhal_buffer[6] = (features>>8);
        G_io_seproxyhal_buffer[7] = (features>>0);

        // version
        G_io_seproxyhal_buffer[8] = sizeof(VERSION);
        memcpy(G_io_seproxyhal_buffer+8+1, VERSION, sizeof(VERSION));

        io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3+1+4+1+sizeof(VERSION));
        G_io_seproxyhal_state = WAIT_COMMAND;


        HAL_Delay(200);

        // go back se perso mode
        NVIC_SystemReset();

      }


      /*
      // too long, sorry
      if (G_io_button.pressed != 0 && G_io_button.duration_ms >= BUTTON_PRESS_DURATION_BOOT_POWER_OFF && !(G_io_button.displayed_mode & MODE_POWER_OFF) ) {
        //pre_harakiri();
        G_io_button.displayed_mode |= MODE_POWER_OFF;
      }
      */
      /*
    update_pressed:
      G_io_button.pressed = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)?0:1) | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)?0:2); // fake pressed (booting means pressed)
    }
    */
  }


#endif // HAVE_SE_PERSO

  // NOTE: cannot be in bagl init, as it doesn't exists, and as the memory to be used is not reserved by bagl
  // erase the animation status to reset the animation if the component is animated            
  memset(&bagl_animated[0], 0, sizeof(bagl_animated_t));

#ifdef TESTLONGLABEL

  // little endian to little endian, cross finger and hope for same fields alignment
  const bagl_component_t c[] = {
      {BAGL_LABEL, 0x00,    32, 10,  64, 13, 10, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_EXTRABOLD_11px|BAGL_FONT_ALIGNMENT_CENTER, 32 },
      {BAGL_RECTANGLE, 0x00,    0,  0,  32,  5,  0, 0, BAGL_FILL, 1, 0, 0    ,  0 },
      {BAGL_RECTANGLE, 0x00,   64,  0,  32,  5,  0, 0, BAGL_FILL, 1, 0, 0    ,  0 },
      {BAGL_RECTANGLE, 0x00,    0,  0,  16, 32,  0, 0, BAGL_FILL, 1, 0, 0    ,  0 },
      {BAGL_LABEL, 0x00,    32, 23,  64, 13,  0, 0, 0, 1, 0, BAGL_FONT_OPEN_SANS_REGULAR_11px|BAGL_FONT_ALIGNMENT_CENTER, 0 },
  };

  bagl_draw_with_context(&c[1], NULL, 0, 0);  
  bagl_draw_with_context(&c[2], NULL, 0, 0);
  bagl_draw_with_context(&c[3], NULL, 0, 0);

  memcpy(&bagl_animated[0].c, &c[0], sizeof(bagl_component_t));

  // store the last 
  bagl_animated[0].text = "SUPER LONG TEXT THAT DOES NOT FIT THE SCREEN OMG OMG OMG!!";
  bagl_animated[0].text_length = strlen(bagl_animated[0].text);
  bagl_animated[0].text_encoding = BAGL_ENCODING_LATIN1;

  bagl_draw_with_context(&c[0], bagl_animated[0].text, bagl_animated[0].text_length, bagl_animated[0].text_encoding);
  bagl_draw_with_context(&c[4], "OYEAH LINE", 10, bagl_animated[0].text_encoding);
  //screen_update();
  void screen_animate(int);
  
  screen_animate(0);
  for(;;) {
    // animate label if any to be animated
    bagl_animate(&bagl_animated[0], uwTick, SYSTICK_MS);
  }
  return;
#endif // TESTLONGLABEL


#ifdef HAVE_BLE
  // session start the SE
  G_io_ble.powered = 0;
#endif // HAVE_BLE  

  // we can't talk to the st loader in seproxyhal dialect
  if (G_io_se_atr_length != 5) {   
    NVIC_SystemReset();
  }

  // Small delay for baudrate to be taken into account in the SE
  HAL_Delay(2);

  // session start the SE
  G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_SESSION_START_EVENT;
  G_io_seproxyhal_buffer[1] = 0;
  G_io_seproxyhal_buffer[2] = 1+4+1+sizeof(VERSION);
  // ask SE recovery when button press is longer than the required delay
#ifdef FORCE_ST31_RECOVERY
  G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_SESSION_START_EVENT_RECOVERY; 
#else
  G_io_seproxyhal_buffer[3] = button_press_duration >= BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY ? SEPROXYHAL_TAG_SESSION_START_EVENT_RECOVERY:0;
#endif
#ifdef DEV_RECOVERY_IS_FLASHBACKREQ
  G_io_seproxyhal_buffer[3] = button_press_duration >= BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY ? 4:0;
#endif // DEV_RECOVERY_IS_FLASHBACKREQ
#ifdef DEBUG_BUTTON_FLASHBACK
  // flashback asked
  if (G_io_button.displayed_mode & MODE_SE_FLASHBACK || button_press_duration >= BUTTON_PRESS_DURATION_BOOT_SE_FLASHBACK) {
    G_io_seproxyhal_buffer[3] |= 4;
  }
#endif // DEBUG_BUTTON_FLASHBACK
#ifdef DEBUG_BUTTON_ALWAYS_PUSHED
  G_io_seproxyhal_buffer[3] = 0;
#endif // DEBUG_BUTTON_ALWAYS_PUSHED


  // compute the feature flags
  unsigned int features = 0
                        | SEPROXYHAL_TAG_SESSION_START_EVENT_FEATURE_USB
                        | (2<<SEPROXYHAL_TAG_SESSION_START_EVENT_FEATURE_BUTTON_COUNT_POS)
                        | SEPROXYHAL_TAG_SESSION_START_EVENT_FEATURE_SCREEN_SML
                        ;

  G_io_seproxyhal_buffer[4] = (features>>24);
  G_io_seproxyhal_buffer[5] = (features>>16);
  G_io_seproxyhal_buffer[6] = (features>>8);
  G_io_seproxyhal_buffer[7] = (features>>0);

  // version
  G_io_seproxyhal_buffer[8] = sizeof(VERSION);
  memcpy(G_io_seproxyhal_buffer+8+1, VERSION, sizeof(VERSION));

  io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3+1+4+1+sizeof(VERSION));
  G_io_seproxyhal_state = WAIT_COMMAND;

#ifdef DEV_RECOVERY_IS_FLASHBACKREQ
  if (button_press_duration >= BUTTON_PRESS_DURATION_BOOT_SE_RECOVERY) {
    // go back BL to reflash the se_perso
    HAL_Delay(200);
    return;
  }
#endif // DEV_RECOVERY_IS_FLASHBACKREQ

#ifdef FORCE_ST31_BACK_TO_STLOADER
  for(;;);
#endif // FORCE_ST31_BACK_TO_STLOADER


#ifdef HAVE_SEPROXYHAL
  volatile unsigned short rx;
  volatile unsigned short tx;
  volatile unsigned int i;
  volatile unsigned int l;
  volatile unsigned int x,y;
  volatile unsigned int usb_tx = 0;

#ifdef IO_USB_HID
  io_usb_hid_init();
#endif // IO_USB_HID

  unsigned int bagl_l;

  // init unsecure firm signature check
  G_io_unsec_chunk.offset = 0;
  USBD_Device.pData = NULL;

  for(;;) {     

    // animate label if any to be animated
    bagl_animate(&bagl_animated[0], uwTick, SYSTICK_MS);

    switch (G_io_seproxyhal_state) {
      case WAIT_EVENT:

        // check if any message is comming from the SE, it is unexpected but for printf
        
        // avoid polling, stay low power
        rx = io_seproxyhal_rx_available();
        if (rx >= 3) {
          // receive tlv header
          rx = io_seproxyhal_recv(G_io_seproxyhal_buffer, sizeof(G_io_seproxyhal_buffer));
          l = (G_io_seproxyhal_buffer[1]<<8)|(G_io_seproxyhal_buffer[2]&0xFF);

          // reset power off, a new command has been received (even if not expected)
          backlight_enable(1);
		  
          switch(G_io_seproxyhal_buffer[0]) {
            default:
              #ifdef DEBUG_BUTTON_LINK_DEBUG
              if (G_io_button.link_debug) {
                screen_printf("unexpected tlv: %.*H\n", MIN(l+3, 256), G_io_seproxyhal_buffer);
              }
              #endif // DEBUG_BUTTON_LINK_DEBUG
              break;

            // interpret printf within wait event, for easier debug of hardfault on the SE 
            // side (and of various invalid protocol handling)
            case SEPROXYHAL_TAG_PRINTF_STATUS: {
#ifdef HAVE_PRINTF
              unsigned char* pc = &G_io_seproxyhal_buffer[3];
              //poor's man printf pipe
              while(l--) {
                screen_printc(*pc++);
              }
#endif // HAVE_PRINTF
              // printf ok
              G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
              G_io_seproxyhal_buffer[1] = 0;
              G_io_seproxyhal_buffer[2] = 0;

              io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3);
              break;
            }
          }
        }
        
#ifdef HAVE_TOUCHPANEL
        // check for touch events (i2c touchscreen is somewhat interacting with usart)
        screen_update_touch_event();
#endif // HAVE_TOUCHPANEL

        // ready to send the next incoming event (BLE/NFC/TOUCH/WATCHDOG/USB etc)
        if (G_io_seproxyhal_events) {
        
          volatile unsigned int consumed_events = 0;

          // power off
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_WATCHDOG) {
            // VERY OVERPOWERED => chlik !chlak
          }
#ifdef HAVE_UNSEC_CHUNK_READ
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_UNSEC_CHUNK) {
            #define UNSEC_TOTAL_LENGTH (&_esignature-&_signed) // implicit
            #define UNSEC_SIGNED_LENGTH (&_esigned-&_signed)

            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_UNSEC_CHUNK_EVENT;
            G_io_seproxyhal_buffer[1] = G_io_unsec_chunk.size>>8;
            G_io_seproxyhal_buffer[2] = G_io_unsec_chunk.size;

            // send in 2 packets
            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3);

            // first packet encode the total length on a U4BE
            if (G_io_unsec_chunk.offset==0) {
              G_io_unsec_chunk.size-=4;
              G_io_seproxyhal_buffer[0] = UNSEC_SIGNED_LENGTH>>24;
              G_io_seproxyhal_buffer[1] = UNSEC_SIGNED_LENGTH>>16;
              G_io_seproxyhal_buffer[2] = UNSEC_SIGNED_LENGTH>>8;
              G_io_seproxyhal_buffer[3] = UNSEC_SIGNED_LENGTH;
              io_seproxyhal_send(G_io_seproxyhal_buffer, 4);
            }

            if (G_io_unsec_chunk.size) {
              io_seproxyhal_send((void*)(((unsigned int)&_signed)+G_io_unsec_chunk.offset), G_io_unsec_chunk.size);
            }

            // prepare next chunk if any
            G_io_unsec_chunk.offset+=G_io_unsec_chunk.size;

            consumed_events = SEPROXYHAL_EVENT_UNSEC_CHUNK;
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;            
          }
#endif // HAVE_UNSEC_CHUNK_READ

#ifdef HAVE_BLE
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_CONNECT) {
            PRINTF("BC. ");
            
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_CONNECTION_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = 1; // conn
            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 4);

            consumed_events = SEPROXYHAL_EVENT_BLE_CONNECT;
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_DISCONNECT) {
            PRINTF("BD. ");
            

            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_CONNECTION_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = 0; // disconn
            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 4);

            consumed_events = SEPROXYHAL_EVENT_BLE_DISCONNECT;
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_WRITE) {
            //PRINTF("BW. ");
            // forge BLE packet to the SE
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_WRITE_REQUEST_EVENT;
            G_io_seproxyhal_buffer[1] = (G_io_ble.last_write_size+3)>>8;
            G_io_seproxyhal_buffer[2] = (G_io_ble.last_write_size+3);
            for (i = 0; i < IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT; i++) {
              if (G_io_seproxyhal_ble_handles[i] == G_io_ble.last_write_attr_handle) {
                G_io_seproxyhal_buffer[3] = i;
                break;
              }
            }
            G_io_seproxyhal_buffer[4] = G_io_ble.last_write_size>>8;
            G_io_seproxyhal_buffer[5] = G_io_ble.last_write_size;

            // send in 2 packets
            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3+3);
            io_seproxyhal_send(G_io_ble.last_write_buffer, G_io_ble.last_write_size);

            // switch to command state until the SE replies a general status
            G_io_seproxyhal_state = WAIT_COMMAND;
            // validate write on the blue tooth media, to avoid overwriting the previous value
            BLE_accept_previous_write();
            consumed_events = SEPROXYHAL_EVENT_BLE_WRITE;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_READ) {
            //PRINTF("BR. ");
            for (i = 0; i < IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT; i++) {
              if (G_io_seproxyhal_ble_handles[i] == G_io_ble.last_read_attr_handle) {
                G_io_seproxyhal_ble_last_read_request_handle = i;
                break;
              }
            }
            G_io_seproxyhal_state = WAIT_BLE_READ_DATA;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_READ_REQUEST_EVENT;
            // TODO send the read event to the SE
            consumed_events = SEPROXYHAL_EVENT_BLE_READ;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_REGISTER) {
            PRINTF("NR. ");
            consumed_events = SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_REGISTER;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_UNREGISTER) {
            PRINTF("NU. ");
            consumed_events = SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_UNREGISTER;
            goto consume;
          }
#endif // HAVE_BLE

          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_RESET) {
            //PRINTF("UR. ");
            consumed_events = SEPROXYHAL_EVENT_USB_RESET;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_USB_EVENT_RESET;

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 4);
            __asm("cpsid i");
            G_io_usb.ep_in = 0;
            G_io_usb.ep_out = 0;
            G_io_seproxyhal_events &= ~ (SEPROXYHAL_EVENT_USB_SETUP|SEPROXYHAL_EVENT_USB_XFER_IN|SEPROXYHAL_EVENT_USB_XFER_OUT);
            __asm("cpsie i");
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_SUSPENDED) {
            //PRINTF("US. ");
            consumed_events = SEPROXYHAL_EVENT_USB_SUSPENDED;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_USB_EVENT_SUSPENDED;

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 4);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_RESUMED) {
            //PRINTF("UP. ");
            consumed_events = SEPROXYHAL_EVENT_USB_RESUMED;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_USB_EVENT_RESUMED;

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 4);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_SETUP) {
            //PRINTF("UT. ");

            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EP_XFER_EVENT;
            G_io_seproxyhal_buffer[1] = (3+8)>>8;
            G_io_seproxyhal_buffer[2] = (3+8);
            G_io_seproxyhal_buffer[3] = 0;
            G_io_seproxyhal_buffer[4] = SEPROXYHAL_TAG_USB_EP_XFER_SETUP;
            G_io_seproxyhal_buffer[5] = 8;

            __asm("cpsid i");
            memcpy(G_io_seproxyhal_buffer+6, hpcd_USB_OTG_FS.Setup, 8);
              G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_USB_SETUP;
              // cleanup ep transfer on ep0 upon setup
              G_io_usb.ep_in &= ~(1);
              if (!G_io_usb.ep_in) {
                G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_USB_XFER_IN;
              }
              G_io_usb.ep_out &= ~(1);
              if (!G_io_usb.ep_out) {
                G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_USB_XFER_OUT;
              }
            __asm("cpsie i");

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3+3+8);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consumed;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_XFER_IN) {
            unsigned char epnum;
            //PRINTF("UX. ");
            // process in before out to ensure device holds the DoS way
            if (G_io_usb.ep_in) {
              for (epnum=0; epnum<MAX_USB_BIDIR_ENDPOINTS; epnum++) {
                if (G_io_usb.ep_in & (1<<epnum)) {
                  G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EP_XFER_EVENT;
                  G_io_seproxyhal_buffer[1] = (3)>>8;
                  G_io_seproxyhal_buffer[2] = (3);
                  G_io_seproxyhal_buffer[3] = epnum|0x80;
                  G_io_seproxyhal_buffer[4] = SEPROXYHAL_TAG_USB_EP_XFER_IN;

                  __asm("cpsid i");
                  G_io_seproxyhal_buffer[5] = hpcd_USB_OTG_FS.IN_ep[epnum].xfer_count;
                  G_io_usb.ep_in &= ~(1<<epnum);
                  if (!G_io_usb.ep_in) {
                    G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_USB_XFER_IN;
                  }
                  __asm("cpsie i");

                  io_seproxyhal_send_start(G_io_seproxyhal_buffer, 6);
                  G_io_seproxyhal_state = WAIT_COMMAND;
                  goto consumed;
                }
              }
            }
            // nothing to be done, consume the event
            consumed_events = SEPROXYHAL_EVENT_USB_XFER_IN;
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_XFER_OUT) {
            unsigned char epnum;
            if (G_io_usb.ep_out) {
              for (epnum=0; epnum<MAX_USB_BIDIR_ENDPOINTS; epnum++) {
                if (G_io_usb.ep_out & (1<<epnum)) {
                  unsigned int len = MIN(sizeof(G_io_seproxyhal_buffer), PCD_GET_EP_RX_CNT(hpcd_USB_OTG_FS.Instance, epnum));
                  G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EP_XFER_EVENT;
                  G_io_seproxyhal_buffer[1] = (3+len)>>8;
                  G_io_seproxyhal_buffer[2] = (3+len);
                  G_io_seproxyhal_buffer[3] = epnum;
                  G_io_seproxyhal_buffer[4] = SEPROXYHAL_TAG_USB_EP_XFER_OUT;
                  G_io_seproxyhal_buffer[5] = len;

                  io_seproxyhal_send_start(G_io_seproxyhal_buffer, 6);

                  // read data from usb core and forward to the SE
                  PCD_ReadPMA(hpcd_USB_OTG_FS.Instance, G_io_seproxyhal_buffer, hpcd_USB_OTG_FS.OUT_ep[epnum].pmaadress, len);
                  io_seproxyhal_send(G_io_seproxyhal_buffer, len);
 
                  G_io_seproxyhal_state = WAIT_COMMAND;
                  __asm("cpsid i");
                  G_io_usb.ep_out &= ~(1<<epnum);
                  if (!G_io_usb.ep_out) {
                    G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_USB_XFER_OUT;
                  }
                  __asm("cpsie i");
                  goto consumed;
                }
              }
            }
            // nothing to be done, consume the event
            consumed_events = SEPROXYHAL_EVENT_USB_XFER_OUT; // shall not be this
            goto consume;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_USB_SOF) {
            //PRINTF("UF. ");
            consumed_events = SEPROXYHAL_EVENT_USB_SOF;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_USB_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_USB_EVENT_SOF;

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 4);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }

          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_BUTTON) {
            __asm("cpsid i");
            // avoid races :)
            G_io_seproxyhal_buffer[3] = (G_io_button.pressed << 1); // send the pressed button mask
            G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_BUTTON;
            __asm("cpsie i");
            
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BUTTON_PUSH_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 4);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consumed;
          }

#ifdef HAVE_TOUCHPANEL
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_RELEASE) {
            consumed_events = SEPROXYHAL_EVENT_RELEASE;
           
            // avoid clicking anywhere, user must use the button to awake from sleep
            if (!backlight_is_enabled()) {
              goto consume_nowakeup;
            }

            //PRINTF("TR. ");
            x = G_io_touch.ts_last_x;
            y = G_io_touch.ts_last_y;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_FINGER_EVENT_RELEASE;
            // consume x&y to avoid multiple touch release (thanks SUPERB touchscreen impl)
            G_io_touch.ts_last_x = -1;
            G_io_touch.ts_last_y = -1;
            goto send_finger_event;
          }
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_TOUCH) {
            consumed_events = SEPROXYHAL_EVENT_TOUCH;
           
            // avoid clicking anywhere, user must use the button to awake from sleep
            if (!backlight_is_enabled()) {
              goto consume_nowakeup;
            }

            //PRINTF("TT. ");
            x = G_io_touch.ts_last_x;
            y = G_io_touch.ts_last_y;
            G_io_seproxyhal_buffer[3] = SEPROXYHAL_TAG_FINGER_EVENT_TOUCH;
          send_finger_event:
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_FINGER_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 5;
            // [3] already set
            G_io_seproxyhal_buffer[4] = x>>8;
            G_io_seproxyhal_buffer[5] = x;
            G_io_seproxyhal_buffer[6] = y>>8;
            G_io_seproxyhal_buffer[7] = y;

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 8);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }
#endif // HAVE_TOUCHPANEL
#ifdef ASYNCH_DISPLAY_PROCESSED_EVENT
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_DISPLAYED) {
            consumed_events = SEPROXYHAL_EVENT_DISPLAYED;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0; 

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }

#endif // ASYNCH_DISPLAY_PROCESSED_EVENT

          // last to avoid overflowing io events with a too short ticker
          if (G_io_seproxyhal_events & SEPROXYHAL_EVENT_TICKER) {
            //PRINTF("TI. ");
            consumed_events = SEPROXYHAL_EVENT_TICKER;
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_TICKER_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0; 

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3);
            G_io_seproxyhal_state = WAIT_COMMAND;
            goto consume;
          }


          // no event to be consumed
          break;

        consume_nowakeup:
          __asm("cpsid i");
          G_io_seproxyhal_events &= ~consumed_events;
          __asm("cpsie i");
          break;

        consume:
          __asm("cpsid i");
          G_io_seproxyhal_events &= ~consumed_events;
          __asm("cpsie i");
        consumed:
#ifdef HAVE_SEPROXYHAL_NO_USB_DURING_COMMAND
          // whatever the event, usb transfer can't take place behind the SE to avoid spurious hardware race that alter the SE replied bytes
          HAL_NVIC_DisableIRQ(USB_IRQn);
#endif // HAVE_SEPROXYHAL_NO_USB_DURING_COMMAND

          // reset power off, an event command has been received
          backlight_enable(1);
        }
        break;

#ifdef HAVE_BLE
      case WAIT_BLE_READ_DATA:
#endif
      case WAIT_COMMAND:
        // avoid polling, stay low power
        rx = io_seproxyhal_rx_available();

        // discard invalid replies
        if (rx < 3) {
          // nothing to do yet
          break;
        }
        // receive tlv header
        rx = io_seproxyhal_recv(G_io_seproxyhal_buffer, sizeof(G_io_seproxyhal_buffer));
        l = (G_io_seproxyhal_buffer[1]<<8)|(G_io_seproxyhal_buffer[2]&0xFF);

        // only reset timeout when receiving a command, to give more time before the status
        G_seproxyhal_event_timeout = 0;

        // reset power off, a new command has been received
        backlight_enable(1);

#ifdef DEBUG_BUTTON_LINK_DEBUG
        if (G_io_button.link_debug >= 2) {
          screen_printf("< %.*H\n", MIN(l+3, 256), G_io_seproxyhal_buffer);
        }
#endif // DEBUG_BUTTON_LINK_DEBUG

        switch(G_io_seproxyhal_buffer[0]) {
#ifdef HAVE_BLE
          case SEPROXYHAL_TAG_BLE_RADIO_POWER:
            PRINTF("BO. ");
            // turn BLE ON or OFF, use the last defined service. (make discoverable)
            BLE_power(G_io_seproxyhal_buffer[3]&0x2, NULL); // if not advertising, then don't turn on
            // set default handles
            memset(G_io_seproxyhal_ble_handles, 0, sizeof(G_io_seproxyhal_ble_handles));
            G_io_seproxyhal_ble_handles[1] = G_io_ble.tx_characteristic_handle ;
            G_io_seproxyhal_ble_handles[2] = G_io_ble.rx_characteristic_handle + 1;
            break;
          case SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_STATUS:
            //PRINTF("BI. ");
            if (l > 3) {
              // will only return when data has been propagated. timeout => watchdog
              BLE_send(G_io_seproxyhal_ble_handles[G_io_seproxyhal_buffer[3]], &G_io_seproxyhal_buffer[6], l-3);
            }

            // say ok to the SE
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_BLE_NOTIFY_INDICATE_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 1;
            // unchanged: G_io_seproxyhal_buffer[3] = G_io_seproxyhal_buffer[3];

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 4);
            // stay in command mode
            break;
#endif // HAVE_BLE

          case SEPROXYHAL_TAG_SCREEN_POWER:
            // TODO PWN off
            break;
          case SEPROXYHAL_TAG_MORE_TIME:
            // TODO add watchdog time
            break;
          case SEPROXYHAL_TAG_SE_POWER_OFF:
          case SEPROXYHAL_TAG_DEVICE_OFF:
            harakiri();
            break;

          case SEPROXYHAL_TAG_SET_TICKER_INTERVAL: {
            PRINTF("TK ");
            if (l == 2) { 
              // ticker interval ms in U2BE
              // when interval is 0 then ticker is disabled
              // consume ticker event if the ticker is being disabled (in case we took a long time to disable it)
              __asm("cpsid i");
              G_io_seproxyhal_ticker_interval_ms = (G_io_seproxyhal_buffer[3]<<8)|(G_io_seproxyhal_buffer[4]&0xFF);
              G_io_seproxyhal_ticker_enabled = (G_io_seproxyhal_ticker_interval_ms != 0);
              G_io_seproxyhal_ticker_current_ms = 0; // reset interval
              G_io_seproxyhal_events &= ~SEPROXYHAL_EVENT_TICKER;
              __asm("cpsie i");
            }
            break;
          }

          case SEPROXYHAL_TAG_PRINTF_STATUS: {
#ifdef HAVE_PRINTF
            unsigned char* pc = &G_io_seproxyhal_buffer[3];
            //poor's man printf pipe
            while(l--) {
              screen_printc(*pc++);
            }
#endif // HAVE_PRINTF
            // printf ok
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0;

            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3);
            // stay in command mode
            break;
          }
          case SEPROXYHAL_TAG_SCREEN_DISPLAY_STATUS: {
            //PRINTF("CD. ");
            if (l < sizeof(bagl_component_t)) {
              PRINTF("INVALID");
              // too short packet
              break;
            }

            // check last display value to avoid blink without support in the application
            if (l != bagl_l 
              || memcmp(&bagl_animated[0].c, &G_io_seproxyhal_buffer[3], MIN(sizeof(bagl_component_t), l)) != 0
              || memcmp(bagl_animated_text[0], &G_io_seproxyhal_buffer[3+sizeof(bagl_component_t)], l-sizeof(bagl_component_t)) != 0) {

              __asm("cpsid i");
              // erase the animation status to reset the animation if the component is animated            
              memset(&bagl_animated[0], 0, sizeof(bagl_animated_t));
              __asm("cpsie i");

	      // store the last 
              memcpy(bagl_animated_text[0], &G_io_seproxyhal_buffer[3+sizeof(bagl_component_t)], l-sizeof(bagl_component_t));
              bagl_animated[0].text = bagl_animated_text[0];
              bagl_animated[0].text_length = l-sizeof(bagl_component_t);
              bagl_animated[0].text_encoding = BAGL_ENCODING_LATIN1;

              // little endian to little endian, cross finger and hope for same fields alignment
              memcpy(&bagl_animated[0].c, &G_io_seproxyhal_buffer[3], MIN(sizeof(bagl_component_t), l));
              bagl_l = l;

              bagl_draw_with_context(&bagl_animated[0].c, bagl_animated[0].text, bagl_animated[0].text_length, bagl_animated[0].text_encoding);
            }

            /*
            // centering helpers
            const bagl_element_t surrounding_color_rect1 = {{BAGL_RECTANGLE                         , 0x00,    0, 0,  128,   1, 0, 0, BAGL_FILL, 0xFFFFFF, 0x000000, 0, 0   }, NULL};
            bagl_draw_with_context(&surrounding_color_rect1, NULL, 0, BAGL_ENCODING_LATIN1);
            const bagl_element_t surrounding_color_rect2 = {{BAGL_RECTANGLE                         , 0x00,    0, 0,  1,   32, 0, 0, BAGL_FILL, 0xFFFFFF, 0x000000, 0, 0   }, NULL};
            bagl_draw_with_context(&surrounding_color_rect2, NULL, 0, BAGL_ENCODING_LATIN1);
            const bagl_element_t surrounding_color_rect3 = {{BAGL_RECTANGLE                         , 0x00,    127, 0,  1,   32, 0, 0, BAGL_FILL, 0xFFFFFF, 0x000000, 0, 0   }, NULL};
            bagl_draw_with_context(&surrounding_color_rect3, NULL, 0, BAGL_ENCODING_LATIN1);
            const bagl_element_t surrounding_color_rect4 = {{BAGL_RECTANGLE                         , 0x00,    0, 31,  128,   1, 0, 0, BAGL_FILL, 0xFFFFFF, 0x000000, 0, 0   }, NULL};
            bagl_draw_with_context(&surrounding_color_rect4, NULL, 0, BAGL_ENCODING_LATIN1);
            */

            // screen debug only, else it blinks a bit too much
            //screen_update();

          #ifdef ASYNCH_DISPLAY_PROCESSED_EVENT
            __asm("cpsid i");
            G_io_seproxyhal_events |= SEPROXYHAL_EVENT_DISPLAYED;
            __asm("cpsie i");
            G_io_seproxyhal_state = WAIT_EVENT;
          #else // ASYNCH_DISPLAY_PROCESSED_EVENT
            // reply display processed event
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0;
            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3);
            // stay in command state
          #endif // ASYNCH_DISPLAY_PROCESSED_EVENT
            break;
          }

#ifdef HAVE_M24SR
          case SEPROXYHAL_TAG_NFC_READ_RESPONSE_STATUS:
            if (G_io_seproxyhal_state != WAIT_BLE_READ_DATA) {
              continue;
            }            
            BLE_send(G_io_seproxyhal_ble_last_read_request_handle, &G_io_seproxyhal_buffer[4], G_io_seproxyhal_buffer[0] - 4);
            // this is the last command
            G_io_seproxyhal_state = WAIT_EVENT;
            G_seproxyhal_event_timeout_enable = 0;
            break;
#endif // HAVE_M24SR

          case SEPROXYHAL_TAG_GENERAL_STATUS:
            //PRINTF("END ");
            if (l == 2) {
              switch((G_io_seproxyhal_buffer[3]<<8)|(G_io_seproxyhal_buffer[4]&0xFF)) {
                case SEPROXYHAL_TAG_GENERAL_STATUS_LAST_COMMAND:
                  // update screen at onde after a sequence of display command, to limit glitches
                  screen_update();

#ifdef HAVE_SEPROXYHAL_NO_USB_DURING_COMMAND
                  // Reenable OTG fs interrupt to perform requested USB transmit if any.
                  // this workaround the hardware race between usart and otg.
                  HAL_NVIC_EnableIRQ(USB_IRQn);
#endif //HAVE_SEPROXYHAL_NO_USB_DURING_COMMAND

                  for (l = 0; l < 32; l++) {
                    if (usb_tx & (1<<l)) {
                      USBD_LL_Transmit_exec(&USBD_Device, l|0x80);
                      //consume
                      usb_tx &= ~(1<<l);
                    }
                  }

                  G_io_seproxyhal_state = WAIT_EVENT;
                  G_seproxyhal_event_timeout_enable = 0;
                  break;
                /*
                case SEPROXYHAL_TAG_GENERAL_STATUS_MORE_COMMAND:
                  // card has replied and is indicating it will be speaking later on
                  G_seproxyhal_event_timeout_enable = 0;
                */
                default:
                  // remain in wait command
                  break;
              }
            }
            break;

          case SEPROXYHAL_TAG_USB_CONFIG:
            switch (G_io_seproxyhal_buffer[3]) {

              case SEPROXYHAL_TAG_USB_CONFIG_CONNECT:
                // already done at boot // clock_config();

                // startup the low level usb stack
                USBD_Device.pClass = NULL;
                USBD_Device.pData = NULL;
                USBD_LL_Init(&USBD_Device);
                USBD_LL_Start(&USBD_Device);

                // enable usb interrupt
                //HAL_NVIC_EnableIRQ(USB_IRQn);
                break;
              case SEPROXYHAL_TAG_USB_CONFIG_DISCONNECT:
                if (USBD_Device.pData) {
                  USBD_LL_Stop(&USBD_Device); 
                  USBD_Device.pData = NULL;
                }
                break;
              case SEPROXYHAL_TAG_USB_CONFIG_ADDR:
                if (USBD_Device.pData) {
                  USBD_LL_SetUSBAddress(&USBD_Device, G_io_seproxyhal_buffer[4]);
                }
                break;
              case SEPROXYHAL_TAG_USB_CONFIG_ENDPOINTS:
                if (USBD_Device.pData) {
                  // configure all endpoints
                  for(l=0;l<G_io_seproxyhal_buffer[4];l++) {
                    if (G_io_seproxyhal_buffer[5+l*3+1] == SEPROXYHAL_TAG_USB_CONFIG_TYPE_DISABLED) {
                      // still skip the other bytes
                      USBD_LL_CloseEP(&USBD_Device, G_io_seproxyhal_buffer[5+l*3]);
                    }
                    else {
                      uint8_t ep_type = -1;
                      switch(G_io_seproxyhal_buffer[5+l*3+1]) {
                        case SEPROXYHAL_TAG_USB_CONFIG_TYPE_CONTROL:
                          ep_type = USBD_EP_TYPE_CTRL;
                          break;
                        case SEPROXYHAL_TAG_USB_CONFIG_TYPE_ISOCHRONOUS:
                          ep_type = USBD_EP_TYPE_ISOC;
                          break;
                        case SEPROXYHAL_TAG_USB_CONFIG_TYPE_BULK:
                          ep_type = USBD_EP_TYPE_BULK;
                          break;
                        case SEPROXYHAL_TAG_USB_CONFIG_TYPE_INTERRUPT:
                          ep_type = USBD_EP_TYPE_INTR;
                          break;
                      }
                      USBD_LL_OpenEP(&USBD_Device, 
                                     G_io_seproxyhal_buffer[5+l*3], 
                                     ep_type, 
                                     G_io_seproxyhal_buffer[5+l*3+2]);
                    }
                  }
                }
                break;
            }
            break;

          case SEPROXYHAL_TAG_USB_EP_PREPARE:
            if (USBD_Device.pData) {
              switch (G_io_seproxyhal_buffer[4]) { 
                case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_UNSTALL:
                  USBD_LL_ClearStallEP(&USBD_Device, G_io_seproxyhal_buffer[3]);
                  break;

                case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_STALL:
                  USBD_LL_StallEP(&USBD_Device, G_io_seproxyhal_buffer[3]);
                  break;

                case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_IN:
                  USBD_LL_Transmit_delay(&USBD_Device, 
                                   G_io_seproxyhal_buffer[3]/*|0x80*/, 
                                   G_io_seproxyhal_buffer+6, 
                                   G_io_seproxyhal_buffer[5]);
                  usb_tx |= 1<<(G_io_seproxyhal_buffer[3]&0x7F);
                  break;

                case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_SETUP:
                case SEPROXYHAL_TAG_USB_EP_PREPARE_DIR_OUT:
                  USBD_LL_PrepareReceive(&USBD_Device, 
                                         G_io_seproxyhal_buffer[3]/*&0x7F*/, 
                                         (G_io_seproxyhal_buffer[5]>MAX_USB_ENDPOINT_SIZE?MAX_USB_ENDPOINT_SIZE:G_io_seproxyhal_buffer[5]));
                  break;
              }
              //HAL_Delay(15);
            }
            break;

#ifdef SEPROXYHAL_TAG_MCU_BOOTLOADER
          case SEPROXYHAL_TAG_MCU_BOOTLOADER:
            // return to bootloader, permanently
            return;
#endif

#ifdef HAVE_UNSEC_CHUNK_READ
          case SEPROXYHAL_TAG_UNSEC_CHUNK_READ:
            // reset the unsec offset read if requested
            if (G_io_seproxyhal_buffer[5]) {
              G_io_unsec_chunk.offset = 0;
            }

            // | vector | bootloader | [gap] | seproxyhal | signature |
            // ^_signed                      ^ _text      ^_esigned   ^_esignature
            G_io_unsec_chunk.size = MIN((&_esignature-&_signed)-G_io_unsec_chunk.offset, 
                                        (G_io_seproxyhal_buffer[3]<<8) | G_io_seproxyhal_buffer[4]);
										
            // prepare async event after get status
            __asm("cpsid i");
            G_io_seproxyhal_events |= SEPROXYHAL_EVENT_UNSEC_CHUNK;
            __asm("cpsie i");

            break;
#endif // HAVE_UNSEC_CHUNK_READ

          case SEPROXYHAL_TAG_SET_SCREEN_CONFIG:
            screen_invert(G_io_seproxyhal_buffer[3]&1);
            screen_brightness(G_io_seproxyhal_buffer[4]);
            switch(G_io_seproxyhal_buffer[3]&6) {
              case 0:
                main_setup_screen_rotation(0);              
                break;
              case 4:
                main_setup_screen_rotation(180);
                break;
            }
            screen_update();
            break;

          case SEPROXYHAL_TAG_SCREEN_ANIMATION_STATUS:
            screen_animate(G_io_seproxyhal_buffer[3], &G_io_seproxyhal_buffer[4]);

          #ifdef ASYNCH_DISPLAY_PROCESSED_EVENT
            __asm("cpsid i");
            G_io_seproxyhal_events |= SEPROXYHAL_EVENT_DISPLAYED;
            __asm("cpsie i");
            G_io_seproxyhal_state = WAIT_EVENT;
          #else // ASYNCH_DISPLAY_PROCESSED_EVENT
            // reply display processed event
            G_io_seproxyhal_buffer[0] = SEPROXYHAL_TAG_DISPLAY_PROCESSED_EVENT;
            G_io_seproxyhal_buffer[1] = 0;
            G_io_seproxyhal_buffer[2] = 0;
            io_seproxyhal_send_start(G_io_seproxyhal_buffer, 3);
            // stay in command state
          #endif // ASYNCH_DISPLAY_PROCESSED_EVENT
            break;
        }
        break;
    }
  }

#endif // HAVE_SEPROXYHAL

#endif // BOOTLOADER_UPGRADE

  // shall never be reached
  NVIC_SystemReset();
}


void io_seproxyhal_send_start(unsigned char * buffer, unsigned short length) {
  // reset command timeout detection
  G_seproxyhal_event_timeout = 0;
  G_seproxyhal_event_timeout_enable = 1;
  memmove(G_seproxyhal_event_timeout_header, buffer, 3);
#ifdef DEBUG_BUTTON_LINK_DEBUG
  if (G_io_button.link_debug >= 2) {
    screen_printf("> %.*H\n", MIN(length, 256), buffer);
  }
#endif // DEBUG_BUTTON_LINK_DEBUG
  io_seproxyhal_send(buffer, length);
}

/**
 * @}
 */

/**
 * @}
 */

