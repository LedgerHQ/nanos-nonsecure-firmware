/*******************************************************************************
*   Ledger Blue - Non secure firmware
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

#ifndef SEPROXYHAL_H
#define SEPROXYHAL_H

#include "seproxyhal_protocol.h"

// chip's frequency, useful for many subsystems initialization
extern unsigned int frequency_hz;

#define IO_SEPROXYHAL_BUFFER_SIZE_B 300
extern volatile unsigned char G_io_seproxyhal_buffer[IO_SEPROXYHAL_BUFFER_SIZE_B];
extern volatile unsigned int G_io_seproxyhal_events;

#define SEPROXYHAL_EVENT_WATCHDOG                       0x001UL
#define SEPROXYHAL_EVENT_TOUCH                          0x002UL
#define SEPROXYHAL_EVENT_BLE_WRITE                      0x004UL
#define SEPROXYHAL_EVENT_BLE_READ                       0x008UL
#define SEPROXYHAL_EVENT_USB_XFER_IN                    0x010UL
#define SEPROXYHAL_EVENT_USB_SETUP                      0x020UL
#define SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_REGISTER    0x040UL
#define SEPROXYHAL_EVENT_BLE_NOTIFIFICATION_UNREGISTER  0x080UL
#define SEPROXYHAL_EVENT_BLE_CONNECT                    0x100UL
#define SEPROXYHAL_EVENT_BLE_DISCONNECT                 0x200UL
#define SEPROXYHAL_EVENT_RELEASE                        0x400UL
#define SEPROXYHAL_EVENT_TICKER                         0x800UL
//#define SEPROXYHAL_EVENT_USB_OUT                       0x1000UL
#define SEPROXYHAL_EVENT_USB_RESET                     0x2000UL
#define SEPROXYHAL_EVENT_USB_SOF                       0x4000UL
#define SEPROXYHAL_EVENT_USB_SUSPENDED                 0x8000UL
#define SEPROXYHAL_EVENT_USB_RESUMED                  0x10000UL
#define SEPROXYHAL_EVENT_USB_XFER_OUT                 0x20000UL
#define SEPROXYHAL_EVENT_BUTTON                       0x40000UL
#define SEPROXYHAL_EVENT_DISPLAYED                    0x80000UL
#define SEPROXYHAL_EVENT_UNSEC_CHUNK                 0x100000UL
#define SEPROXYHAL_EVENT_SET_LINK_SPEED              0x200000UL
#define SEPROXYHAL_EVENT_BLE_NOTIFY_INDICATE         0x400000UL

#ifdef HAVE_TOUCHPANEL
extern volatile struct touch_state_s {
  short ts_last_x; 
  short ts_last_y;
  short ts_last_x2; 
  short ts_last_y2;
  short ts_last_2;
} G_io_touch;
#endif // HAVE_TOUCHPANEL

extern volatile unsigned char G_io_apdu_protocol_enabled;

extern volatile unsigned char G_io_apdu_buffer[260];
extern volatile unsigned short G_io_apdu_length;

#ifdef HAVE_BLE
extern volatile struct ble_state_s {
  unsigned int powered;
  
#ifdef BLE_PACKET_ACK
  unsigned char apdu_transport_ack;
#endif // BLE_PACKET_ACK

#ifdef HAVE_BLE_MCU
#define IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT 32
  unsigned char handles[IO_SEPROXYHAL_BLE_HANDLE_MAXCOUNT];
  // public
  unsigned char apdu_available;
  unsigned short apdu_length;
  unsigned short gap_service_handle;
  unsigned short gap_dev_name_char_handle;
  unsigned short gap_appearance_char_handle;

  unsigned short service_handle, tx_characteristic_handle, rx_characteristic_handle;
  
  unsigned char client_link_established;
  unsigned short apdu_transport_remlen;
  unsigned short apdu_transport_seq;
  unsigned char* apdu_transport_ptr;
  unsigned char apdu_transport_busy_sending;
  unsigned char apdu_transport_lock;
  unsigned char connection_reset;
  unsigned int connection_timeout_ms;
  unsigned char connection_timeout_enabled;
  unsigned short notification_reg_handle;
  unsigned short notification_unreg_handle;
  unsigned short last_read_conn_handle;
  unsigned short last_read_attr_handle;
  unsigned char  last_read_request_handle;
  unsigned short last_write_conn_handle;
  unsigned short last_write_attr_handle;
  unsigned char last_write_size;
  unsigned char last_write_buffer[BLE_PACKET_MAX_SIZE];

  unsigned char* last_discovered_name;
  unsigned int notify_indicate_char;
#endif // HAVE_BLE_MCU
  unsigned char bluenrg_buffer [128/*HCI_READ_PACKET_SIZE*/];
  unsigned int bluenrg_buffer_size;
} G_io_ble;

#ifdef HAVE_BLE_MCU
void BLE_accept_previous_write(void);
void BLE_send(unsigned short handle, uint8_t* data_buffer, uint8_t Nb_bytes);
#endif // HAVE_BLE_MCU

void BLE_power(unsigned char powered, const char* discovered_name);
// called by the EXTI corresponding to the BlueNRG
void BlueNRG_EXTI_Process(void);
// called upon pendsv
void BlueNRG_PENDSV_Process(void);
#endif // HAVE_BLE

#ifdef HAVE_SE
extern unsigned int G_io_se_link_next_etu;
extern unsigned int G_io_se_link_next_mhz;
// return 1 if ok
unsigned char SE_iso_power_up(void);
// return 1 if ok
unsigned char SE_iso_power(unsigned char powered);
void SE_set_link_speed(unsigned int mhz, unsigned int etu);
unsigned short io_seproxyhal_rx_available(void);
unsigned short io_seproxyhal_recv(unsigned char* buffer, unsigned short length);
void io_seproxyhal_send_start(unsigned char * buffer, unsigned short length);
void io_seproxyhal_send(unsigned char* buffer, unsigned short length);
unsigned short SE_iso_exchange_apdu(unsigned char* apdu, unsigned short length);
#endif // HAVE_SE

#ifdef HAVE_LED
void led_init(void);
void setled(unsigned int color);
unsigned int getled(void);
#endif // HAVE_LED

void screen_init(unsigned char reinit);
void screen_clear(void);
void screen_poweroff(void);
void screen_update(void);
void screen_printf(const char* format,...);
void screen_xy(unsigned short x, unsigned short y, unsigned short rotation);
void screen_on(void); 
void screen_update_touch_event(void);
void screen_brightness(uint8_t percentage);
void screen_invert(unsigned int inverted);
void screen_rotation(unsigned int degree);
void screen_animation(unsigned int kind, unsigned char* parameters);
unsigned int battery_get_level_mv(void);

extern volatile struct usb_state_s {
  // up to 32 ep manageable this way
  unsigned int  ep_in;
  unsigned int  ep_out;

  unsigned int  ep_in_len[MAX_USB_BIDIR_ENDPOINTS];
#ifdef HAVE_USBL4
  unsigned int  ep_out_len[MAX_USB_BIDIR_ENDPOINTS];
  unsigned char ep_out_buff[MAX_USB_BIDIR_ENDPOINTS][MAX_USB_ENDPOINT_SIZE];
#endif // HAVE_USBL4
} G_io_usb;

#define CHANNEL_APDU 0
#define CHANNEL_KEYBOARD 1
#define CHANNEL_SPI 2
#define IO_RESET_AFTER_REPLIED 0x80
#define IO_RECEIVE_DATA 0x40
#define IO_RETURN_AFTER_TX 0x20
#define IO_FLAGS 0xF0

typedef struct {
  uint32_t duration_ms;
  uint32_t pressed;
  #define MODE_SE_RECOVERY    1
  #define MODE_MCU_BOOTLOADER 2
  #define MODE_POWER_OFF      4
  #define MODE_POWER_ON       8
  #define MODE_BOOT          16
#ifdef DEBUG_BUTTON_FLASHBACK
  #define MODE_SE_FLASHBACK  32
#endif // DEBUG_BUTTON_FLASHBACK
#ifdef DEBUG_BUTTON_LINK_DEBUG
  #define MODE_LINK_DEBUG    64
#endif // DEBUG_BUTTON_LINK_DEBUG
  uint32_t displayed_mode;


#ifdef DEBUG_BUTTON_LINK_DEBUG
  unsigned int link_debug;
#endif // DEBUG_BUTTON_LINK_DEBUG
  uint32_t boot_moment;

  uint32_t next_event_ms;

  // button mapping when screen is rotated
  unsigned char mapping[2];
} io_button_t;
extern volatile io_button_t G_io_button;

extern unsigned short io_exchange(unsigned char flags, unsigned short tx_length);
extern void io_usb_hid_init(void);

extern unsigned char _signed;
extern unsigned char _esigned;
extern unsigned char _esignature;
typedef struct {
  unsigned int size;
  unsigned int offset;
} io_unsec_chunk_t;
extern volatile io_unsec_chunk_t G_io_unsec_chunk;


#define WAIT_COMMAND 1
#define WAIT_EVENT 2
#ifdef HAVE_BLE
#define WAIT_BLE_READ_DATA 3
#endif // HAVE_BLE
#define WAIT_COMMAND_TICKER 4

#define U2(hi,lo) ((((hi)&0xFF)<<8) | ((lo)&0xFF))
#define U4(hi3, hi2, lo1,lo0) ((((hi3)&0xFF)<<24) | (((hi2)&0xFF)<<16) | (((lo1)&0xFF)<<8) | ((lo0)&0xFF))
#define U2BE(buf, off) ((((buf)[off]&0xFF)<<8) | ((buf)[off+1]&0xFF) )
#define U4BE(buf, off) ((U2BE(buf, off)<<16) | (U2BE(buf, off+2)&0xFFFF))
#define U2LE(buf, off) ( (((buf)[off]&0xFF)) | (((buf)[off+1]&0xFF)<<8) )
#define U4LE(buf, off) ( (U2LE(buf, off)&0xFFFF) | ((U2LE(buf, off+2)&0xFFFF)<<16))
#define MIN(x,y) ((x)<(y)?(x):(y))
#define MAX(x,y) ((x)>(y)?(x):(y))

// faster than the huge hal
#define REG_SET(reg, mask, value) reg = (reg & ~mask) | value 
#define REG_GET(reg, mask) (reg & mask)
#define BB_OUT(port, pin, value) REG_SET((port)->ODR, (1<<(pin)), (value<<(pin))) 
#define BB_IN(port, pin) REG_GET((port)->IDR, (1<<(pin)))

#endif

