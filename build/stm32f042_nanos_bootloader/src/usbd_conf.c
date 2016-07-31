/**
  ******************************************************************************
  * @file           : usbd_conf.c
  * @brief          : This file implements the board support package for the USB device library
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"

#include "seproxyhal.h"
//#include "usbd_hid.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;


#define IO_HID_EP_LENGTH 64
unsigned char G_io_hid_chunk[IO_HID_EP_LENGTH];

void PCD_ReadPMA(USB_TypeDef  *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);

//#define USB_LOCAL

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/* Exported function prototypes -----------------------------------------------*/
extern USBD_StatusTypeDef USBD_LL_BatteryCharging(USBD_HandleTypeDef *pdev);
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN 1 */
static void SystemClockConfig_Resume(void);
/* USER CODE END 1 */
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state);
extern void SystemClock_Config(void);

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/
/* MSP Init */

void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hpcd->Instance==USB)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

  /* USER CODE END USB_OTG_FS_MspInit 0 */
  
    /**USB_OTG_FS GPIO Configuration    
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP 
    */
    /*
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    */

    /* Peripheral clock enable */
    __USB_CLK_ENABLE();


    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USB_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_IRQn);
  /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

  /* USER CODE END USB_OTG_FS_MspInit 1 */
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB)
  {
  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

  /* USER CODE END USB_OTG_FS_MspDeInit 0 */
    /* Peripheral clock disable */
    __USB_CLK_DISABLE();
  
    /**USB_OTG_FS GPIO Configuration    
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);


    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(USB_IRQn);

  /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

  /* USER CODE END USB_OTG_FS_MspDeInit 1 */
  }
}

/**
  * @brief  Setup Stage callback
  * @param  hpcd: PCD handle
  * @retval None
  */

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
#ifdef USB_DELEG_SE
  if (G_io_apdu_protocol_enabled) {
#endif
    USBD_LL_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
#ifdef USB_DELEG_SE
  }
  else {
    // notify setup to the ST31
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_SETUP;
    
  }
#endif
}

/**
  * @brief  Data Out Stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
#ifdef USB_DELEG_SE
  if (G_io_apdu_protocol_enabled) {
#endif
    PCD_ReadPMA(hpcd->Instance, G_io_hid_chunk, hpcd->OUT_ep[epnum].pmaadress, MIN(sizeof(G_io_hid_chunk), PCD_GET_EP_RX_CNT(hpcd->Instance, epnum)));
    USBD_LL_DataOutStage(hpcd->pData, epnum, G_io_hid_chunk);
#ifdef USB_DELEG_SE
  } 
  else {
    G_io_usb.ep_out |= 1<<epnum;
    // buffer is set when preparing OUT
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_XFER_OUT;
  }
#endif
}

/**
  * @brief  Data In Stage callback..
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
#ifdef USB_DELEG_SE
  if (G_io_apdu_protocol_enabled) {
#endif
    USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
#ifdef USB_DELEG_SE
  }
  else {
    G_io_usb.ep_in |= 1<<epnum;
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_XFER_IN;
  }
#endif
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
#ifdef USB_DELEG_SE
  if (G_io_apdu_protocol_enabled) {
#endif
    USBD_LL_SOF(hpcd->pData);
#ifdef USB_DELEG_SE
  }
  else {
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_SOF;
  }
#endif
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{  
  /*Reset Device*/
  USBD_LL_SetSpeed(hpcd->pData, USBD_SPEED_FULL);  
  USBD_LL_Reset(hpcd->pData);

#ifdef USB_DELEG_SE
  if (!G_io_apdu_protocol_enabled) {
    G_io_seproxyhal_events |= SEPROXYHAL_EVENT_USB_RESET;
  }
#endif
}

/**
  * @brief  Suspend callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
#ifdef USB_DELEG_SE
  /* Inform USB library that core enters in suspend Mode */
  USBD_LL_Suspend(hpcd->pData);
#endif
}

/**
  * @brief  Resume callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
#ifdef USB_DELEG_SE
  USBD_LL_Resume(hpcd->pData);
#endif  
  
}

/**
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
#ifdef USB_DELEG_SE
  if (G_io_apdu_protocol_enabled) {
    USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
  }
#endif
}

/**
  * @brief  ISOINIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
#ifdef USB_DELEG_SE
  if (G_io_apdu_protocol_enabled) {
    USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
  }
#endif
}

/**
  * @brief  ConnectCallback callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected(hpcd->pData);
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected(hpcd->pData);
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/
/**
  * @brief  Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Init (USBD_HandleTypeDef *pdev)
{ 
  /* Init USB_IP */
  if (pdev->id == DEVICE_FS) {
    // KTHX
    memset(&hpcd_USB_OTG_FS, 0, sizeof(hpcd_USB_OTG_FS));

    /* enable USB power on Pwrctrl CR2 register */
    /* Link The driver to the stack */	
    hpcd_USB_OTG_FS.pData = pdev;
    pdev->pData = &hpcd_USB_OTG_FS;
    
    hpcd_USB_OTG_FS.Instance = USB;
    hpcd_USB_OTG_FS.Init.dev_endpoints = MAX_USB_BIDIR_ENDPOINTS;
    hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
    hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
    //hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
    //hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE; // disabled for setup rx to occur without a debug stall
    HAL_PCD_Init(&hpcd_USB_OTG_FS);

    // do the minimum for first setup handling (redone upon endpoint configure)
    HAL_PCDEx_PMAConfig(pdev->pData , 0x00 , PCD_SNG_BUF, 0x40);
    HAL_PCDEx_PMAConfig(pdev->pData , 0x80 , PCD_SNG_BUF, 0x80);
  }
  return USBD_OK;
}

/**
  * @brief  De-Initializes the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_DeInit (USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit(pdev->pData);
  return USBD_OK; 
}

/**
  * @brief  Starts the Low Level portion of the Device driver. 
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Start(pdev->pData);
  return USBD_OK; 
}

/**
  * @brief  Stops the Low Level portion of the Device driver.
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Stop (USBD_HandleTypeDef *pdev)
{
  HAL_PCD_Stop(pdev->pData);
  return USBD_OK; 
}

/**
  * @brief  Opens an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_OpenEP  (USBD_HandleTypeDef *pdev, 
                                      uint8_t  ep_addr,                                      
                                      uint8_t  ep_type,
                                      uint16_t ep_mps)
{
  // ensure fifo is configured for this endpoint
  // set pma address before opening
  HAL_PCDEx_PMAConfig(pdev->pData , ep_addr , PCD_SNG_BUF, 0x40+(2*(ep_addr&0x7F)+((ep_addr&0x80)?1:0))*(0x40));

  HAL_PCD_EP_Open(pdev->pData, 
                  ep_addr, 
                  ep_mps, 
                  ep_type);

  
  return USBD_OK; 
}

/**
  * @brief  Closes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_CloseEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  
  HAL_PCD_EP_Close(pdev->pData, ep_addr);
  return USBD_OK; 
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_FlushEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  
  HAL_PCD_EP_Flush(pdev->pData, ep_addr);
  return USBD_OK; 
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_StallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  
  HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
  return USBD_OK; 
}

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_ClearStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  
  HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);  
  return USBD_OK; 
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP (USBD_HandleTypeDef *pdev, uint8_t ep_addr)   
{
  PCD_HandleTypeDef *hpcd = pdev->pData; 
  
  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall; 
  }
  else
  {
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall; 
  }
}
/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_SetUSBAddress (USBD_HandleTypeDef *pdev, uint8_t dev_addr)   
{
  
  HAL_PCD_SetAddress(pdev->pData, dev_addr);
  return USBD_OK; 
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size    
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_Transmit (USBD_HandleTypeDef *pdev, 
                                      uint8_t  ep_addr,                                      
                                      uint8_t  *pbuf,
                                      uint16_t  size)
{

  // wait until packet is sent if the EP is valid
  while (PCD_GET_EP_TX_STATUS(hpcd_USB_OTG_FS.Instance, (ep_addr & 0x7F)) == USB_EP_TX_VALID);

  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
  return USBD_OK;   
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD Status
  */
USBD_StatusTypeDef  USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, 
                                           uint8_t  ep_addr,                                      
                                           uint16_t  size)
{

  HAL_PCD_EP_Receive(pdev->pData, ep_addr, NULL /*G_io_usb.ep_out_buff[ep_addr&0x7F]*/, size);
  return USBD_OK;   
}

/**
  * @brief  Returns the last transfered packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Recived Data Size
  */
uint32_t USBD_LL_GetRxDataSize  (USBD_HandleTypeDef *pdev, uint8_t  ep_addr)  
{
  return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}


/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void  USBD_LL_Delay (uint32_t Delay)
{
  HAL_Delay(Delay);  
}



/**
* @brief Software Device Connection
* @param hpcd: PCD handle
* @param state: connection state (0 : disconnected / 1: connected) 
* @retval None
*/
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state)
{
/* USER CODE BEGIN 6 */
  if (state == 1)
  {
    /* Configure Low Connection State */
	
  }
  else
  {
    /* Configure High Connection State */
   
  } 
/* USER CODE END 6 */
}

/**
  * @brief  Verify if the Battery Charging Detection mode (BCD) is used :
  *         return USBD_OK if true
  *         else return USBD_FAIL if false
  * @param  pdev: Device handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_LL_BatteryCharging(USBD_HandleTypeDef *pdev)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef*)pdev->pData;
  if (hpcd->Init.battery_charging_enable == ENABLE) 
  {
    return USBD_OK;
  }
  else
  {
    return USBD_FAIL;
  }
}

#define CHANNEL_APDU 0
#define CHANNEL_KEYBOARD 1
#define CHANNEL_SPI 2
#define IO_RESET_AFTER_REPLIED 0x80
#define IO_RECEIVE_DATA 0x40
#define IO_RETURN_AFTER_TX 0x20
#define IO_FLAGS 0xF0

typedef enum io_usb_hid_receive_status_e {
  IO_USB_APDU_RESET,
  IO_USB_APDU_MORE_DATA,
  IO_USB_APDU_RECEIVED,
} io_usb_hid_receive_status_t;

typedef void (*io_send_t)(unsigned char* buffer, unsigned short length);

typedef unsigned short (*io_recv_t)(unsigned char* buffer, unsigned short maxlenth);



extern volatile unsigned short G_io_apdu_length;

/**
 *  Ledger Protocol with FIDO injection
 *  HID Report Content
 *  [______________________________]
 *   CCCC TT VVVV.........VV FILL..
 *
 *  All fields are big endian encoded.
 *  CCCC: 2 bytes channel identifier (when multi application are processing).
 *  TT: 1 byte content tag
 *  VVVV..VV: variable length content
 *  FILL..: 00's to fillup the HID report length
 *
 *  LL is at most the length of the HID Report.
 *
 *  Command/Response APDU are split in chunks.
 * 
 *  Filler only allowed at the end of the last hid report of a apdu chain in each direction.
 * 
 *  APDU are using either standard or extended header. up to the application to check the total received length and the lc field
 *
 *  Tags:
 *  Direction:Host>Token T:0x00 V:no  Get protocol version big endian encoded. Replied with a protocol-version. Channel identifier is ignored for this command.
 *  Direction:Token>Host T:0x00 V:yes protocol-version-4-bytes-big-endian. Channel identifier is ignored for this reply.
 *  Direction:Host>Token T:0x01 V:no  Allocate channel. Replied with a channel identifier. Channel identifier is ignored for this command.
 *  Direction:Token>Host T:0x01 V:yes channel-identifier-2-bytes. Channel identifier is ignored for this reply.
 *  Direction:*          T:0x02 V:no  Ping. replied with a ping. Channel identifier is ignored for this command.
 *  NOTSUPPORTED Direction:*          T:0x03 V:no  Abort. replied with an abort if accepted, else not replied.
 *  Direction:*          T:0x05 V=<sequence-idx-U16><seq==0?totallength:NONE><apducontent> APDU (command/response) packet.
 */

volatile unsigned int   G_io_usb_hid_total_length;
volatile unsigned int   G_io_usb_hid_remaining_length;
volatile unsigned int   G_io_usb_hid_sequence_number;
volatile unsigned char* G_io_usb_hid_current_buffer;


void io_usb_hid_init(void) {
  G_io_usb_hid_sequence_number = 0; 
  //G_io_usb_hid_remaining_length = 0; // not really needed
  //G_io_usb_hid_total_length = 0; // not really needed
  //G_io_usb_hid_current_buffer = G_io_apdu_buffer; // not really needed
}

io_usb_hid_receive_status_t io_usb_hid_receive (io_send_t sndfct, unsigned char* buffer, unsigned short l) {
  /*
  // avoid over/under flows
  memset(G_io_hid_chunk, 0, sizeof(G_io_hid_chunk));
  memcpy(G_io_hid_chunk, buffer, MIN(l, sizeof(G_io_hid_chunk)));
  */
  
  // process the chunk content
  switch(G_io_hid_chunk[2]) {
  case 0x05:
    // ensure sequence idx is 0 for the first chunk ! 
    if (G_io_hid_chunk[3] != (G_io_usb_hid_sequence_number>>8) || G_io_hid_chunk[4] != (G_io_usb_hid_sequence_number&0xFF)) {
      // ignore packet
      goto apdu_reset;
    }
    // cid, tag, seq
    l -= 2+1+2;
    
    // append the received chunk to the current command apdu
    if (G_io_usb_hid_sequence_number == 0) {
      /// This is the apdu first chunk
      // total apdu size to receive
      G_io_usb_hid_total_length = (G_io_hid_chunk[5]<<8)+(G_io_hid_chunk[6]&0xFF);
      // check for invalid length encoding (more data in chunk that announced in the total apdu)
      if (G_io_usb_hid_total_length > sizeof(G_io_apdu_buffer)) {
        goto apdu_reset;
      }
      // seq and total length
      l -= 2;
      // compute remaining size to receive
      G_io_usb_hid_remaining_length = G_io_usb_hid_total_length;
      G_io_usb_hid_current_buffer = G_io_apdu_buffer;

      if (l > G_io_usb_hid_remaining_length) {
        l = G_io_usb_hid_remaining_length;
      }
      // copy data
      memcpy(G_io_usb_hid_current_buffer, G_io_hid_chunk+7, l);
    }
    else {
      // check for invalid length encoding (more data in chunk that announced in the total apdu)
      if (l > G_io_usb_hid_remaining_length) {
        l = G_io_usb_hid_remaining_length;
      }

      /// This is a following chunk
      // append content
      memcpy(G_io_usb_hid_current_buffer, G_io_hid_chunk+5, l);
    }
    // factorize (f)
    G_io_usb_hid_current_buffer += l;
    G_io_usb_hid_remaining_length -= l;
    G_io_usb_hid_sequence_number++;
    break;

  }

  // if more data to be received, notify it
  if (G_io_usb_hid_remaining_length) {
    return IO_USB_APDU_MORE_DATA;
  }

  // reset sequence number for next exchange
  io_usb_hid_init();
  return IO_USB_APDU_RECEIVED;

apdu_reset:
  io_usb_hid_init();
  return IO_USB_APDU_RESET;
}

unsigned short io_usb_hid_exchange(io_send_t sndfct, unsigned short sndlength,
                                   unsigned char flags) {
  unsigned char l;

  // perform send
  if (sndlength) {
    G_io_usb_hid_sequence_number = 0; 
    G_io_usb_hid_current_buffer = G_io_apdu_buffer;
    // consume previous command apdu
    G_io_apdu_length = 0;
    while(sndlength) {

      // fill the chunk
      memset(G_io_hid_chunk+2, 0, IO_HID_EP_LENGTH-2);

      // keep the channel identifier
      G_io_hid_chunk[2] = 0x05;
      G_io_hid_chunk[3] = G_io_usb_hid_sequence_number>>8;
      G_io_hid_chunk[4] = G_io_usb_hid_sequence_number;

      if (G_io_usb_hid_sequence_number == 0) {
        l = ((sndlength>IO_HID_EP_LENGTH-7) ? IO_HID_EP_LENGTH-7 : sndlength);
        G_io_hid_chunk[5] = sndlength>>8;
        G_io_hid_chunk[6] = sndlength;
        memcpy(G_io_hid_chunk+7, G_io_usb_hid_current_buffer, l);
        G_io_usb_hid_current_buffer += l;
        sndlength -= l;
        l += 7;
      }
      else {
        l = ((sndlength>IO_HID_EP_LENGTH-5) ? IO_HID_EP_LENGTH-5 : sndlength);
        memcpy(G_io_hid_chunk+5, G_io_usb_hid_current_buffer, l);
        G_io_usb_hid_current_buffer += l;
        sndlength -= l;
        l += 5;
      }
      // prepare next chunk numbering
      G_io_usb_hid_sequence_number++;

      // this is the last chunk to be sent
      if (!sndlength) {
        // prepare for next apdu
        io_usb_hid_init();     
		G_io_apdu_length = 0;   
      }

      // always pad :)
      // send the chunk
      sndfct(G_io_hid_chunk, sizeof(G_io_hid_chunk));
    }
  }

  if (flags & IO_RESET_AFTER_REPLIED) {
    // give some time to reply on the bus
    HAL_Delay(200);
    NVIC_SystemReset();
  }

  if (flags & IO_RETURN_AFTER_TX ) {
    return 0;
  }

  return G_io_apdu_length;
}


extern USBD_HandleTypeDef USBD_Device;
void io_usb_send_apdu_data(unsigned char* buffer, unsigned short length) {
  USBD_LL_Transmit(&USBD_Device, 0x82, buffer, length);
}

int8_t CUSTOM_HID_OutEvent  (uint8_t* hid_report)
{ 
  if (hid_report) {
    // add to the hid transport
    switch(io_usb_hid_receive(io_usb_send_apdu_data, hid_report, hpcd_USB_OTG_FS.OUT_ep[2].xfer_count)) {
      default:
        break;

      case IO_USB_APDU_RECEIVED:
        G_io_apdu_length = G_io_usb_hid_total_length;
        break;
    }
  }
  return (0);
}


unsigned short io_exchange(unsigned char channel_and_flags, unsigned short tx_length) {
  return io_usb_hid_exchange(io_usb_send_apdu_data, tx_length, channel_and_flags);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
