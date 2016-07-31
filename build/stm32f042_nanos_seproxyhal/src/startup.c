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

void Reset_Handler(unsigned int button_press_duration);
void NMI_Handler(void);
void HardFault_Handler(void);
void Handler(void);
extern void SystemInit (void);

extern void main(unsigned int button_press_duration);


extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;
extern unsigned long _estack;

void NMI_Handler(void) __attribute__ ((weak, alias ("Handler")));
void HardFault_Handler(void) __attribute__ ((weak, alias ("Handler")));
void MemManage_Handler(void) __attribute__ ((weak, alias ("Handler")));
void BusFault_Handler(void) __attribute__ ((weak, alias ("Handler")));
void UsageFault_Handler(void) __attribute__ ((weak, alias ("Handler")));
void SVC_Handler(void) __attribute__ ((weak, alias ("Handler"))) ;
void DebugMon_Handler(void) __attribute__ ((weak, alias ("Handler")));
void PendSV_Handler(void) __attribute__ ((weak, alias ("Handler")));
void SysTick_Handler(void) __attribute__ ((weak, alias ("Handler")));
void WWDG_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void PVD_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));

void RTC_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void FLASH_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void RCC_CRS_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void EXTI0_1_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void EXTI2_3_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void EXTI4_15_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void TSC_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel1_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel2_3_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void DMA1_Channel4_5_6_7_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void ADC1_COMP_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void LPTIM1_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void TIM2_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void TIM6_DAC_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void TIM21_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void TIM22_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void I2C1_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void I2C2_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void SPI1_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void SPI2_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void USART1_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void USART2_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void RNG_LPUART1_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void LCD_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));
void USB_IRQHandler(void) __attribute__ ((weak, alias ("Handler")));


void
Reset_Handler(unsigned int button_press_duration)
{
    unsigned long *pulSrc, *pulDest;

/*
    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &_etext;
    for(pulDest = &_data; pulDest < &_edata; )
    {
        *pulDest++ = *pulSrc++;
    }
*/

    // OTO: if not done, then the whole ST hal is pure junk, *Init are definitely well written ...
    //
    // Zero fill the bss segment.
    //
    for(pulDest = &_bss; pulDest < &_ebss; )
    {
        *pulDest++ = 0;
    }    

    //
    // Call the application's entry point.
    //
    main(button_press_duration);
}

void
Handler(void)
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}

__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
  &_estack, // 0
  Reset_Handler,   // 0x4
  NMI_Handler, // 0x8
  HardFault_Handler, // 0xC
  MemManage_Handler, // 0x10
  BusFault_Handler, // 0x14
  UsageFault_Handler, // 0x18
  0, // 0x1C
  0, // 0x20
  0, // 0x24
  0, // 0x28
  SVC_Handler, // 0x2C
  DebugMon_Handler, // 0x30
  0, // 0x34
  PendSV_Handler, // 0x38
  SysTick_Handler, // 0x3C

#ifndef BOOTLOADER_UPGRADE
  WWDG_IRQHandler, // 0x40
  PVD_IRQHandler,                
  RTC_IRQHandler,                
  FLASH_IRQHandler,              
  RCC_CRS_IRQHandler,            
  EXTI0_1_IRQHandler,         
  EXTI2_3_IRQHandler,            
  EXTI4_15_IRQHandler,           
  TSC_IRQHandler,                
  DMA1_Channel1_IRQHandler,      
  DMA1_Channel2_3_IRQHandler,    
  DMA1_Channel4_5_6_7_IRQHandler,
  ADC1_COMP_IRQHandler,          
  LPTIM1_IRQHandler,             
  0,                             
  TIM2_IRQHandler,               
  0,                             
  TIM6_DAC_IRQHandler,           
  0,                             
  0,                             
  TIM21_IRQHandler,              
  0,                             
  TIM22_IRQHandler,              
  I2C1_IRQHandler,               
  I2C2_IRQHandler,               
  SPI1_IRQHandler,               
  SPI2_IRQHandler,               
  USART1_IRQHandler,             
  USART2_IRQHandler,             
  RNG_LPUART1_IRQHandler,        
  LCD_IRQHandler,                
  USB_IRQHandler,         
#endif // BOOTLOADER_UPGRADE       
};

#include "bootsector.h"
bootsector_ram_t G_bootsector __attribute__ ((section(".bootsector_ram")));
