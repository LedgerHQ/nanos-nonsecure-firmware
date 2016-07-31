//*****************************************************************************
//
// startup.c - Boot code for Stellaris.
//
// Copyright (c) 2005-2007 Luminary Micro, Inc.  All rights reserved.
// 
// Software License Agreement
// 
// Luminary Micro, Inc. (LMI) is supplying this software for use solely and
// exclusively on LMI's microcontroller products.
// 
// The software is owned by LMI and/or its suppliers, and is protected under
// applicable copyright laws.  All rights are reserved.  Any use in violation
// of the foregoing restrictions may subject the user to criminal sanctions
// under applicable laws, as well as to civil liability for the breach of the
// terms and conditions of this license.
// 
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// LMI SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
// CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 1392 of the Stellaris Peripheral Driver Library.
//
//*****************************************************************************

#include "stm32f0xx.h"


#include "bootsector.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void Handler(void);

//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern void main(void);


//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;
extern unsigned long _estack;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied main() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
__attribute__ ((section(".entry")))
void
Reset_Handler(void)
{
    unsigned long *pulSrc, *pulDest;

    // Copy the data segment initializers from flash to SRAM.
    //
    pulSrc = &_etext;
    for(pulDest = &_data; pulDest < &_edata; )
    {
        *pulDest++ = *pulSrc++;
    }

    // Zero fill the bss segment.
    //
    for(pulDest = &_bss; pulDest < &_ebss; )
    {
        *pulDest++ = 0;
    }    

    // Call the application's entry point.
    //
    main();
}

//*****************************************************************************
// 
// Forward declaration for weak symbols of ISR, to enable overloading on
// need by HALs
// By default, if not defined, the ISR is mapped on default 'Handler'
//
//*****************************************************************************
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
Handler(void)
{
  while(1) {}
}

__attribute__ ((section(".isr")))
void (* const isr[])(void) =
{
  (main_t)&_estack, // 0
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
};
