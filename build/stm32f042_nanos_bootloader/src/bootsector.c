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
// Declaration of the code entry point (bootsector delegation).
//
//*****************************************************************************
void Reset_Handler(void);
extern unsigned long _estack;

/*************************************************************************************
 * Bootsector variables
 */
bootsector_nvram_t N_bootsector __attribute__ ((section(".bootsector_nvram")));
#ifndef ARM_CORE_HAS_VTOR
bootsector_ram_t G_bootsector __attribute__ ((section(".bootsector_ram")));
#endif // ARM_CORE_HAS_VTOR


/*************************************************************************************
 * Bootsector entry point
 */
__attribute__ ((section(".bootsector_entry")))
void
bootsector_Reset_Handler(void)
{
  // disable interruptions
  __asm volatile("cpsid i");

#ifndef ARM_CORE_HAS_VTOR
  // reset VTOR
  G_bootsector.VTOR = NULL;
#endif // !ARM_CORE_HAS_VTOR

  // branch when bootsector is enabled
  if (N_bootsector.bs_magic == BS_MAGIC) {
    N_bootsector.bs();
  }

  // call the code entry point
  Reset_Handler();
}

/*************************************************************************************
 * Default handler for bootsector handlers
 */
#ifndef ARM_CORE_HAS_VTOR
__attribute__ ((section(".bootsector_handler")))
void
bootsector_Handler(void)
{
  if (G_bootsector.VTOR) {
    // jump to user isr vector
    G_bootsector.VTOR[SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk]();
  }
  else {
    while(1);
  }
}
#endif // !ARM_CORE_HAS_VTOR

/*************************************************************************************
 * Bootsector ISRs
 */
__attribute__ ((section(".bootsector_isr")))
void (* const bootsector_isr[])(void) =
{
  (main_t)&_estack, // 0
  bootsector_Reset_Handler,   // 0x4
#ifndef ARM_CORE_HAS_VTOR
  bootsector_Handler, // 0x8
  bootsector_Handler, // 0xC
  bootsector_Handler, // 0x10
  bootsector_Handler, // 0x14
  bootsector_Handler, // 0x18
  bootsector_Handler, // 0x1C
  bootsector_Handler, // 0x20
  bootsector_Handler, // 0x24
  bootsector_Handler, // 0x28
  bootsector_Handler, // 0x2C
  bootsector_Handler, // 0x30
  bootsector_Handler, // 0x34
  bootsector_Handler, // 0x38
  bootsector_Handler, // 0x3C
  bootsector_Handler, // 0x40
  bootsector_Handler,                
  bootsector_Handler,                
  bootsector_Handler,              
  bootsector_Handler,            
  bootsector_Handler,         
  bootsector_Handler,            
  bootsector_Handler,           
  bootsector_Handler,                
  bootsector_Handler,      
  bootsector_Handler,    
  bootsector_Handler,
  bootsector_Handler,          
  bootsector_Handler,             
  bootsector_Handler,                             
  bootsector_Handler,               
  bootsector_Handler,                             
  bootsector_Handler,           
  bootsector_Handler,                             
  bootsector_Handler,                             
  bootsector_Handler,              
  bootsector_Handler,                             
  bootsector_Handler,              
  bootsector_Handler,               
  bootsector_Handler,               
  bootsector_Handler,               
  bootsector_Handler,               
  bootsector_Handler,             
  bootsector_Handler,             
  bootsector_Handler,        
  bootsector_Handler,                
  bootsector_Handler,   
  // MOD: adjust depending on the chip isr table length             
#endif // !ARM_CORE_HAS_VTOR
};
