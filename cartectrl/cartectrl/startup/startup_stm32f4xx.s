/* Startup for STM32F407 Cortex-M4 ARM MCU */
// NOTE: must compile with -x assembler-with-cpp
// $Id: stm32f407vg.S 4610 2013-01-15 16:08:39Z svn $

// Copyright (C)2013, Philip Munts, President, Munts AM Corp.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification,are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
// * Neither the name of Munts AM Corp. nor the names of its contributors may
//   be used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

        .syntax     unified
        .cpu        cortex-m4
        .thumb
        .text

// Export these symbols

        .global        _start
        .global        Reset_Handler
        .global        Default_Handler

// Import these symbols
        .extern     __data_start__
        .extern     __data_load__
        .extern     __data_size__
        .extern     __fastcode_start__
        .extern     __fastcode_load__
        .extern     __fastcode_size__
        .extern     __bss_start__
        .extern     __bss_size__
        .extern     __stack_end__
        .extern     __ctors_start__
        .extern     __ctors_end__
        .extern     main
        .extern     initClock

//=============================================================================

// Use Default_handler for all exceptions and interrupts, unless another
// handler is provided elsewhere.

        .macro      IRQ handler
        .word       \handler
        .weak       \handler
        .set        \handler, Default_Handler
        .endm

//=============================================================================

// Exception vector table--Common to all Cortex-M4
        .section .isr_vector,"a",%progbits
        .type _start, %object
        .size _start, .-_start
_start:
        .word       __stack_end__
        .word       Reset_Handler
        IRQ         NMI_Handler
        IRQ         HardFault_Handler
        IRQ         MemManage_Handler
        IRQ         BusFault_Handler
        IRQ         UsageFault_Handler
        .word       0
        .word       0
        .word       0
        .word       0
        IRQ         SVC_Handler
        IRQ         DebugMon_Handler
        .word       0
        IRQ         PendSV_Handler
        IRQ         SysTick_Handler

// Hardware interrupts specific to the STM32F407

        IRQ         WWDG_IRQHandler
        IRQ         PVD_IRQHandler
        IRQ         TAMP_STAMP_IRQHandler
        IRQ         RTC_WKUP_IRQHandler
        IRQ         FLASH_IRQHandler
        IRQ         RCC_IRQHandler
        IRQ         EXTI0_IRQHandler
        IRQ         EXTI1_IRQHandler
        IRQ         EXTI2_IRQHandler
        IRQ         EXTI3_IRQHandler
        IRQ         EXTI4_IRQHandler
        IRQ         DMA1_Stream0_IRQHandler
        IRQ         DMA1_Stream1_IRQHandler
        IRQ         DMA1_Stream2_IRQHandler
        IRQ         DMA1_Stream3_IRQHandler
        IRQ         DMA1_Stream4_IRQHandler
        IRQ         DMA1_Stream5_IRQHandler
        IRQ         DMA1_Stream6_IRQHandler
        IRQ         ADC_IRQHandler
        IRQ         CAN1_TX_IRQHandler
        IRQ         CAN1_RX0_IRQHandler
        IRQ         CAN1_RX1_IRQHandler
        IRQ         CAN1_SCE_IRQHandler
        IRQ         EXTI9_5_IRQHandler
        IRQ         TIM1_BRK_TIM9_IRQHandler
        IRQ         TIM1_UP_TIM10_IRQHandler
        IRQ         TIM1_TRG_COM_TIM11_IRQHandler
        IRQ         TIM1_CC_IRQHandler
        IRQ         TIM2_IRQHandler
        IRQ         TIM3_IRQHandler
        IRQ         TIM4_IRQHandler
        IRQ         I2C1_EV_IRQHandler
        IRQ         I2C1_ER_IRQHandler
        IRQ         I2C2_EV_IRQHandler
        IRQ         I2C2_ER_IRQHandler
        IRQ         SPI1_IRQHandler
        IRQ         SPI2_IRQHandler
        IRQ         USART1_IRQHandler
        IRQ         USART2_IRQHandler
        IRQ         USART3_IRQHandler
        IRQ         EXTI15_10_IRQHandler
        IRQ         RTC_Alarm_IRQHandler
        IRQ         OTG_FS_WKUP_IRQHandler
        IRQ         TIM8_BRK_TIM12_IRQHandler
        IRQ         TIM8_UP_TIM13_IRQHandler
        IRQ         TIM8_TRG_COM_TIM14_IRQHandler
        IRQ         TIM8_CC_IRQHandler
        IRQ         DMA1_Stream7_IRQHandler
        IRQ         FSMC_IRQHandler
        IRQ         SDIO_IRQHandler
        IRQ         TIM5_IRQHandler
        IRQ         SPI3_IRQHandler
        IRQ         UART4_IRQHandler
        IRQ         UART5_IRQHandler
        IRQ         TIM6_DAC_IRQHandler
        IRQ         TIM7_IRQHandler
        IRQ         DMA2_Stream0_IRQHandler
        IRQ         DMA2_Stream1_IRQHandler
        IRQ         DMA2_Stream2_IRQHandler
        IRQ         DMA2_Stream3_IRQHandler
        IRQ         DMA2_Stream4_IRQHandler
        IRQ         ETH_IRQHandler
        IRQ         ETH_WKUP_IRQHandler
        IRQ         CAN2_TX_IRQHandler
        IRQ         CAN2_RX0_IRQHandler
        IRQ         CAN2_RX1_IRQHandler
        IRQ         CAN2_SCE_IRQHandler
        IRQ         OTG_FS_IRQHandler
        IRQ         DMA2_Stream5_IRQHandler
        IRQ         DMA2_Stream6_IRQHandler
        IRQ         DMA2_Stream7_IRQHandler
        IRQ         USART6_IRQHandler
        IRQ         I2C3_EV_IRQHandler
        IRQ         I2C3_ER_IRQHandler
        IRQ         OTG_HS_EP1_OUT_IRQHandler
        IRQ         OTG_HS_EP1_IN_IRQHandler
        IRQ         OTG_HS_WKUP_IRQHandler
        IRQ         OTG_HS_IRQHandler
        IRQ         DCMI_IRQHandler
        IRQ         CRYP_IRQHandler
        IRQ         HASH_RNG_IRQHandler
        IRQ         FPU_IRQHandler

//=============================================================================
        .text

// Default exception handler--does nothing but return

        .section .text.Default_Handler,"ax",%progbits
Default_Handler: bx        lr
        .size Default_Handler, .-Default_Handler
//=============================================================================

// Reset vector: Set up environment to call C main()

        .section .text.Reset_Handler
        .weak Reset_Handler
        .type Reset_Handler, %function

Reset_Handler:
// Enable FPU
                // CPACR is located at address 0xE000ED88
                ldr.w   r0, =0xE000ED88
                ldr     r1, [r0]
                // Set bits 20-23 to enable CP10 and CP11 coprocessors
                orr     r1, r1, #(0xF << 20)
                // Write back the modified value to the CPACR
                str     r1, [r0] // wait for store to complete
                dsb
                // reset pipeline now the FPU is enabled
                isb
// Call initClock
                bl          initClock
// Copy initialized data from flash to RAM
                ldr         r0, DATA_START
                ldr         r1, DATA_LOAD
                ldr         r2, DATA_SIZE
                bl          copy_bytes

// Copy fastcode from flash to RAM
                ldr         r0, FASTC_START
                ldr         r1, FASTC_LOAD
                ldr         r2, FASTC_SIZE
                bl          copy_bytes

// Zero uninitialized data (bss)
                ldr         r0, BSS_START
                mov         r1, #0
                ldr         r2, BSS_SIZE
                orrs        r2, r2, #0
                beq         call_ctors

zero_bss:       strb        r1, [r0], #1
                subs        r2, r2, #1
                bgt         zero_bss

// Call C++ constructors.  The compiler and linker together populate the .ctors
// code section with the addresses of the constructor functions.

call_ctors:     ldr         r0, CTORS_START
                ldr         r1, CTORS_END
                subs        r1, r1, r0          // Length of ctors section
                beq         call_main           // Skip if no constructors

ctors_loop:     ldr         r2, [r0], #4        // Load next constructor address
                push        {r0,r1}             // Save registers
                blx         r2                  // Call constructor
                pop         {r0,r1}             // Restore registers
                subs        r1, r1, #4          // Decrement counter
                bgt         ctors_loop          // Repeat until done

// Call main()

call_main:      mov         r0, #0              // argc=0
                mov         r1, #0              // argv=NULL

                bl          main

// main() should never return, but if it does, just do nothing forever.
// Should probably put processor into sleep mode instead.

                b        .

copy_bytes:     orrs        r2, r2, #0
                beq         copy_bytes_x
copy_bytes_l:   ldrb        r3, [r1], #1
                strb        r3, [r0], #1
                subs        r2, r2, #1
                bgt         copy_bytes_l
copy_bytes_x:   bx          lr
        .size Reset_Handler, .-Reset_Handler

//=============================================================================

// These are filled in by the linker

        .align        4
DATA_LOAD:      .word       __data_load__
DATA_START:     .word       __data_start__
DATA_SIZE:      .word       __data_size__
FASTC_LOAD:     .word       __fastcode_load__
FASTC_START:    .word       __fastcode_start__
FASTC_SIZE:     .word       __fastcode_size__
BSS_START:      .word       __bss_start__
BSS_SIZE:       .word       __bss_size__
CTORS_START:    .word       __ctors_start__
CTORS_END:      .word       __ctors_end__

//=============================================================================

// libstdc++ needs this

        .bss
        .align      4
__dso_handle:       .word
        .global     __dso_handle
        .weak       __dso_handle

        .end
