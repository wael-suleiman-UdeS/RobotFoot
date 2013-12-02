/* Linker script for STM32F407VG Cortex-M4 ARM MCU */

/* $Id: stm32f407vg.ld 4610 2013-01-15 16:08:39Z svn $ */

/* Copyright (C)2013, Philip Munts, President, Munts AM Corp.			      */
/* All rights reserved.                                                       */
/*                                                                            */
/* Redistribution and use in source and binary forms, with or without         */
/* modification,are permitted provided that the following conditions are met: */
/*                                                                            */
/* * Redistributions of source code must retain the above copyright notice,   */
/*   this list of conditions and the following disclaimer.                    */
/*                                                                            */
/* * Neither the name of Munts AM Corp. nor the names of its contributors may */
/*   be used to endorse or promote products derived from this software without*/
/*   specific prior written permission.                                       */
/*                                                                            */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*/
/* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  */
/* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE */
/* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE  */
/* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        */
/* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF       */
/* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS   */
/* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN    */
/* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)    */
/* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE */
/* POSSIBILITY OF SUCH DAMAGE.                                                */

/* Entry Point */
ENTRY(Reset_Handler)

/* Memory blocks */
/*
 * flashv : vectors are there.
 * flash1 : config sector, 16K
 * flash  : .text, .rodata, etc. are there,
 */
MEMORY
{
  /* flash (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K */
  flashv (rx) : ORIGIN = 0x08000000, LENGTH = 16K
  flash1 (rw) : ORIGIN = 0x08004000, LENGTH = 16K
  flash  (rw) : ORIGIN = 0x08008000, LENGTH = 1024K-32K
  ram (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
  ccm (rwx)   : ORIGIN = 0x10000000, LENGTH = 64K
}

/*--------------------------------------------------------------------------*/
/* Definable constants*/
/*
 * Size of:
 * -C stack
 * -IRQ stack
 * -Fast IRQ stack
 * -Supervisor call stack
 * -Abort stack
 * -Undefined exception stack
 */

C_STACK_SIZE   = 16K;
IRQ_STACK_SIZE = 0K;
FIQ_STACK_SIZE = 0K;
SVC_STACK_SIZE = 0K;
ABT_STACK_SIZE = 0K;
UND_STACK_SIZE = 0K;

__heap_min_size__   = 1K;

/*--------------------------------------------------------------------------*/
/* Useful constants related to memory blocks */
__rom_start__	= ORIGIN(flash);
__rom_size__	= LENGTH(flash);
__rom_end__	    = __rom_start__ + __rom_size__;
__ram_start__	= ORIGIN(ram);
__ram_size__	= LENGTH(ram);
__ram_end__	    = __ram_start__ + __ram_size__;

__stack_size__      =   C_STACK_SIZE + IRQ_STACK_SIZE +
                        FIQ_STACK_SIZE + SVC_STACK_SIZE +
                        ABT_STACK_SIZE + UND_STACK_SIZE;

__stack_end__	= __ram_end__;		/* Top of RAM */
__stack_start__	= __stack_end__ - __stack_size__;

__heap_start__	= __bss_end__;		/* Between bss and stack */
__heap_end__	= __stack_start__;

/* Ensure that the heap is large enough, bail out if it is not */
ASSERT(((__heap_end__ - __heap_min_size__) >= __heap_start__),
    "Insufficent heap size.");

/* Other useful constants. Note that symbols are defined latter. */
/* Vector table size */
__vector_size__         = __vector_end__    - __vector_start__;
__fastcode_size__       = __fastcode_end__  - __fastcode_start__;
__text_size__           = __text_end__      - __text_start__;
__data_size__           = __data_end__      - __data_start__;
__bss_size__            = __bss_end__       - __bss_start__;

/*--------------------------------------------------------------------------*/
SECTIONS
{
    /* The vector table goes first into FLASH */
    /* (Otherwise, it will fail miserably) */
    .isr_vector :
    {
        . = ALIGN(4);
        __vector_start__ = .;
        KEEP(*(.isr_vector)) /* Startup code */
        . = ALIGN(4);
        __vector_end__ = .;
    } >flashv

    .config_sector :
    {
        . = ALIGN(4);
        /* Config */
        *(.configdata)
    } >flash1

    .fastcode : {
        . = ALIGN (4);

        __fastcode_load__  = LOADADDR (.fastcode);
        __fastcode_start__ = .;

        /* Glue code AMR <-> thumb. Normally unused on cortex-M*. */
        *(.glue_7t) *(.glue_7)

        /* __attribute__ ((section (".text.fastcode"))) */
        *(.text.fastcode)
        . = ALIGN (4);
        __fastcode_end__ = .;
    } >ram AT>flash

    .text : {
        . = ALIGN (4);
        __text_start__ = .;
        *(.text)                /* Program code */
        *(.text*)
        *(.rodata)              /* Read only data */
        *(.rodata*)

        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        __ctors_start__ = .;
        KEEP(*(.init_array));	/* C++ constructors */
        KEEP(*(.ctors));		/* C++ constructors */
        __ctors_end__ = .;
        . = ALIGN(16);
        __text_end__ = .;
    } >flash

    .data :  {
        __data_load__   = LOADADDR (.data);
        __data_start__  = .;
        *(.data .data.* .gnu.linkonce.d.*)
        __data_end__    = .;
    } >ram AT> flash

    .bss :
    {
        /* This is used by the startup in order to initialize the .bss section */
        _sbss = .;         /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss .bss* .gnu.linkonce.b.*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end */
        __bss_end__     = _ebss;
    } >ram

/* C++ exception unwinding stuff--needed by some toolchains */

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >flash

  __exidx_start = .;
  .ARM.exidx : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) } >flash
  __exidx_end = .;
  .ARM.attributes 0 : { *(.ARM.attributes) }
}

