/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeIDE
**
**  Abstract    : Linker script for STM32H7 series
**                2048Kbytes FLASH and 1056Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** Copyright (c) 2022 STMicroelectronics.
** All rights reserved.
**
** This software is licensed under terms that can be found in the LICENSE file
** in the root directory of this software component.
** If no LICENSE file comes with this software, it is provided AS-IS.
**
****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

#include "Core/Src/Cam/omv_boardconfig.h"

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM_D1) + LENGTH(RAM_D1);    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 2048K
  DTCMRAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1 (xrw)   : ORIGIN = 0x24000000, LENGTH = 512K
  RAM_D2 (xrw)   : ORIGIN = 0x30000000, LENGTH = 288K
  RAM_D3 (xrw)   : ORIGIN = 0x38000000, LENGTH = 64K
  ITCMRAM (xrw)  : ORIGIN = 0x00000000, LENGTH = 64K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >FLASH

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);


  



  /* Main framebuffer memory */
  .fb_memory :
  {
    . = ALIGN(4);
    _fb_base = .;
    . += OMV_FB_SIZE;

    _fb_end = .;
    . += OMV_FB_ALLOC_SIZE;

    . = ALIGN(4);
    _fballoc = .;
    . = ALIGN(4);
  } >OMV_FB_MEMORY

  #if defined(OMV_FB_OVERLAY_MEMORY)
  .fb_overlay_memory (NOLOAD) :
  {
    . = ALIGN(4);
    _fballoc_overlay_start = .;
    . = . + OMV_FB_OVERLAY_MEMORY_OFFSET;
    _fballoc_overlay_end = .;
  } >OMV_FB_OVERLAY_MEMORY
  #endif

  /* Misc DMA buffers section */
  .dma_memory (NOLOAD) :
  {
    . = ALIGN(16);
    _line_buf = .;      // Image line buffer.
    . = . + OMV_LINE_BUF_SIZE;

    . = ALIGN(16);
    _msc_buf  = .;      // USB MSC bot data (2K)
    . = . + OMV_MSC_BUF_SIZE;

    . = ALIGN(16);
    _vfs_buf  = .;      // VFS sturct + FATFS file buffer  (around 624 bytes)
    . = . + OMV_VFS_BUF_SIZE;

    . = ALIGN(16);
    _fir_lepton_buf = .; // FIR Lepton Packet Double Buffer (328 bytes)
    . = . + OMV_FIR_LEPTON_BUF_SIZE;

    #if defined(OMV_FFS_BUF_SIZE)
    . = ALIGN(16);
    _micropy_hw_internal_flash_storage_ram_cache_start = .;
    . = . + OMV_FFS_BUF_SIZE;
    _micropy_hw_internal_flash_storage_ram_cache_end = .;
    #endif

    #if !defined(OMV_JPEG_MEMORY)
    . = ALIGN(16);
    _jpeg_buf = .;      // IDE JPEG buffer
    . = . + OMV_JPEG_BUF_SIZE;
    #endif

   . = ALIGN(16);
    *(.dma_buffer)
  } >OMV_DMA_MEMORY

  /* Domain 1 DMA buffers. */
  #if defined(OMV_DMA_MEMORY_D1)
  .d1_dma_memory (NOLOAD) :
  {
   . = ALIGN(16);
    *(.d1_dma_buffer)
  } >OMV_DMA_MEMORY_D1
  #endif

  /* Domain 2 DMA buffers. */
  #if defined(OMV_DMA_MEMORY_D2)
  .d2_dma_memory (NOLOAD) :
  {
   . = ALIGN(16);
    *(.d2_dma_buffer)
  } >OMV_DMA_MEMORY_D2
  #endif

  /* Domain 3 DMA buffers. */
  #if defined(OMV_DMA_MEMORY_D3)
  .d3_dma_memory (NOLOAD) :
  {
   . = ALIGN(16);
    *(.d3_dma_buffer)
  } >OMV_DMA_MEMORY_D3
  #endif





  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM_D1 AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss section */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM_D1

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM_D1

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


