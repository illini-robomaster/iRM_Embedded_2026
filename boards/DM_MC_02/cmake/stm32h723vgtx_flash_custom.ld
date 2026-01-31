/*
******************************************************************************
**
**  File        : stm32h723vgtx_flash_custom.ld
**
**  Abstract    : Custom linker script for STM32H723VGTx series
**                Modified to place heap in AXI SRAM for DMA compatibility.
**
**                IMPORTANT: STM32H7 DMA1/DMA2 cannot access DTCM RAM (0x20000000).
**                The heap must be in AXI SRAM (0x24000000) for DMA transfers to work.
**
**  Based on    : STM32CubeMX generated LinkerScript.ld
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of AXI SRAM - stack at top of DMA-accessible RAM */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0x4000;      /* 16KB heap for DMA buffers */
_Min_Stack_Size = 0x1000;     /* 4KB stack */

/* Specify the memory areas */
MEMORY
{
  DTCMRAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 128K   /* Fast RAM, but NOT DMA accessible */
  RAM (xrw)       : ORIGIN = 0x24000000, LENGTH = 128K   /* AXI SRAM - DMA accessible */
  RAM_D2 (xrw)    : ORIGIN = 0x30000000, LENGTH = 32K    /* SRAM1/SRAM2 - DMA accessible */
  RAM_D3 (xrw)    : ORIGIN = 0x38000000, LENGTH = 16K    /* SRAM4 - DMA accessible */
  ITCMRAM (xrw)   : ORIGIN = 0x00000000, LENGTH = 64K    /* Instruction TCM */
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 1024K
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

  .preinit_array :
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

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM AT> FLASH

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
  } >RAM

  /* D2 SRAM section - DMA accessible memory for UART/SPI DMA transfers */
  /* DMA1/DMA2 can access both AXI SRAM (0x24000000) and D2 SRAM (0x30000000) */
  /* Use this section for buffers that MUST be in D2 domain */
  .RAM_D2 (NOLOAD) :
  {
    . = ALIGN(4);
    _sram_d2 = .;
    *(.RAM_D2)
    *(.RAM_D2*)
    . = ALIGN(4);
    _eram_d2 = .;
  } >RAM_D2

  /* User_heap_stack section, used to check that there is enough RAM left */
  /* CRITICAL: Must be in DMA-accessible RAM (AXI SRAM) for STM32H7 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
