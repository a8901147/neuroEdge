/* Minimal linker script for STM32F401RCT6: 256KB Flash @ 0x08000000, 64KB SRAM @ 0x20000000.
 *
 * This specific Black Pill ships pre-flashed with WeAct Studio's HID
 * bootloader occupying the first 16KB of Flash (0x08000000-0x08003FFF) --
 * confirmed by USB enumeration showing "WeAct Studio HID Bootloader"
 * (VID 0x0483 PID 0x572a) and WeAct's own published memory layout. Our
 * application must start at 0x08004000, not 0x08000000, or flashing it
 * would overwrite the bootloader and brick the board's DFU-free recovery
 * path. LENGTH is reduced by 16K to match. */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08004000, LENGTH = 240K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text*)
    *(.rodata*)
  } > FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > RAM AT> FLASH

  .bss :
  {
    _sbss = .;
    *(.bss*)
    *(COMMON)
    _ebss = .;
  } > RAM

  /DISCARD/ :
  {
    *(.ARM.exidx*)
  }
}
