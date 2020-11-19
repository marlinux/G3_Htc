;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: File: specific.s
;: Company: Sunco Systems Inc.
;: Author: Marlin Unruh
;: Date: unknown
;: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
;: File Description: functions specific to the hitch module
;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#define __ASSEMBLER__ 1
#define __SFR_OFFSET 0
#include <avr/io.h>
;
; this is needed to test the version of firmware in the hitch module
;
.section .firmver, "ax", @progbits
.section .firmver
.global _firmware_version_
_firmware_version_:
.byte 2 ; major version
.byte 2 ; minor revision
.byte 0x08 ; high byte
.byte 0xdf ; low byte
; number of days from jan 01, 2000.
; years * 364.25 rounded down add days from first of current year

.section .bootver, "ax", @progbits
.section .bootver
.global _mcp_msg_header_
_mcp_msg_header_:
.byte 0 ; always zero (used by mcp_put function)
.byte 'B' ; content byte
.byte 0 ; priority flags
.byte 'P' ; (P) target address (pc)
.byte 'H' ; (H) sender address (htc)
.byte 0x08 ; should be 6 byte values
.global _bootloader_version_
_bootloader_version_:
.byte 1 ; major version
.byte 0 ; minor revision
.byte 0x08 ; 08 ; high byte
.byte 0xdf ; low byte
; number of days from jan 01, 2000.
; years * 364.25 rounded down add days from first of current year

.global _ini_ddr_port_
.global _ini_spi_port_
.global _led_on_
.global _led_off_
.global _led_toggle_

.section .bootloader, "ax", @progbits
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
;: _ini_ddr_port_
;: void _ini_ddr_port_(void)
;:
;: set up port and data direction registers
;:
;: in: none
;: out: noe
;: clobbers: none
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
.func _ini_ddr_port_
_ini_ddr_port_:
      ldi r24, 0x00
      out PORTA, r24 ;
      out DDRA, r24
      ldi r24, 0x10
      out PORTB, r24
      ldi r24, 0xb3
      out DDRB, r24
      ldi r24, 0xf4
      out PORTC, r24
      ldi r24, 0xcf
      out DDRC, r24
      ldi r24, 0x30
      out PORTD, r24
      ldi r24, 0xf3
      out DDRD, r24
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
;: _ini_spi_port_
;: void _ini_spi_port_(void)
;:
;: enable spi port as master
;:
;: in: none
;: out: none
;: clobber: none
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
.func _ini_spi_port_
_ini_spi_port_:
      ;push r18
      ;in r18, SPSR
      ;ori r18, (1<<SPI2X) ; clock doubler
      ;out SPSR, r18
      ldi r24, ((1<<SPE) | (1<<MSTR)) ; spi enable, master mode
      out SPCR, r24
      ;pop r18
      ret
.endfunc

;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
;: _led_on_
;: void _led_on_(void)
;:
;: toggles port c pin 0. used for debugging during develoment
;:
;: clobbers: none (used registers are pushed and poped)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
.func _led_on_
_led_on_:
      push r24
      in r24, PORTC
      andi r24, ~(1<<PC1)
      out PORTC, r24
      pop r24
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
;: _led_off_
;: void _led_off_(void)
;:
;: toggles port c pin 0. used for debugging during develoment
;:
;: clobbers: none (used registers are pushed and poped)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
.func _led_off_
_led_off_:
      push r24
      in r24, PORTC
      or r24, (1<<PC1)
      out PORTC, r24
      pop r24
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
;: _led_toggle_
;: void _led_toogle_(void)
;:
;: toggles port c pin 0. used for debugging during develoment
;:
;: clobbers: none (used registers are pushed and poped)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
.func _led_toggle_
_led_toggle_:
      push r18
      push r24
      in r24, PORTC
      ldi r18, (1<<PC1)
      eor r24, r18
      out PORTC, r24
      pop r24
      pop r18
      ret
.endfunc

