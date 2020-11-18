;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: File: spi.s
;: Company: Sunco Systems Inc.
;: Author: Marlin Unruh
;: Date: 20050509
;: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
;: File Description: spi port related functions
;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#define __ASSEMBLER__ 1
#define __SFR_OFFSET 0
#include <avr/io.h>
#include "../shared/mcp2515.h"

.global _spi_transfer_ ; ok
.global _spi_mcp_reset_ ; ok
.global _spi_mcp_bit_modify_  ; ok
.global _spi_mcp_read_ ; ok
.global _spi_mcp_write_ ; ok
.global _spi_mcp_seq_read_ ; ok
.global _spi_mcp_seq_write_ ; ok

; place the following code in the bootloader area
.section .bootloaderext, "ax", @progbits

;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _spi_transfer_ (SPI FUNCTION)
;:- unsigned char _spi_transfer_(unsigned char)
;:
;: intellection: This was written to be able to continue and not wait
;: for the spi transfer to complete. This is also assuming that no irq
;: can take control of the spi port. With this scheme after a byte is
;: written to the SPDR the next byte can be fetched and waiting for
;: SPSR-SPIF flag to set. The function looks long, but really no
;: penalty because of the write and exit nature. The function will
;: not hang if a byte has never been written, by using the 'for' loop
;: count down. Write collision is also detected and accounted for.
;: Because of the Read then Write, it takes an extra loop thru the
;: routine to get the correct results. This funciton was written with
;: the need for speed.
;: Note:
;: on leaving the the function the 'spi transfer complete flag' will
;: be set and lowered on entering and reading SPSR flag register and
;: SPDR.
;:
;: in: r24
;: out: r24
;: clobbers: r25
;: (hf)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _spi_transfer_
_spi_transfer_:
      push r17
      mov r17, r24 ; save byte to be sent out clear r24 for read
hf3:  ldi r24, 64 ; count of 64
hf1:  sbic SPSR, SPIF ; test SPI transfer complete flag
      rjmp hf2 ; if complete then move on
      dec r24 ; decrement r17
      brne hf1 ; if r1 is zero then continue
hf2:  in r24, SPDR ; get data byte from SPI Data Reg
      out SPDR, r17   ; byte to be sent on the SPI port
      sbic SPSR, WCOL ; write collision
      rjmp hf3 ; collision happened
      pop r17
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _spi_mcp_bit_modify_ (SPI FUNCTION)
;:- void _spi_mcp_bit_modify_(uint8_t addr, uint8_t mask, uint8_t data)
;:
;: in: address(r24), mask(r22), data(r20)
;: out: none
;: clobbers: r1
;: (nc)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _spi_mcp_bit_modify_
_spi_mcp_bit_modify_:
      push r17 ; save r17 on the stack
      push r28 ; save r28 on the stack
      mov r28, r24 ; save r24 in r25 so r24 can be used for passing
      in r17, GICR ; read i/o GICR
      mov r24, r17 ; don't modify to original
      cbr r24, (1<<INT1) ; disable irq1 interrupt from mcp2515
      out GICR, r24 ;
      cbi PORTC, PC2 ; CAN bus CS true
      ldi r24, CAN_BIT_MODIFY ; send instruction byte
      rcall _spi_transfer_ ; do transfer
      mov r24, r28 ; get address byte
      rcall _spi_transfer_ ; do transfer
      mov r24, r22 ; get mask byte
      rcall _spi_transfer_ ; do transfer
      mov r24, r20 ; get data byte
      rcall _spi_transfer_ ; do transfer
nc1:  sbis SPSR, SPIF ; test for transfer complete
      rjmp nc1 ; loop till condition is meet
      sbi PORTC, PC2 ; mcp2515 chip de-selected
      out GICR, r17 ; restore original state
      pop r28 ; restore r28 from the stack
      pop r17 ; restore r17 from the stack
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _spi_mcp_read_ (SPI FUNCTION)
;:- unsigned char _spi_mcp_read(unsigned char address)
;:
;: used for a single byte read of the mcp2510
;:
;: in: (address)r24
;: out: (data)r24
;: clobbers:
;: (kd)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _spi_mcp_read_
_spi_mcp_read_:
      push r17 ; save r17 on the stack
      push r28 ; save r28 on the stack
      mov r28, r24 ; save r24 in r25 so r24 can be used for passing
      in r17, GICR ; read i/o GICR
      mov r24, r17 ; save original in r17
      cbr r24, (1<<INT1) ; disable irq1 interrupt from mcp2515
      out GICR, r24 ;
      cbi PORTC, PC2 ; CAN bus CS true
      ldi r24, CAN_Read ; send instruction byte
      rcall _spi_transfer_ ; do transfer
      mov r24, r28 ; get passed address byte from r25
      rcall _spi_transfer_ ; do transfer
      ldi r24, DUMMY_BYTE ;
      rcall _spi_transfer_ ; do transfer
      ldi r24, DUMMY_BYTE ;
      rcall _spi_transfer_ ; do transfer data is now in r24
kd1:  sbis SPSR, SPIF ; test for transfer complete
      rjmp kd1 ; loop till condition is meet
      sbi PORTC, PC2 ; mcp2515 chip de-selected
      out GICR, r17 ; restore original state
      pop r28 ; restore r28 from the stack
      pop r17 ; restore r17 from the stack
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _spi_mcp_write (SPI FUNCTION)
;:- used for a single byte write to the mcp2510.
;:
;: in: address(r24) data(r22)
;: out: none
;: clobbers: r1
;: (vg)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _spi_mcp_write_
_spi_mcp_write_:
      push r17 ; save r17 on the stack
      push r28 ; save r28 on the stack
      mov r28, r24 ; save r24 in r25 so r24 can be used for passing
      in r17, GICR ; read i/o GICR
      mov r24, r17 ; save original in r17
      cbr r24, (1<<INT1) ; disable irq1 interrupt from mcp2515
      out GICR, r24 ;
      cbi PORTC, PC2 ; CAN bus CS true
      ldi r24, CAN_Write ; send instruction byte
      rcall _spi_transfer_ ; do transfer
      mov r24, r28 ; get passed address byte from r25
      rcall _spi_transfer_ ; do transfer
      mov r24, r22 ; get data byte to transfer
      rcall _spi_transfer_ ; do transfer
vg1:  sbis SPSR, SPIF ; test for transfer complete
      rjmp vg1 ; loop till condition is meet
      sbi PORTC, PC2 ; mcp2515 chip de-selected
      out GICR, r17 ; restore original state
      pop r28 ; restore r28 from the stack
      pop r17 ; restore r17 from the stack
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _spi_mcp_seq_read_ (SPI FUNCTION)
;:- void _spi_mcp_seq_read(uint8_t* buff, uint8_t address, uint8_t count)
;:
;: reads a sequencial string of bytes from the mcp2515 chip
;:
;: in: pointer(r25-r24) address(r22) count(r20)
;: out: none
;: clobbers:
;: (hv)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _spi_mcp_seq_read_
_spi_mcp_seq_read_:
      push r14 ; save r14 on the stack
      push r15 ; save r15 on the stack
      push r16 ; save r16 on the stack
      push r17 ; save r17 on the stack
      push r28 ; save r28 on the stack
      movw r16, r24 ; save pointer
      mov r15, r22 ; save address byte
      mov r28, r20 ; count of bytes to write
      in r17, GICR ; read i/o GICR
      mov r24, r17 ; save original in r1
      cbr r24, (1<<INT1) ; disable irq1 interrupt from mcp2515
      out GICR, r24 ;
      cbi PORTC, PC2 ; CAN bus CS true
      cpi r28, 0x11 ; test count
      brcs hv1 ; branch carry set
      ldi r28, 0x10 ; max count
hv1:  ldi r24, CAN_Read ; instuction
      rcall _spi_transfer_ ; do transfer
      mov r24, r15 ; get the address byte
      rcall _spi_transfer_ ; do transfer
      ldi r24, DUMMY_BYTE ;
      rcall _spi_transfer_ ; do transfer
hv2:  subi r28, 1 ; decrement r28
      cpi r28, 0xFF ;
      breq hv3 ; done so exit loop
      ldi r24, DUMMY_BYTE ;
      rcall _spi_transfer_
      movw r30, r16 ; load Z pointer
      st Z+, r24 ; load data byte
      movw r16, r30 ; save Z pointer
      rjmp hv2;
hv3:  sbis SPSR, SPIF ; test for transfer complete
      rjmp hv3 ; loop till condition is meet
      sbi PORTC, PC2 ; mcp2515 chip de-selected
      out GICR, r17 ; restore original state
      pop r28 ; restore r28 from stack
      pop r17 ; restore r17 from stack
      pop r16 ; restore r16 from stack
      pop r15 ; restore r15 from stack
      pop r14 ; restore r14 from stack
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _spi_mcp_seq_write_ (SPI FUNCTION)
;:- void _spi_mcp_seq_write(uint8_t* buff, uint8_t address, uint8_t count)
;:
;: writes a sequencial string of bytes from the mcp2515 chip
;:
;: in: pointer(r25-r24) address(r22) count(r20)
;: out: none
;: clobbers:
;: (vr)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _spi_mcp_seq_write_
_spi_mcp_seq_write_:
      push r14 ; save r14 on the stack
      push r15 ; save r15 on the stack
      push r16 ; save r16 on the stack
      push r17 ; save r17 on the stack
      push r28 ; save r28 on the stack
      movw r16, r24 ; save pointer
      mov r15, r22 ; save address byte
      mov r28, r20 ; count of bytes to write
      in r17, GICR ; read i/o GICR
      mov r24, r17 ; save original in r1
      cbr r24, (1<<INT1) ; disable irq1 interrupt from mcp2515
      out GICR, r24 ;
      cbi PORTC, PC2 ; CAN bus CS true
      ldi r24, CAN_Write ; instuction
      rcall _spi_transfer_ ; do transfer
      cpi r28, 0x11 ; count
      brcs vr1
      ldi r28, 0x10
vr1:  mov r24, r15 ; get the address byte
vr2:  rcall _spi_transfer_ ; do transfer
      subi r28, 1 ; decrement r28
      cpi r28, 0xFF ;
      breq vr3 ; done so exit loop
      movw r30, r16 ; load Z pointer
      ld r24, Z+ ; load data byte
      movw r16, r30 ; save Z pointer
      rjmp vr2;
vr3:  sbis SPSR, SPIF ; test for transfer complete
      rjmp vr3 ; loop till condition is meet
      sbi PORTC, PC2 ; mcp2515 chip de-selected
      out GICR, r17 ; restore original state
      pop r28 ; restore r28 from stack
      pop r17 ; restore r17 from stack
      pop r16 ; restore r16 from stack
      pop r15 ; restore r15 from stack
      pop r14 ; restore r14 from stack
      ret ; return
.endfunc

