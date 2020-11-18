;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: File: mcp.s
;: Company: Sunco Systems Inc.
;: Author: Marlin Unruh
;: Date: 20050509
;: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
;: File Description: mcp2515 CAN bus chip related functions
;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#define __ASSEMBLER__ 1
#define __SFR_OFFSET 0
#include <avr/io.h>
#include "../shared/mcp2515.h"

.global _mcp_ini_
.global _mcp_reset_
.global _mcp_baud_
.global _mcp_filters_
.global _mcp_ctrl_
.global _mcp_config_
.global _mcp_handler_
.global _mcp_error_handler_
.global _mcp_get_msg_
.global _mcp_put_msg_

.section .bootloaderext, "ax", @progbits
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _mcp_ini_
;: void _mcp_ini_(void)
;:
;: This is not callable from the application area. It is *NOT* included
;: in the gateway section. used for the bootloader only.
;:
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_ini_
_mcp_ini_:
      rcall _ini_spi_port ; set-up the spi for use
      rcall _mcp_reset_ ; reset the mcp2515
      ; set baud rate
      ldi r24, (SJW_1TQ | BRP_1) ;
      ldi r22, (BTLMODE_CNF3 | PHSEG1_6TQ | PRSEG_3TQ)
      ldi r20, (WAKFIL_DISABLED | PHSEG2_6TQ)
      rcall _mcp_baud_
      ; mask
      ldi r24, RXM0SIDH ; rx mask zero
      ldi r22, 0xff ; filter on
      ldi r20, 0xff ; filter on
      rcall _mcp_filters_ ; setup mask 0
      ldi r24, RXM1SIDH ; rx mask one
      ldi r22, 0xff ; filter on
      ldi r20, 0xff ; filter on
      rcall _mcp_filters_ ; setup mask 1
      ; filter (add filters if needed)
      ldi r24, RXF0SIDH ; rx filter zero
      ldi r22, 'H' ; only allow ('H') this controller
      ldi r20, 'P' ; only allow ('P') external controller
      rcall _mcp_filters_ ; setup filter 0
      rcall _mcp_ctrl_
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _mcp_reset_ (MCP2515 LAYER FUNCTION)
;: void spi_mcp_reset(void)
;:
;: intellection: sends the reset command to the mcp2510.
;: Reset places the mcp2510 into configuration mode, and re-intializes
;: the registers to the default values.  Initialization should follow
;: a reset. This function disables global interrupts, but returns the
;: global interrupt flag back to its original state.
;:
;: in: none
;: out: none
;: clobbers: r24
;: (rm)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_reset_
_mcp_reset_:
      push r17 ; save r17 on the stack
      in r17, SREG ; save status register
      cli ; disable global interrupts
      cbi PORTC, PC2 ; CAN bus CS true
      ldi r24, CAN_RESET ; mcp2515 reset command
      rcall _spi_transfer_ ; call spi transfer function
rm1:  sbis SPSR, SPIF ; test for transfer complete
      rjmp rm1 ; loop till condition is meet
      ldi r24, 250 ; value to count down from
      and r24, r24 ; set flags in status register
rm2:  breq rm3
      dec r24 ; count down
      rjmp rm2 ; test if done
rm3:  sbi PORTC, PC2 ; CAN bus CS false
      out SREG, r17 ; restore status register
      pop r17 ; restore r17 from the stack
      ret ;
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _mcp_baud_ (MCP2515 LAYER FUNCTION)
;: void mcp_baud(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3)
;:
;: Pass CNF1, CNF2, CNF3 values as per the prototype to set or modiify
;: the timing register in the mcp2515.
;:
;: in: cnf1(r24), cnf2(r22) cnf3(r20)
;: out: none
;: clobbers:
;: (lj)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_baud_
_mcp_baud_:
      push r16 ; push r16 on the stack
      push r17 ; push r17 on the stack
      push YL ; push YL on the stack
      mov YL, r24 ;
      mov r17, r22 ;
      mov r16, r20 ;
      ldi r24, 0x80 ; configuration mode
      rcall _mcp_config_ ; put mcp2515 into config mode
      mov r22, YL ; get CNF1 data
      ldi r24, CNF1 ; address of CNF1
      rcall _spi_mcp_write_ ;
      mov r22, r17 ; get CNF2 data
      ldi r24, CNF2 ; address of CNF2
      rcall _spi_mcp_write_ ;
      mov r22, r16 ; get CNF3 data
      ldi r24, CNF3 ; address of CNF3
      rcall _spi_mcp_write_ ;
      ldi r24, 0x00 ; normal mode
      rcall _mcp_config_ ; put mcp2515 into normal mode
      pop YL ; restore YL
      pop r17 ; restore r17
      pop r16 ; restore r16
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _mcp_filters_ (MCP2515 LAYER FUNCTION)
;: void inimcp_filters(uint8_t addr, uint8_t sid8, uint8_t sid0)
;:
;: This function only has the ability to set filters and mask on the
;: address portion of the can bus message.
;:
;: in: addr(r24), sid8(r22), sid0(r20)
;: out: none
;: clobbers:
;: (hj)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_filters_
_mcp_filters_:
      push r19 ; save r19
      mov r19, r24 ; address in r19
      ldi r24, 0x80 ; configuration mode
      rcall _mcp_config_ ;
      eor r24, r24 ; clr r24
      sts wr_buff+0, r24 ; SIDH
      ldi r24, 0x08 ; include extended id flag
      sts wr_buff+1, r24 ; SIDL
      sts wr_buff+2, r22 ; EID8
      sts wr_buff+3, r20 ; EID0
      ldi r20,0x04 ; number of bytes to write to mcp2515
      mov r22, r19 ; address of mask or filter in mcp2515
      ldi r24, lo8(wr_buff) ; pass pointer lo
      ldi r25, hi8(wr_buff) ; pass pointer hi
      rcall _spi_mcp_seq_write_ ; call sequencial write function
      ldi r24, 0x00 ; normal mode
      rcall _mcp_config_ ;
      pop r19 ; restore r19
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _mcp_filters_ (MCP2515 LAYER FUNCTION)
;:- void inimcp_filters(uint8_t addr, uint8_t sid8, uint8_t sid0)
;: sets up the control registers for items like generated interrupts
;:
;: in: addr(r24), sid8(r22), sid0(r20)
;: out: none
;: clobbers:
;: (hj)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_ctrl_
_mcp_ctrl_:
      ldi r24, 0x80 ; select configuration mode
      rcall _mcp_config_
      ldi r22, 0x44 ;
      ldi r24, RXB0CTRL ;
      rcall _spi_mcp_write_ ;
      ldi r22, 0x40 ;
      ldi r24, RXB1CTRL ;
      rcall _spi_mcp_write_ ;
      ldi r22, 0x00 ;
      ldi r24, CANINTF ;
      rcall _spi_mcp_write_ ;
      ldi r22, (ERRIE | RX1IE | RX0IE) ;
      ldi r24, CANINTE ;
      rcall _spi_mcp_write_ ;
      ldi r24, 0x00 ;
      rcall _mcp_config_ ;
      in r24, MCUCR ; set-up external INT1
      andi r24, ~(0x0c) ;
      ori r24, 0x08 ; active on falling edge
      out MCUCR, r24 ; write to MCU control register
      in r24, GIFR ; clear any pending interrupts
      ori r24, (1<<INTF1) ;
      out GIFR, r24 ;
      in r24, GICR ; enable external interrupts on int1
      ori r24, (1<<INT1) ;
      out GICR, r24 ;
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _mcp_config_ (MCP2515 LAYER FUNCTION)
;:- void _mcp_config_(uint8_t mode)
;:
;: pass the mode in r24 to place the mcp2515 into that mode. pass 0x00
;: to enter normal mode, or 0x80 to enter configuration mode.
;:
;: in: r24
;: out: none
;: globbers: none
;: (hg)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_config_
_mcp_config_:
      push r20 ; push r20 on the stack
      push r22 ; push r22 on the stack
      push r24 ; push r24 on the stack
      mov r20, r24 ; mode byte
      andi r20, 0xe7 ; allow only valid flags
hg1:  ldi r24, CANCTRL ; instruction byte for rcall to bit modify
      ldi r22, 0xe7 ;
      rcall _spi_mcp_bit_modify_ ; set configuration mode
      ldi r24, CANSTAT ;
      rcall _spi_mcp_read_ ; read the canstatus register to test if mode selected
      andi r24, 0xe7 ; just the bits of interest
      cp r24, r20 ; test if bit is high
      brne hg1 ; loop till config mode is true
      pop r24 ; restore r24
      pop r22 ; restore r22
      pop r20 ; restore r20
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
;: _mcp_handler_
;: uint8_t _mcp_handler_(void)
;:
;: The irq1 flag is used to determine if this function is needed.
;: If there are any messages they will reside in the rd_buffs, test
;: byte zero of the rd_buffs to see if is contains a unprocessed
;: message.
;:
;: in: none
;: out: r24 (0 no message, 1 message recieved)
;: clobber: none
;: (sd)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::-
.func _mcp_handler_
_mcp_handler_:
      push r18 ; place r18 on the stack
      eor r24, r24 ; clear r24
      in r18, GIFR ; first test if mcp2515 has generated an irq
      sbrs r18, INTF1 ; on irq1
      rjmp sd3 ; if not then done
      ldi r24, CANINTF ; read interrupt source register in the mcp2515
      rcall _spi_mcp_read_ ;
      mov r18, r24 ; move results to r18
      andi r24, ERRIF ; test for comm errors
      breq sd1 ; no errors? then
      rcall _mcp_error_handler_ ; handle errors here
sd1:  mov r24, r18 ; get the copy
      andi r24, (RX1IF | RX0IF) ; test for received messages
      breq sd2 ; no messages to process
      rcall _mcp_get_msg_ ;
      sts rd_buff, r1 ; clear to free up the rd buffer
      ldi r24, 0x01 ; return w/ non zero value
sd2:  in r18, GIFR ; general external interrupt flag register
      ori r18, (1<<INTF1) ; clear flag
      out GIFR, r18 ;
sd3:  pop r18 ; restore r18 from the stack
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- _mcp_error_handler_ (MCP2515 LAYER FUNCTION)
;:- void mcp_error_handler(void)
;:
;: reset  the mcp2515 if off bus condition
;: clears receiver 0, and 1 over flow error flags
;: clears error interrupt flag
;:
;: in: none
;: out: none
;: clobbers:
;: (lh)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_error_handler_
_mcp_error_handler_:
      ldi r24, EFLG ; load address of ErrorFlag register in mcp
      rcall _spi_mcp_read_ ; read register
      mov r20, r24 ; make a copy in r20
      mov r18, r24 ; make a copy in r18
      andi r24, 0x3F ; test for bus off errors
      breq lh1 ; then skip init can bus chip
      rcall _mcp_ini_ ; do CAN bus chip reset
      rjmp lh4 ; done for this round
lh1:  sbrs r18, (1<<RX0OVR) ; test receiver zero overflow flag
      rjmp lh2 ; if clear then move on
      ldi r20, 0x00 ; bit(s) to set/clear using the mask
      ldi r22, (1<<RX0OVR) ; mask/bit(s) to set/clear
      rjmp lh3 ;
lh2:  sbrs r20, (1<<RX1OVR) ; test receiver one overflow flag
      rjmp lh4 ; if clear then move on
      ldi r20, 0x00 ; bit(s) to set/clear using the mask
      ldi r22, (1<<RX1OVR) ;mask/bit(s) to set/clear
lh3:  ldi r24, EFLG ; register to modify
      rcall _spi_mcp_bit_modify_ ; do modify
lh4:  ldi r20, 0x00 ; bit(s) to set/clear using the mask
      ldi r22, (1<<ERRIE) ; mask/bit to clear
      ldi r24, CANINTF ; address of the mcp interrupt flags
      rcall _spi_mcp_bit_modify_ ; do modify
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _mcp_get_msg_ (MCP2515 LAYER FUNCTION)
;: void mcp_get_msg(uint8_t rx_flag) rx_flag is the mcp2515 receive
;: buffer that is full.
;:
;: byte 0: contains 1 if holding a valid message, 0 if message has been processed.
;: byte 1: message content
;: byte 2: message priority
;: byte 3: receiver address 'intended target'
;: byte 4: transmitter address 'message owner'
;: byte 5: data byte count w/
;: byte 6:...byte n: data
;: notes: reads port pins to detect which mcp2510 RX buffer needs servicing. fills
;: the rd_buff 'read buffer'. raises an error flag bit 7 of flag.mcp if the buffer
;: is full and has not been processed. the flag.mcp is advanced for each message
;: placed in the array. each message that is processed should subtract one from
;: flag.mcp. buff[0] contains 1 if the message in the buffer has not been
;: processed. the processing function should lower buff[0] to indicate the buffer
;: is available. clears mcp2510 RXBn buffer full flag.
;:
;:
;: (rw)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_get_msg_
_mcp_get_msg_:
      push r16 ; push r16 on the stack
      push r17 ; push r17 on the stack
      push YL ; push YL on the stack
      push YH ; push YH on the stack
      mov r17, r24 ; rx_flag (received message)
      ldi YL, lo8(rd_buff) ; save pointer lo in YL
      ldi YH, hi8(rd_buff) ; save pointer hi in YH
      lds r24, rd_buff ; get byte 0 from the read buffer
      or r24, r24 ; test for non zero (old message not processed)
      breq rw1 ; if zero then buffer is available
      adiw YL,16 ; if not then try the next read buffer
      lds r24, rd_buff+16 ; test next read buffer
      or r24, r24 ; test for non zero (old message not processed)
      breq rw1 ; if zero then buffer is available
      lds r24, flag_mcp ; get flag
      ori r24, 0x40 ; raise buffer overwritten flag
      sts flag_mcp, r24 ; save flag
rw1:  sbrs r17, 1 ; test RX1IF
      rjmp rw2
      ldi r16, RXB1CTRL ; load RXB1 register address
      andi r17, RX1IF ; ensure no other flag are set but the RX1IF
      rjmp rw3
rw2:  ldi r16, RXB0CTRL ; if not RX1IF then must be RXB0CTRL
rw3:  ldi r20, 0x05 ; number of bytes to read +
      mov r22, r16 ; move RXB1 into r24
      inc r22 ; add one to get past the ctrl register
      movw r24, YL ; move the pointer
      adiw r24,1 ;
      rcall _spi_mcp_seq_read_ ; read mcp2515
      ldd r24, Y+2
      mov r18, r24
      andi r18, 0xe0
      lsr r18
      lsr r18
      lsr r18
      andi r24, 0x03
      or r18, r24
      ldd r24, Y+1
      mov r25, r24
      swap r25
      lsr r25
      andi r25, 0x07
      std Y+2, r25
      swap r24
      lsl r24
      andi r24, 0xe0
      or r24, r18
      std Y+1, r24
      ldi r24, 0x01
      st Y, r24
      ldd r20, Y+5
      mov r24, r20
      andi r24, 0x0f
      breq rw4
      andi r20, 0x0f
      subi r16, 0xfa
      mov r22, r16
      movw r24, YL
      adiw r24,6
      rcall _spi_mcp_seq_read_
rw4:  ldi r20, 0x00
      mov r22, r17
      ldi r24, 0x2c
      rcall _spi_mcp_bit_modify_
      lds r24, flag_mcp
      subi r24, 0xff
      sts flag_mcp, r24
      pop YH
      pop YL
      pop r17
      pop r16
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _mcp_put_msg_ (MCP2515 LAYER FUNCTION)
;: uint8_t mcp_put_msg(uint8_t *buff)
;:
;: buff: buffer to be written to mcp2510
;: buffer content arrangement
;: byte 0: used by function, purpose is to sync data on even byte for byte, or
;:    word operations starting with byte 6 or word 3. 'scatch pad in function'
;: byte 1: message content byte
;: byte 2: message priority '0x00 highest priority.. 0x07 lowest priority'
;: byte 3: receiver address 'to whom message is being sent'
;: byte 4: transmitter address 'filled in by function'
;: byte 5: number of data bytes to write
;: byte 6:...n: data bytes
;: note: this function tests the mcp2510 transmit buffers 0..2. if no buffer is
;: available the function raises a error flag in flag.mcp bit 6 then returns w/ 1.
;: the message is placed in the first available buffer. the function splits up the
;: content byte between RXBnSIDH and RXBnSIDL then raises the extended id flag. the
;: message is written along with the data bytes if any to the mcp2510 and the
;: request to transmit flag is set in the TXBnCTRL register. the message should be
;: transmitted when the CAN bus is idle. buff[0] is cleared encase the buffer is to
;: be retransmitted for some reason, e.g. testing.
;: TESTED OK
;:
;: in: r24, r25 (pointer to buffer to write to the mcp2515 CAN
;: out: r24 (1=error:0=ok)
;: clobbers:
;: (wt)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _mcp_put_msg_
_mcp_put_msg_:
      push r17
      push YL
      push YH
      movw YL, r24 ; save r25, r25
      ldi r17, 0x30 ; load mask
wt1: mov r24, r17
      rcall _spi_mcp_read_
      sbrs r24, 3
      rjmp wt3
      andi r24, 0x30
      breq wt2
      ldi r20, 0x00
      ldi r22, 0x38
      mov r24, r17
      rcall _spi_mcp_bit_modify_
      ldi r20, 0x08
      mov r22, r20
      mov r24, r17
      rcall _spi_mcp_bit_modify_
wt2: subi r17, 0xf0
      cpi r17, 0x51
      brlo wt1
      ldi r24, 0x01 ; error flag
      ldi r25, 0x00
      rjmp wt4
wt3: ldd r19, Y+2
      ldd r25, Y+1
      mov r24, r25
      andi r24, 0x1c
      lsl r24
      lsl r24
      lsl r24
      mov r18, r25
      andi r18, 0x03
      or r24, r18
      ori r24, 0x08
      std Y+2, r24
      swap r25
      lsr r25
      andi r25, 0x07
      swap r19
      lsl r19
      andi r19, 0xe0
      or r25, r19
      std Y+1, r25
      ldi r24, 'H' ; hitch module identifier
      std Y+4, r24
      ldd r24, Y+5
      andi r24, 0x0f
      subi r24, 0xfb
      movw ZL, YL
      st Z+, r24
      in r24, GICR ; disable int1 interrupt
      andi r24, ~(1<<INT1)
      out GICR, r24
      mov r24, r17
      subi r24, 0xff
      ld r20, Y
      mov r22, r24
      movw r24, ZL
      rcall _spi_mcp_seq_write_
      ldi r22, 0x08
      mov r24, r17
      rcall _spi_mcp_write_
      in r24, GICR
      ori r24, (1<<INT1)
      out GICR, r24
      st Y, r1 ; zero
      ldi r24, 0x00
      ldi r25, 0x00
wt4:  pop YH
      pop YL
      pop r17
      ret
.endfunc

