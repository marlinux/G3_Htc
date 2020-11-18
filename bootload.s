;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: File: bootload.s
;: Company: Sunco Systems Inc.
;: Author: Marlin Unruh
;: Date: unknown
;: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
;: File Description: bootloader for hitch module
;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#define __ASSEMBLER__ 1
#define __SFR_OFFSET 0
#include <avr/io.h>

#define ROM_ADDR     tmp_buff+0
#define RAM_ADDR     tmp_buff+2
#define CHKSUM       tmp_buff+4
#define TIMER0       tmp_buff+15
#define PF_BYTE      rd_buff+1
#define RD_CNT       rd_buff+5
#define RD_DATA      rd_buff+6
#define WR_CNT       wr_buff+5
#define WR_DATA      wr_buff+6
#define PAGE_CACHE   0x0100

#define temp1 r16
#define temp2 r17
#define looplo r18
#define spmcrval r24

.global _bootloader_
.section .bootloader, "ax", @progbits
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _bootloader_
;: void _bootloader_(void)
;:
;: sets up the stack pointer, clear SRAM, initialize ports
;: initialize mcp chip, send out header, loops for 1.5 seconds
;: for flash re-programming then jumps to the application code
;: at 0x0000. the timer 0 is reset for each message processed.
;:
;: in: none
;: out: none
;: clobber: no relevent
;: (mc)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _bootloader_
_bootloader_:
      eor r1, r1 ; clear r1
      out 0x3f, r1 ; clear status register
      ldi YL, lo8(RAMEND) ; end of SRAM lo
      ldi YH, hi8(RAMEND) ; end of SRAM hi
      out SPH, YH ; load stack pointer SPH
      out SPL, YL ; load stack pointer SPL
      cli ; disable global interrupts
      ; clear sram
      ldi XL, lo8(0x60) ; starting address lo
      ldi XH, hi8(0x60) ; starting addres hi
      rjmp mc2
mc1:  st X+, r1 ; zero out SRAM pointed to by X
mc2:  cp XL, YL ; test for end of SRAM lo
      cpc XH, YH ; test for end of SRAM hi
      brne mc1 ; branch not equal
      ; end clr sram
      rcall _ini_ddr_port_ ; set-up data direction registers and ports
      rcall _mcp_ini_ ; set-up (spi, mcp reset, baud, mask filters, ctrl)
      rcall _msg_header_ ; load generic header data for transfer
      rcall _msg_bootloader_firmware_ver_ ; load firmware and bootloader versions
      rcall _msg_commit_ ; send message on CAN BUS
      ; ::::- setup delay loop ::::-
mc3:  ldi r24, 50 ; count of timer zero overflows to allow
      rcall _t0_delay_ ;
      ;::::- delay loop head ::::-
mc4:  rcall _mcp_handler_ ; test for incoming messages
      or r24, r24 ; set flags
      breq mc5
      rcall _process_message_ ; process CAN bus message
      rjmp mc3 ; re-set the timer for each message processed
      ; ::::- delay loop ::::-
mc5:  ldi r24, 0 ; load zero to test if delay is done
      rcall _t0_delay_ ; test if done
      or r24, r24 ; set status register flags
      breq mc9 ; count down done?
      rjmp mc4
mc9:  rcall _led_off_
      jmp 0 ; when done in bootloader section jump to reset vector
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _write_flash_page_ (SELF PROGRAMMING FUNCTION)
;: void _write_flash_page_(void)
;:
;: -the routine writes one page of data from RAM to Flash
;:  the first data location in RAM is pointed to by the Y pointer
;:  the first data location in Flash is pointed to by the Z pointer
;: -error handling is not included
;: -the routine must be placed inside the boot space
;:  (at least the Do_spm sub routine). Only code inside NRWW section
;:  can be read during self-programming (page erase and page write).
;: -registers used: r0, r1, temp1 (r16), temp2 (r17), looplo (r18),
;:  spmcrval (r24)
;:  storing and restoring of registers is not included in the routine
;:  register usage can be optimized at the expense of code size
;: -It is assumed that either the interrupt table is moved to the
;:  Boot loader section or that the interrupts are disabled.
;: (zv)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _write_page_
_write_page_:
      ldi YL, lo8(PAGE_CACHE) ; load address in Y lo
      ldi YH, hi8(PAGE_CACHE) ; load address in Y hi
      lds ZL, ROM_ADDR ; load ROM address in Z lo
      lds ZH, ROM_ADDR+1 ; load ROM address in Z hi
      ldi spmcrval, (1<<PGERS) | (1<<SPMEN) ; page erase
      rcall _do_spm_
      ldi spmcrval, (1<<RWWSRE) | (1<<SPMEN) ; re-enable the RWW section
      rcall _do_spm_
      ; transfer data from RAM to Flash page buffer
      ldi looplo, lo8(SPM_PAGESIZE) ; init loop variable
zv1:  ld r0, Y+ ; load r0
      ld r1, Y+ ; load r1
      ldi spmcrval, (1<<SPMEN)
      rcall _do_spm_
      adiw ZL, 2
      subi looplo, 2  ; use subi for SPM_PAGESIZE<=256
      brne zv1
      ; execute page write
      subi ZL, lo8(SPM_PAGESIZE)
      sbci ZH, hi8(SPM_PAGESIZE)
      ldi spmcrval, (1<<PGWRT) | (1<<SPMEN)
      rcall _do_spm_
      ; re-enable the RWW section
      ldi spmcrval, (1<<RWWSRE) | (1<<SPMEN)
      rcall _do_spm_
      ; return to RWW section
      ; verify that RWW section is safe to read
zv2:  in temp1, SPMCR
      sbrs temp1, RWWSB ; If RWWSB is set, the RWW section is not
      ; ready yet
      rjmp zv3
      ; re-enable the RWW section
      ldi spmcrval, (1<<RWWSRE) | (1<<SPMEN)
      rcall _do_spm_
      rjmp zv2
zv3:  eor r1, r1 ; clear r1 before exiting
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _do_spm_ (SELF PROGRAMMING FUNCTION)
;: void _do_spm_(uint8_t spmcrval (r20))
;:
;: used to erase or write flash page
;: pass the operation in the spmcrval register
;: Z pointer must be set to the desired page before calling this
;: function.
;:
;: cut and pasted from a amtel document
;: (rw)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _do_spm_
_do_spm_:
rw1:  ; check for previous SPM complete
      in temp1, SPMCR ; load store program memory control register
      sbrc temp1, SPMEN ; test flag
      rjmp rw1 ; loop til flag is cleared
      in temp2, SREG ; save status register
      cli ; disable interrupts
rw2:  ; check that no EEPROM write access is present
      sbic EECR, EEWE ; test EEPROM flags
      rjmp rw2 ; loop til flag is cleared
      out SPMCR, spmcrval ; SPM timed sequence
      spm ; store program memory
      out SREG, temp2 ; restore status register
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _t0_delay_
;: uint8_t _t0_delay_(uint8_t dur)
;:
;: pass the duration of delay in r24. (1/(8Mhz/256/256))*dur
;: 8.192ms * duration
;:
;: in: r24 (0 to test time, non zero to set timer)
;: out: (0 time is up, non zero timer is active)
;: clobbers: none
;: (gh)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _t0_delay_
_t0_delay_:
      push r18
      or r24, r24 ; set flags
      breq gh2 ; set or test pass
      sts TIMER0, r24 ; save value in static RAM
      eor r18, r18 ; zero register
      out TCNT0, r18 ; zero counter register
      ldi r18, 0x04 ; start timer0 clk/256
      out TCCR0, r18 ;
gh1:  in r18, TIFR ; get timer
      sbr r18, TOV0 ; clear timer 0 overflow flag
      out TIFR, r18 ;
      rjmp gh4 ; exit
gh2:  ldi r24, 0x01 ;
      in r18, TIFR ; get timer flag register
      sbrs r18, TOV0 ; read interrupt flag register
      rjmp gh4 ; exit from here
      lds r24, TIMER0 ; get static RAM value
      dec r24
      sts TIMER0, r24 ; save new count
      brne gh1
      out TCCR0, r24 ; finished turn off timer 0 , r24 is zero (done)
gh4:  pop r18
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _process_message_
;: void _process_message_(void)
;:
;: this function is called if the incoming CAN message is of header
;: type. all the bytes are transfered from the CAN buffer to a temp
;: buffer. the bytes are all added including the checksum. when the
;: remainder of the data is transfered and all the bytes added
;: together the low byte should be zero if no errors occured.
;: (kf)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _process_message_
_process_message_:
      lds r24, PF_BYTE ; load PF byte from message buffer
      or r24, r24 ; --- record ---
      brne kf1 ; if not record type then jump
      rcall _record_ ;
      rjmp kf5
kf1:  cpi r24, 0x04 ; --- header message ---
      brne kf2
      ldi r24, 0xFF ; page fill value
      rcall _page_fill_ ; clear unused bytes before starting new page
      rcall _header_ ;
      rjmp kf5
kf2:  cpi r24, 0x02 ; --- checksum message ---
      brne kf3
      rcall _checksum_
      or r24, r24
      brne kf5
      rcall _write_page_ ; write to flash memory
      rcall _led_toggle_ ; toggle led for each page written
      rjmp kf5
kf3:  cpi r24, 0x08 ; --- request return of ROM RAM and CHKSUM data ---
      brne kf4
      rcall _msg_header_
      rcall _msg_body_
      rcall _msg_commit_
      rjmp kf5
kf4:  cpi r24, 0x01 ; --- exit programming and jump to application code ---
      brne kf5
      jmp 0 ; exit reprogramming mode, jump to start of application code
kf5:  ret ;
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _header_
;:
;: this function sets up the ROM and SRAM pointers and clears the checksum
;: the ram low byte pointer is the ROM low byte address anded with 0x7F.
;: this feature enables partial page writes.
;:
;: in: none
;: out: none
;: clobbers: none
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _header_
_header_:
      push r18 ;
      eor r1, r1 ; clear r1
      lds r24, RD_DATA ; load flash address from CAN message low
      mov r18, r24 ; get working copy of r24
      andi r18, 0x80 ; strip the lower 128 (just need ROM block address)
      sts ROM_ADDR, r18 ; save at ROM pointer low
      andi r24, 0x7F ; create low byte pointer for the SRAM buffer
      sts RAM_ADDR, r24 ; save as SRAM pointer
      lds r24, RD_DATA+1 ; load flash address from CAN message high
      sts ROM_ADDR+1, r24 ; save at ROM pointer high
      ldi r24, hi8(PAGE_CACHE) ; load PAGE_CACHE address high
      sts RAM_ADDR+1, r24 ; load in SRAM pointer high
      sts CHKSUM, r1 ; clear checksum scratch pad low
      sts CHKSUM+1, r1 ; clear checksum scratch pad high
      pop r18 ;
      ret ; done
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _record_
;: void _record_(void)
;:
;: message type of record.
;: record type is the application code to be written into the MUC flash
;: memory. the data is written to the location pointed to by the SRAM
;: pointer.
;: (qh)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _record_
_record_:
      eor r1, r1 ; clear r1
      ldi ZL, lo8(RAM_ADDR)
      ldi ZH, hi8(RAM_ADDR)
      ld XL, Z+ ; sram pointer lo
      ld XH, Z+ ; sram pointer hi
      ld YL, Z+ ; checksum lo
      ld YH, Z+ ; checksum hi
      ldi ZL, lo8(RD_CNT) ; get pointer lo
      ldi ZH, hi8(RD_CNT) ; get pointer hi
      ld r18, Z+ ; get byte count
      or r18, r18 ; set status register flags
qh1:  breq qh2 ; test if done
      ld r24, Z+ ; load CAN message byte
      add YL, r24 ; add to checksum
      adc YH, r1 ; add carry bit
      st X+, r24 ; store CAN message byte
      dec r18 ; chalk one off
      rjmp qh1 ; go again
qh2:  ldi ZL, lo8(RAM_ADDR)
      ldi ZH, hi8(RAM_ADDR)
      st Z+, XL ; sram pointer lo
      st Z+, XH ; sram pointer hi
      st Z+, YL ; checksum lo
      st Z+, YH ; checksum hi
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _checksum_
;: uint8_t _checksum_(void)
;:
;: test checksum for errors
;: in: none
;: out: r24 (0:OK, 1:error)
;: clobbers:
;: (hq)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _checksum_
_checksum_:
      lds r24, RD_DATA ; load CAN message checksum lo
      lds r25, RD_DATA+1 ; load CAN message checksum hi
      lds XL, CHKSUM ; load the checksum scrach pad lo
      lds XH, CHKSUM+1 ; load the checksum scrach pad hi
      add r24, XL ; add sent chksum to total of data bytes lo
      adc r25, XH ; add sent chksum to total of data bytes hi
      or r24, r25 ; should be zero if no errors
      breq hq1
      ldi r24, 0x01 ; error flag
hq1:  ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _page_fill_
;: void _page_fill_(char value)
;:
;: writes the entire SRAM buffer with the value passed in r24.
;: clobbers: r24, r25 , X (r26, r27)
;: (np)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _page_fill_
_page_fill_:
      push r18 ; save r18
      mov r18, r24 ; the the value to fill buffer with
      ldi XL, lo8(PAGE_CACHE) ; get SRAM buffer low address
      ldi XH, hi8(PAGE_CACHE) ; get SRAM buffer high address
      ldi r24, lo8(PAGE_CACHE+SPM_PAGESIZE) ; top of page cache
      ldi r25, hi8(PAGE_CACHE+SPM_PAGESIZE) ;
      rjmp np2
np1:  st X+, r18 ; fill value
np2:  cp XL, r24 ; compare
      cpc XH, r25 ; compare with carry
      brne np1 ; again if non zero
      pop r18 ; restore r18
      ret ; return
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _msg_header_
;: void _msg_header_(void)
;:
;: loads the CAN bus message header from program memory and places
;: it in write buffer. this is a fixed CAN message header.
;: [ 0x00, 'B', 0x00, 'Z', 'H', 0x08 ]
;:
;: clobbers: r24, Y(r28, r29), Z(r30, r31)
;: (mh)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _msg_header_
_msg_header_:
      push r18
      ldi r18, 0x06 ; byte count
      ldi YL, lo8(wr_buff) ; YL
      ldi YH, hi8(wr_buff) ; YH
      ldi ZL, lo8(_mcp_msg_header_) ; ZL
      ldi ZH, hi8(_mcp_msg_header_) ; ZH
      or r18, r18 ; set flag
mh1:  breq mh2
      lpm r24, Z+ ; read flash memory using the Z pointer
      st Y+, r24 ; place in RAM buffer using the Y pointer
      dec r18 ; count down
      rjmp mh1 ; start over
mh2:  pop r18
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _msg_fw_version_
;: void _msg_fw_version_(void)
;:
;: load ROM_ADDR, RAM_ADDR, CHKSUM into the write buffer as message
;: to return to the host computer
;:
;: cloggers r24, Y(r28, r29), Z(r30, r31)
;: (yx)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _msg_bootloader_firmware_ver_
_msg_bootloader_firmware_ver_:
      push r18
      ldi r18, 8 ; byte count for firmware version data
      ldi YL, lo8(wr_buff+5) ; load Y pointer to write buffer
      ldi YH, hi8(wr_buff+5)
      ldi ZL, lo8(0) ; load ROM_ADDR lo .... original - ldi ZL, lo8(_firmware_version_)
      ldi ZH, hi8(0x02) ; load ROM_ADDR hi .... original - ldi ZH, hi8(_firmware_version_)
      st Y+, r18 ; CAN message byte count
yx1:  cpi r18, 4 ; four bytes of firmware data
      breq yx2 ; check if done (zeroed)
      lpm r24, Z+ ; load r24 from
      st Y+, r24
      dec r18
      rjmp yx1
yx2:  ldi ZL, lo8(_bootloader_version_) ; load ROM_ADDR lo
      ldi ZH, hi8(_bootloader_version_) ; load ROM_ADDR hi
yx3:  or r18, r18
      breq yx4 ; done?
      lpm r24, Z+ ; load r24 from
      st Y+, r24
      dec r18
      rjmp yx3
yx4:  pop r18
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _msg_body_
;: void _msg_body_(void)
;:
;: load ROM_ADDR, RAM_ADDR, CHKSUM into the write buffer as message
;: to return to the host computer
;:
;: cloggers r24, Y(r28, r29), Z(r30, r31)
;: (hh)
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _msg_body_
_msg_body_:
      push r18
      ldi r18, 6; message data byte count
      ldi YL, lo8(ROM_ADDR) ; load ROM_ADDR lo
      ldi YH, hi8(ROM_ADDR) ; load ROM_ADDR hi
      ldi ZL, lo8(wr_buff+5) ; load Z pointer to write buffer
      ldi ZH, hi8(wr_buff+5)
      st Z+, r18 ; CAN message byte count
      or r18, r18 ; set status register flags
hh1:  breq hh2 ; check if done (zeroed)
      ld r24, Y+ ; load r24 from
      st Z+, r24
      dec r18
      rjmp hh1
hh2:  pop r18
      ret
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: _msg_commit_
;: void _msg_commit_(void)
;:
;: loads pointer to the write buffer and calls _mcp_put_msg_
;: clobbers: r24, r25
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func _msg_commit_
_msg_commit_:
      ldi r24, lo8(wr_buff) ; pass buffer pointer
      ldi r25, hi8(wr_buff)
      rcall _mcp_put_msg_ ; put message on the CAN bus
      ret
.endfunc

