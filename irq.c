//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: File: irq.c
//: Company: Sunco Systems Inc.
//: Author: Marlin Unruh
//: Date: 19990816
//: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
//: File Description: interrupt handler functions
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include "htc.h"
#include "../shared/shared.h"
#include "../shared/mcp2515.h"

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: SIG_INTERRUPT0
//: Prototype: SIGNAL (SIG_INTERRUPT0)
//: Description: handles shutdown request on int0
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
ISR (INT0_vect) {
   INT0_DISABLE; // disable int0
   CLR_GUIDANCE; // disable guidance
   shut_down(); // in _symh.c
} // SIGNAL(SIG_INTERRUPT0) [interrupt]
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: SIG_INTERRUPT1
//: Prototype: SIGNAL (SIG_INTERRUPT1)
//: Description: handles int1 (MCP2515 interrupt serviced)
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
ISR (INT1_vect) {
   uint8_t tmp = spi_mcp_read(CANINTF); // read interrupt source register
   if (tmp & ERRIF) {  // test for communication errors
      mcp_error_handler();
   }
   if (tmp & (RX1IF | RX0IF)) { // RX buffer full
      mcp_get_msg(tmp); // read mcp rx buffer and place in mcu buffer
      if (flag_mcp & 0x0F) {
         flag.int1 = 1; // raise flag
         pdu1_in(); // locate canbus message initialize pointer
         flag.int1 = 0; // clear flag
         return;
      }
   }
} // SIGNAL(SIG_INTERRUPT1) [interrupt]
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: SIG_OUTPUT_COMPARE1A
//: Prototype: SIGNAL (SIG_OUTPUT_COMPARE1A)
//: Description: handles timer one compare a interrupts
//: Arg1: none
//: Returns: none
//: Notes:
//:   times active outputs, shuts off outputs after 120 secs.
//:   times the amount of time the outputs are actual on
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
ISR (TIMER1_COMPA_vect) {
	nop();
} // SIGNAL(SIG_OUTPUT_COMPARE1A) [interrupt]
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: SIG_OUTPUT_COMPARE1B
//: Prototype: SIGNAL (SIG_OUTPUT_COMPARE1B)
//: Description: handles timer one compare on b interrupt
//: Arg1: none
//: Returns: none
//: Notes:
//:   delay, task0, task1, sgnl_delay, comm_cab timers
//:	timer to turn off solenoids if left on too long
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
ISR (TIMER1_COMPB_vect) {
   static uint8_t portd_image, j;
   if (TST_DELAY) // delay bit high?
      if (!(--tmr1.delay)) // is equal?
         CLR_DELAY; // lower flag
   if (TST_TASK0) // used to time tasks
      if (!(--tmr1.task0)) // task0 counter/timer
         CLR_TASK0; // lower flag
   if (TST_TASK1) // used to time tasks
      if (!(--tmr1.task1)) // task1 counter/timer
         CLR_TASK1; // lower flag
   if (TST_SGNL_DELAY) // wand delay timer
      if (!(--tmr1.wand_delay)) // time out done?
         CLR_SGNL_DELAY; // lower flag
   if (TST_COMM_CAB) { // watchdog timer for the cab module
      if (!(--tmr1.comm_cab)) { // time out done?
         CLR_COMM_CAB; // lower flag
      }
   }
   if (TST_COMMS) { // comms flag set by cab to start communication
      if (!(--tmr1.comm_out)) { // can bus pulse timer?
         tmr1.comm_out = 10; // set next timeout (20 * 0.01 = .2 or 5Hz)
         if (++j & 0x01) { // one msg one pass the next msg the next pass
            pdu1_out16( _VAR | _WRITE | _SIZE16, FLG_CSW,
                  _CSW_HTC_MASK, uniw[FLG_CSW].word);
         } else {
            pdu1_out8( _VAR | _WRITE, SNR_READING + AD_SNR2,
                 0xff, unib[SNR_READING + AD_SNR2].byte);
         }
      }
   }
   // time outputs to save solenoids or battery power
   if ((PORTD & 0xC3) != portd_image) {
	   portd_image = (PORTD & 0xC3); // save image of portd
	   tmr1.output = 12000; // delay amount
	   SET_OUTPUT; // raise output flag
   }
   if (TST_OUTPUT) { // time expected output duration
	   if (!(--tmr1.output)) { // time out done?
		   CLR_OUTPUT; // lower flag
		   if (portd_shdw & ((1<<_SOL_A)|(1<<_SOL_B)))
		   portd_shdw |= 0x10;
		   output_x_off(); // turn both solenoids off
	   }
   }
} // SIGNAL(SIG_OUTPUT_COMPARE1B) [interrupt]
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: SIG_COMPARATOR
//: Prototype: SIGNAL (SIG_COMPARATOR)
//: Description: used with PCB hardware as a short circuit over-current
//:   circuit breaker
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
ISR (ANA_COMP_vect) {
   if (PORTD & (1<<_SOL_A)) { // test if solenoid A is active
      output_x_off(); // output off function
      SSW_HIGH_BYTE |= (1<<SOL_SHORT_A); // raise solenoid A shorted error flag
   }
   if (PORTD & (1<<_SOL_B)) { // test if solenoid B is active
      output_x_off(); // output off function
      SSW_HIGH_BYTE |= (1<<SOL_SHORT_B); // raise solenoid B shorted error flag
   }
   SET_FATAL_ERR; // set the fatal error flag
} // analog compare

