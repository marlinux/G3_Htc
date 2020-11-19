//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: File: pph.c
//: Company: Sunco Systems Inc.
//: Author: Marlin Unruh
//: Date: 19990816
//: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
//: File Description: peripheral related functions (a/d, e2, timers, etc.)
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#include <avr/io.h>
#include <inttypes.h>
#include "htc.h"

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ad_read
//: Prototype: uint16_t ad_read(uint8_t ch)
//: Description: reads the analog to digital converter
//: Arg1: channel (0-7)
//: Returns: 16 bit result
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint16_t ad_read(uint8_t ch) {
   uint8_t sreg_tmp = SREG;
   asm("cli"); // disable interrupts
   ADMUX = ch; // select channel
   ADCSRA = 0x84; // enable a/d converter, set clock freq. 4 Mhz/32
   ADCSRA |= (1<<ADSC); // start conversion
   while(!(ADCSRA & (1<<ADIF))); // wait for conversion to complete
   ADCSRA |= (1<<ADIF); // clear interrupt flag
   ADCSRA = 0x00; // turn off a/d converter
   SREG = sreg_tmp; // restore to original value
   return(ADC); // return with the results (ADC)
} // ad_read
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: e2_read
//: Prototype: uint16_t e2_read(uint16_t addr)
//: Description: reads eeprom memory
//: Arg1: address
//: Returns: byte stored at address
//: Notes:
//:   reads eeprom '512 bytes of eeprom memory'
//:   returns word size data
//:   bytes stored low byte first in eeprom memory
//:   address is multiplied by 2
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint16_t e2_read(uint16_t addr) {
   uint16_t tmp;
   addr *= 2; // times to for word
   EEAR = addr; // load word address
   while(EECR & (1<<EEWE)); // wait if eeprom write is in process
   EECR |= (1<<EERE); // set request read cycle
   while(EECR & (1<<EERE)); // wait for EECR->EEWE to go zero
   tmp = EEDR; // get low byte
   EEARL = (EEARL+1); // advance address pointer to high byte
   EECR |= (1<<EERE); // EECR->EERE=1 (request read cycle)
   while(EECR & (1<<EERE)); // wait for EECR->EEWE to go zero
   tmp += (EEDR << 8); // get low byte data
   return(tmp); // return with requested data
} // e2_read function
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: e2_write
//: Prototype: void e2_write(uint16_t addr, uint16_t data)
//: Description: writes eeprom 512 bytes (word size data)
//: Arg1: address
//: Arg2: data
//: Returns: none
//: Notes:
//:   1. test new data against existing data and only writes if altered
//:   2. if the write fails the function raises the 'EEprom Write Failer Flag'.
//:   3. address is multiplied by 2 (word)
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void e2_write(uint16_t addr, uint16_t data) {
   uint8_t sreg_tmp = SREG;
   uint8_t counter = 1;
   addr *= 2; // adjust for word
   EEAR = addr; // set address word
   do {
   // read ee byte first to prevent re-writing if same
      EECR |= (1<<EERE); // EECR->EERE=1 (request read cycle)
      while(EECR & (1<<EERE)); // wait for EECR->EEWE to go zero
      if (EEDR != (char)data) {
         EEDR = (char)data; // load data into eeprom data reg.
         asm("cli"); // disable interrupts
         EECR |= (1<<EEMWE); // EECR->EEMWE=1 (master write enable bit)
         EECR |= (1<<EEWE); // EECR->EEWE=1 (secondary write enable bit)
         while(EECR & (1<<EEWE)); // wait for EECR->EEWE to zero
      }
      EEARL = (EEARL+1); // advance to high byte
      data = data >> 8;
   } while(counter--);
   SREG = sreg_tmp; // restore to original value
} // e2_write function
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: t1_delay
//: Prototype: void t1_delay(uint16_t duration)
//: Description: starts the general purpose delay timer
//: Arg1: duration
//: Returns: none (when the timer duration is up)
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void  t1_delay(uint8_t duration) {
   if (duration) {
      OCIE1A_DISABLE;
      tmr1.delay = duration; // delay amount
      SET_DELAY; // raise delay flag
      OCIE1A_ENABLE;
      while(TST_DELAY);
   }
} //  t1_delay
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: t1_wand
//: Prototype: void t1_wand(uint16_t duration)
//: Description: wand active delay timer
//: Arg1: duration
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void t1_wand(uint16_t duration) {
   if (duration) {
      OCIE1A_DISABLE;
      tmr1.wand_delay = duration; // delay amount
      SET_SGNL_DELAY; // raise output flag
      OCIE1A_ENABLE;
   }
} // t1_wand
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: t1_task0 Timer
//: Prototype: void t1_task0(uint8_t duration)
//: Description: starts the general use task0 timer
//: Arg1: duration value
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void t1_task0(uint8_t duration) {
   if (duration) {
      OCIE1A_DISABLE;
      tmr1.task0 = duration; // delay amount
      SET_TASK0; // raise task0 flag
      OCIE1A_ENABLE;
   }
} // Timer_Task0
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: t1_task1
//: Prototype: void t1_task1(uint16_t duration)
//: Description: starts the general use task1 timer
//: Arg1: duration value
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void t1_task1(uint16_t duration) {
   if (duration) {
      OCIE1A_DISABLE;
      tmr1.task1 = duration; // delay amount
      SET_TASK1; // raise task1 flag
      OCIE1A_ENABLE;
   }
} // t1_task1
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: t1_comm_cab
//: Prototype: void t1_comm_cab(uint8_t duration)
//: Description: starts cab comm watchdog timer
//: Arg1: duration
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void t1_comm_cab(uint8_t duration) {
   if (duration) {
      OCIE1A_DISABLE;
      tmr1.comm_cab = duration; // delay amount
      SET_COMM_CAB; // raise output flag
      OCIE1A_ENABLE;
   }
} // t1_comm_token
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: t1_comm_token
//: Prototype: void t1_comm_token(uint8_t duration)
//: Description: token timer for canbus comm operations
//: Arg1: duration
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void t1_comm_token(uint8_t duration) {
   if (duration) {
      OCIE1A_DISABLE;
      tmr1.comm_token = duration; // delay amount
      SET_COMM_TOKEN; // raise output flag
      OCIE1A_ENABLE;
   }
} // t1_comm_token
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ini_wdt
//: Prototype: void  ini_wdt(uint8_t prescale)
//: Description: start MCU watchdog timer
//: Arg1: prescaler value (0.0.0 - 1.1.1) WDP2-WDP0
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void  ini_wdt(uint8_t prescale) {
   return;
   WDTCR = (prescale & 0x07); // setup prescaler to watchdog timer
   wdr(); // reset to activate prescaler value
   WDTCR = (prescale | 0x08); // start watchdog timer
} //  ini_wdt
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: wdt_shutdown
//: Prototype: void  wdt_shutdown(void)
//: Description: MCU watchdog timer shutdown function
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void  wdt_shutdown(void) {
   return;
   uint8_t tmp = 0x10; // need for speed
   WDTCR = 0x18; // timing is critical do not alter
   WDTCR = tmp;
} //  wdt_shutdown

