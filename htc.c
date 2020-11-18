//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: File: htc.c
//: Company: Sunco Systems Inc.
//: Author: Marlin Unruh
//: Date: 20010203
//: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
//: File Description: master file for the hitch module program
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#include <avr/io.h>
#include <inttypes.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#define master_header
#include "htc.h"
#include "../shared/shared.h"
#include "../shared/mcp2515.h"

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: main
//: Prototype: int main(void)
//: Description: main hitch module function
//: Arg1: none
//: Returns: 0
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
int main(void) {
   reset(); // run reset function
//   ini_wdt(5); // set watchdog timer for 0.49s timeout
   while(1) {
      while(TST_COMM_CAB) { // infinite loop
         wdr(); // clear watchdog timer
         if(unib[FLG_DIAG].byte & 0x0F) { // test diagnostics flags
            diag_flags(unib[FLG_DIAG].byte); // run diagnostics functions
         } else if(TST_SNR_CALI) { // test calibration mode flags
            sensor_calibrate(); // do sensor calibrate function
         } else if(TST_GUIDANCE) { // test guidance requested flag
            run_module();// then do run module
         }
      }
      if(!(TST_CAB_RESET)) { // test if cab was ever present or not
         stand_alone(); // if not then run as stand-along
      } else {
         shut_down(); // if cab was present then goto shut_down
      }
   }
   return(0);
} // main
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: debug_flasher
//: Prototype: void debug_flasher(void)
//: Description: flasher function that does not use interrupt timer
//: Arg1: none
//: Returns: none
//: Notes:
//:   place function call where needed in the code to test if the code
//:   is reaching that point
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void debug_flasher(void) {
   uint8_t x = 0;
   uint16_t y = 0;
   FLASH_OFF;
   for(;;) {
      if(++y > 16000) {
         y = 0;
         if(++x & 0x88)
            FLASH_OFF;
         else
            FLASH_ON;
      }
   }
} // debug_flasher
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: reset
//: Prototype: void reset(void)
//: Description: calls all the set-up and ini functions
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void reset(void) {
   ini_device(); // setup ports and registers
   ini_defaults(0); // first time ON ONLY (non forced)
   ini_var(); // copy eeprom to ram variables
   ini_sync(); // copy data to cab module
   output_x_off(); // insure outputs are off
   _SOL_AB_ENABLE; // enable a and b outputs
   CLR_SNR_CALI; // clear sensor calibration flags
   SET_HTC_RESET; // set flag to indicate reset is complete
   tmr1.comm_out = 10;
   t1_task1(600);
   while(!TST_CAB_RESET)
      if(!TST_TASK1)
         break;
} // reset
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ini_device
//: Prototype: void ini_device(void)
//: Description: collection of calls to other initialization functions
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void ini_device(void) {
   cli(); // disable global interrupts
   //stop errant interrupts until set up
   ini_ddr_port(); // data direction and port
   ini_spi_port(); // spi
   ini_canbus(); // initialize the MCP2510 canbus chip
   ini_t1(); // initialize timer1
   // setup irq
   INT0_DISABLE; // disable int0 (used for shut_down)
   // analog compare (solenoid   circuit   breakers)
   ACSR = (ACIS1 | ACIS0); // config analog compare input rising edge
   ACSR |= (1<<ACIE); // enable analog comparator interrupt (electronic fuse)
   sei(); // enable global interrupts
   //all peripherals are now initialised
} // ini_device
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ini_canbus
//: Prototype: void ini_canbus(void)
//: Description: initializes the MCP2515 CANBUS chip
//: Arg1: none
//: Returns: none
//: Notes:
//:   called functions are written in assembly and part of the bootload
//:   section
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void ini_canbus(void) {
   mcp_reset(); // reset mcp2510
   mcp_baud(
         (SJW_1TQ | BRP_1),
         (BTLMODE_CNF3 | PHSEG1_6TQ | PRSEG_3TQ),
         (WAKFIL_DISABLED | PHSEG2_6TQ));
   mcp_filters(RXM0SIDH, 0xff, 0xff); // mask 0 (0xff, 0xff)
   mcp_filters(RXM1SIDH, 0xff, 0xff); // mask 1 (0xff, 0xff)
   mcp_filters(RXF0SIDH, HITCH_MOD, CAB_MOD); // filter 0
   mcp_filters(RXF1SIDH, 0x00, 0x00); // filter 1
   mcp_filters(RXF2SIDH, 0x00, 0x00); // filter 2
   mcp_filters(RXF3SIDH, 0x00, 0x00); // filter 3
   mcp_filters(RXF4SIDH, 0x00, 0x00); // filter 4
   mcp_filters(RXF5SIDH, 0x00, 0x00); // filter 5
   mcp_ctrl();
} // ini_canbus
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ini_t1
//: Prototype: void ini_t1(void)
//: Description:  initialize timer 1 (8MHz / 8 / 10k = 100 1/100 = 0.01)
//: Arg1: none
//: Returns: none
//: Notes:
//:   set up timer 1 to count up to 10,000 then reset and interrupt
//:   compare on b times out at midpoint compare on a resets the counter
//: counter 1A controls the PWM
//: counter 1B controls timing function such as delays, etc.
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void ini_t1(void) {
   TCCR1B = 0x00; //stop timer
   OCR1A = 0xF0; // load register 1A controls PWM
   OCR1B = 0x1388; // load register 1B controls delay timing
   TCCR1A = 0x85; // configure bits for PWM operation
   TCCR1B = 0x0C; // start Timer, frequency = 8MHz clear
   OCIE1B_ENABLE; // enable compare on Timer1 B
} //  ini_t1
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ini_var
//: Prototype: void ini_var(void)
//: Description: read eeprom variables and places them in ram memory
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void ini_var(void) {
   // runtime registers
   uniw[TIME_HRS].word = e2_read(E2_HRS);
   uniw[TIME_SECS].word = e2_read(E2_SECS);
   uniw[SOFTWARE_VER].byte[0] = (__LPM_enhanced__(_firmware_version_ + 1));
   uniw[SOFTWARE_VER].byte[1] = (__LPM_enhanced__(_firmware_version_));
   // misc. registers
   unib[SNR_CALIB].byte = e2_read(E2_SNR_CALIB);
   unib[SNR_POLARITY].byte = e2_read(E2_SNR_POLARITY);
   unib[CTL_FDBK].byte = e2_read(E2_FDBK);
   unib[CTL_BIAS].byte = e2_read(E2_BIAS);
   unib[CTL_WIN_SCF].byte = e2_read(E2_WINSCF);
   unib[CTL_WINDOW].byte = e2_read(E2_WINDOW);
   unib[CTL_SGNL_AVERAGE].byte = e2_read(E2_SGNL_AVERAGE);
   uniw[CTL_SGNL_DELAY].word = e2_read(E2_SGNL_DELAY);
   uniw[SNR_MIN + AD_SNR1].word = e2_read(E2_SNR1_MIN);
   uniw[SNR_MAX + AD_SNR1].word = e2_read(E2_SNR1_MAX);
   uniw[SNR_MIN + AD_SNR2].word = e2_read(E2_SNR2_MIN);
   uniw[SNR_MAX + AD_SNR2].word = e2_read(E2_SNR2_MAX);
   uniw[SNR_MIN + AD_SNR3].word = e2_read(E2_SNR3_MIN);
   uniw[SNR_MAX + AD_SNR3].word = e2_read(E2_SNR3_MAX);
   // sensor's scaling factor to be computed
   sensor_scale_factor(AD_SNR1);
   sensor_scale_factor(AD_SNR2);
   sensor_scale_factor(AD_SNR3);
} //  ini_var
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ini_sync
//: Prototype: void ini_sync(void)
//: Description: synchronize common variables in the hitch and cab controllers.
//: Arg1:
//: Returns:
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void ini_sync(void) {
   uint8_t j=0;
   t1_task0(100); // allow 1.0 second for COMMS flag to set
   while(TST_TASK0) { // test if time is up
      if(TST_COMMS) { // if COMMS flag is set then proceed
         for(j=SNR_POLARITY;j<=CTL_SGNL_AVERAGE;j++) {
            // send 8 bit data to cab module
            pdu1_out8((_VAR | _WRITE), j, 0xFF, unib[j].byte);
         }
         for(j=CTL_SGNL_DELAY;j<=SOFTWARE_VER;j++) {
            // send 16 bit data to cab module
            pdu1_out16((_VAR | _WRITE | _SIZE16), j, 0xFFFF, uniw[j].word);
         }
         break;
      }
   }
} // end ini_sync_cab
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ini_defaults
//: Prototype: void ini_defaults(char mode)
//: Description: initial EEprom system parameters
//: Arg1: mode 0 non forced, 1 forced preload of sensor min and max
//: Returns: none
//: Notes:
//:   loads default values for EEprom prameters. if 'mode' is zero nothing
//:   is done if not needed, non-zero forces new sensor calibration parameters
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void ini_defaults(char mode) {
   if(e2_read(E2_MCU) != 0xA5A5) { // first time on?
      e2_write(E2_HRS, 0); // clear hrs.
      e2_write(E2_SECS, 0); // clear secs.
      e2_write(E2_FDBK,50u); // feedback percentage (10=10%, 100=100%)
      e2_write(E2_BIAS, 0); // bias value
      e2_write(E2_WINSCF, 60u); // window scaling factor 10-10% (was 50%)
      e2_write(E2_WINDOW, 5u); // default window width
      e2_write(E2_SNR_CALIB, 0); // clear calibration status
      e2_write(E2_SNR_POLARITY, 0x05); // clear calibration status
      e2_write(E2_SGNL_AVERAGE, 12u); // default response rate
      e2_write(E2_SGNL_DELAY, 200u); // default wand delay
      e2_write(E2_MCU, 0xA5A5); // eeprom initilized code
      mode = 1;
   }
   if(mode) {
      e2_write(E2_SNR1_MIN, 312u);
      e2_write(E2_SNR1_MAX, 712u);
      e2_write(E2_SNR2_MIN, 312u);
      e2_write(E2_SNR2_MAX, 712u);
      e2_write(E2_SNR3_MIN, 312u);
      e2_write(E2_SNR3_MAX, 712u);
   }
} // ini_defaults
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: stand_alone
//: Prototype: void stand_alone(void)
//: Description: runs if no cab module is detected
//: Arg1: none
//: Returns: none
//: Notes:
//:   Load parameters in the hitch module if no cab module was located.
//:   If there are no critical errors the systems will go into stand-alone
//:   mode of operation.
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void stand_alone(void) {
   wdt_shutdown(); // insure watchdog timer is off
   diag_system(); // test system for low voltage, a/d converter
   diag_solenoid(); // test solenoids
   diag_sensor(); // test for sensors
   if(!TST_FATAL_ERR) { // if no fatal errors then
      FLASH_ON; // turn on led
      _SOL_AB_ENABLE; // enable a and b outputs
      CLR_2ND_SGNL; // select priority wand
      CLR_SGNL_CTL; // insure wand is off
      t1_task1(1000); // set for 10 second timeout
      while(TST_TASK1) // center hitch
          run_module();// then do run module
      FLASH_OFF; // turn led off
      t1_delay(100);
      FLASH_ON; // turn on led
      CLR_2ND_SGNL; // select primary wand
      SET_SGNL_CTL; // enable wand
      while(!TST_FATAL_ERR && !TST_MOTION) { // loop forever, as it were
         run_module(); // do run module
         if(!TST_HTC_RESET) // if cab comms requests a reset
            __asm__ volatile("jmp 0"); // psuedo reset
      }
   }
   shut_down();
} //  stand_alone
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: ShutDown
//: Prototype: void shut_down(void)
//: Description: shutdown related operations
//: Arg1: none
//: Returns: none
//: Notes:
//:   Tests if any of the sensors have touched thier outer limits. If so
//:   then that limit is moved out by 20 points. System run time timer,
//:   and operating parameters are stored in EEprom. Then the cpu goes
//:   into an infinite loop til the power source is removed.
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void shut_down(void) {
   wdt_shutdown(); // deactivate watchdog timer
   // runtime reg.
   e2_write(E2_HRS, uniw[TIME_HRS].word); // save runtime HRS.
   e2_write(E2_SECS, uniw[TIME_SECS].word);// save runtime SECS.
   // sensor registers
   if((flag.min_touch ^ flag.max_touch) & (1<<AD_SNR1)) {
      if(flag.min_touch & (1<<AD_SNR1))
         uniw[SNR_MAX + AD_SNR1].word -= 20;
      else
         uniw[SNR_MIN + AD_SNR1].word += 20;
   }
   if((flag.min_touch ^ flag.max_touch) & (1<<AD_SNR2)) {
      if(flag.min_touch & (1<<AD_SNR2))
         uniw[SNR_MAX + AD_SNR2].word -= 20;
      else
         uniw[SNR_MIN + AD_SNR2].word += 20;
   }
   if((flag.min_touch ^ flag.max_touch) & (1<<AD_SNR3)) {
      if(flag.min_touch & (1<<AD_SNR3))
         uniw[SNR_MAX + AD_SNR3].word -= 20;
      else
         uniw[SNR_MIN + AD_SNR3].word += 20;
   }
   e2_write(E2_SNR1_MIN, uniw[SNR_MIN + AD_SNR1].word);
   e2_write(E2_SNR1_MAX, uniw[SNR_MAX + AD_SNR1].word);
   e2_write(E2_SNR2_MIN, uniw[SNR_MIN + AD_SNR2].word);
   e2_write(E2_SNR2_MAX, uniw[SNR_MAX + AD_SNR2].word);
   e2_write(E2_SNR3_MIN, uniw[SNR_MIN + AD_SNR3].word);
   e2_write(E2_SNR3_MAX, uniw[SNR_MAX + AD_SNR3].word);
   e2_write(E2_SNR_CALIB, unib[SNR_CALIB].byte & 0x07);
   // working registers
   e2_write(E2_FDBK, unib[CTL_FDBK].byte);
   e2_write(E2_BIAS, unib[CTL_BIAS].byte);
   e2_write(E2_WINSCF, unib[CTL_WIN_SCF].byte);
   e2_write(E2_WINDOW, unib[CTL_WINDOW].byte);
   e2_write(E2_SGNL_AVERAGE, unib[CTL_SGNL_AVERAGE].byte);
   e2_write(E2_SGNL_DELAY, uniw[CTL_SGNL_DELAY].word);
   // final shut down duties
   output_x_off(); // turn off both solenoids
   sei(); // enable interrupts again
   ini_canbus(); // re-intialize the can bus chip
   for(;;) { // loop til power drops
      FLASHER; // toggle led fast
      t1_delay(10);
      pdu1_out8( _VAR | _WRITE, 0xAA, 0x00, flag.tmr1b);
      if(TST_COMM_CAB) // test if cab comm are coming in
         break; // if so then break out of for loop
      if(!TST_HTC_RESET) // if cab comms request a reset
         __asm__ volatile("jmp 0"); // psuedo reset
   }
} // shut down function

