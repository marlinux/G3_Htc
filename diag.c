//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: File: diag.c
//: Company: Sunco Systems Inc.
//: Author: Marlin Unruh
//: Date: 19990518
//: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
//: File Description: diagnostic functions for the hitch module
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#include <avr/io.h>
#include <inttypes.h>
#include "htc.h"
#include "../shared/shared.h"

uint8_t diag_voltage_gt(uint8_t channel, uint16_t range);
uint8_t diag_voltage_lt(uint8_t channel, uint16_t range);
uint8_t diag_motion(uint8_t mode);
void diag_ssword(void);
/*
// **** System Status High Byte ****
SSW_HIGH_BYTE   uniw[FLG_SSW].byte[1]
A2D_FAIL        7 // 100 A/D Converter Failer
E2_FAIL         6 // 101 EE Write Failer
LOW_VOLTAGE     5 // 102 Low Voltage ( < 12V Supply)
SNR_SW          4 // 103 No Secondary Wand Sensor or Shorted
SNR_FB          3 // 104 No Feedback Sensor or Shorted
SNR_PW          2 // 105 No Primary Wand Sensor or Shorted
SOL_SHORT_A     1 // 106 Solenoid A Shorted Out
SOL_SHORT_B     0 // 107 Solenoid B Shorted Out
// **** System Status Low Byte ****
SSW_LOW_BYTE   uniw[FLG_SSW].byte[0]
SOL_HIGH_A      7 // 108 High voltage flag
SOL_HIGH_B      6 // 109 High voltage flag
SOL_LOW_A       5 // 110 Low voltage flag
SOL_LOW_B       4 // 111 Low voltage flag
HYDRL_OFF       3 // 112 Hydraulics off error flag
HYDRL_REVERSE   2 // 113 Reversed plugs or Reversed hydraulics
HYDRL_SLOW      1 // 114 Hydraulics too slow
HYDRL_FAST      0 // 115 Hydraulics too fast
*/
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_flags
//: Prototype: void diag_flags(uint8_t flags)
//: Description: this function will preform the selected test as per set flags
//:   then the diagnostic flag register is returned to the cab unit
//: Arg1: flags of systems to test
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void diag_flags(uint8_t flags) {
   wdt_shutdown(); // stop watchdog timer
   if (flags & DIAG_SYSTEM)
      diag_system(); // diagnostics for voltage and a/d converter
   if (flags & DIAG_SENSOR)
      diag_sensor(); // run sensor diagostics
   if (flags & DIAG_SOLENOID)
      diag_solenoid(); // run solenoid diagnostics
   if (flags & DIAG_HYDRL)
      diag_hydrl(); // run hydraulics diagnostics
   diag_ssword();
   pdu1_out8(_VAR | _WRITE, FLG_DIAG, 0x0F, unib[FLG_DIAG].byte);
    ini_wdt(5); // restart watchdog timer
} // diag_flags
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_system
//: Prototype: void diag_system(void)
//: Description: tests a/d converter, and supply voltage
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void diag_system(void) {
   SSW_HIGH_BYTE &= ~((1<<A2D_FAIL) | (1<<LOW_VOLTAGE));
   output_x_off(); // turn off both outputs
   // test for low voltage and a/d converter
   if (ad_read(AD_V_SUPPLY) == 0) { // must be voltage present to get here
      SSW_HIGH_BYTE |= (1<<A2D_FAIL); // set a/d failed
   } else {
      if (diag_voltage_gt(AD_V_SUPPLY, TARGET_12V))
         SSW_HIGH_BYTE |= (1<<LOW_VOLTAGE); // set low voltage flag
   //   if (ad_read(AD_V_SUPPLY) < TARGET_12V)
   }
   diag_ssword(); // set/clear fatal error flag in CSWORD
   unib[FLG_DIAG].byte &= ~DIAG_SYSTEM; // clear flag
} // diag_system
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_voltage_gt
//: Prototype: uint8_t diag_voltage_gt(uint8_t channel, uint16_t range)
//: Description:
//: Arg1: a/d channel to test
//: Arg2: range (level)
//: Returns: 1 if channel voltage is lower than range, else 0
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint8_t diag_voltage_gt(uint8_t channel, uint16_t range) {
   t1_delay(5); // delay before testing to let things settle
   t1_task0(5); // length of test
   while(TST_TASK0) {
      if (ad_read(channel) > range) // if greater then break out
         break;
   }
   if (TST_TASK0) return(0); // if timer is still running then okay
   else return(1); // else !true
} //  diag_voltage_gt
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_voltage_lt
//: Prototype: uint8_t diag_voltage_lt(uint8_t channel, uint16_t range)
//: Description:
//: Arg1: a/d channel to test
//: Arg2: range or (level)
//: Returns: 1 if channel voltage is higher than range, else 0
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint8_t diag_voltage_lt(uint8_t channel, uint16_t range) {
   t1_delay(5); // delay before testing to let things settle
   t1_task0(5); // length of test
   while(TST_TASK0) {
      if (ad_read(channel) < range) // if less than then break out
         break;
   }
   if (TST_TASK0) return(0); // if timer is still running then okay
   else return(1); // else !true
} //  diag_voltage_lt
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_solenoid
//: Prototype: void diag_solenoid(void)
//: Description: runs amp current draw on solenoids and sets error flags
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void diag_solenoid(void) {
	uint8_t tmp = OCR1A; // save original value of OCR1A
	OCR1A = 0xF8;
   SSW_LOW_BYTE &= ~((1<<SOL_HIGH_A) | (1<<SOL_HIGH_B) | (1<<SOL_LOW_A) | (1<<SOL_LOW_B));
   output_x_off(); // turn off both outputs
   // test if solenoid A is present
   if (diag_voltage_lt(AD_AMP_SOLENOID, AMPS_MIN ))
      SSW_LOW_BYTE |= (1<<SOL_HIGH_A); // solenoid always on (bad trans)
   // test if solenoid B is present
   if (diag_voltage_lt(AD_AMP_SOLENOID, AMPS_MIN ))
      SSW_LOW_BYTE |= (1<<SOL_HIGH_B); // solenoid always on (bad trans)
   // test if transistor A is working
   output_a_on();
   if (diag_voltage_gt(AD_AMP_SOLENOID, AMPS_MIN )) {
      // test solenoid A shorted error flag
      if (!(SSW_HIGH_BYTE & (1<<SOL_SHORT_A)))
         // no current flow (solenoid not connected)
         SSW_LOW_BYTE |= (1<<SOL_LOW_A);
   }
   // test if transistor B is working
   output_b_on();
   if (diag_voltage_gt(AD_AMP_SOLENOID, AMPS_MIN )) {
      // test solenoid A shorted error flag
      if (!(SSW_HIGH_BYTE & (1<<SOL_SHORT_B)))
         // no current flow (solenoid not connected)
         SSW_LOW_BYTE |= (1<<SOL_LOW_B);
   }
   output_x_off(); // insure outputs are off
   diag_ssword(); // set/reset fatal error flag in CSWORD
   unib[FLG_DIAG].byte &= ~DIAG_SOLENOID; // clear flag
	OCR1A = tmp; // restore original value of OCR1A
} // diag_solenoid
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_sensor
//: Prototype: void diag_sensor(void)
//: Description: test the hall sensors to determine if connected and working.
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void diag_sensor(void) {
   uint16_t ad_tmp;
   SSW_HIGH_BYTE &= ~((1<<SNR_PW) | (1<<SNR_FB));
   unib[SNR_STAT].byte = 0x07; // set all flags and lower if not true
   t1_delay(10); // 100ms delay to stabilize inputs
   ad_tmp = ad_read(AD_SNR1); // read primary wand sensor
   if ((ad_tmp < 40) || (ad_tmp > 984)) {// is sensor value too low or high
      unib[SNR_STAT].byte &= ~0x01; // sensor present flag
      SSW_HIGH_BYTE |= (1<<SNR_PW); // primary wand sensor error flag
   }
   ad_tmp = ad_read(AD_SNR2); // read feedback sensor
   if ((ad_tmp < 40) || (ad_tmp > 984)) {// is sensor value too low or high
      unib[SNR_STAT].byte &= ~0x02; // sensor present flag
      SSW_HIGH_BYTE |= (1<<SNR_FB); // feedback sensor error flag
   }
   ad_tmp = ad_read(AD_SNR3); // read secondrary wand sensor
   if ((ad_tmp < 40) || (ad_tmp > 984)) { // is sensor value too low or high
      unib[SNR_STAT].byte &= ~0x04; // sensor present flag
      // no error in case 2nd row sensor is
   }
   // lower calibration flag of sensor(s) not present
   unib[SNR_CALIB].byte = (unib[SNR_CALIB].byte & unib[SNR_STAT].byte);
   diag_ssword(); // set/clear fatal error flag in CSWORD
   unib[FLG_DIAG].byte &= ~DIAG_SENSOR; // clear flag
} // diag_sensor
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_hydrl
//: Prototype: void diag_hydrl(void)
//: Description: test for hydra on and reversed condition
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void diag_hydrl(void) {
   // test hydraulics on and if reversed plugs
   // if no errors yet then proceed
   if ((!(SSW_LOW_BYTE & 0xF0)) && (!(SSW_HIGH_BYTE))) {
      SSW_LOW_BYTE &= ~((1<<HYDRL_OFF) | (1<<HYDRL_REVERSE));
      if (diag_motion(0))
         if (diag_motion(1)) // test for hydraulic response
            SSW_LOW_BYTE |= (1<<HYDRL_OFF);// hydraulics off flag
   }
   output_x_off(); // insure outputs are off
   diag_ssword(); // set/reset fatal error flag in CSWORD
   unib[FLG_DIAG].byte &= ~DIAG_HYDRL; // clear flag
} // diag_hydrl
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_motion
//: Prototype: uint8_t  diag_motion(uint8_t mode)
//: Description: detect correct movement of feedback sensor
//: Arg1: mode (select solenoids)
//: Returns: 1 if no motion detected on feedback sensor SNR2
//:          0 if movement was detected
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint8_t  diag_motion(uint8_t mode) {
   uint8_t uiX, uiY;
   uiX =  sensor_read(AD_SNR2);// read feedback sensor
   t1_task1(150); // set timer for 1.5 secs.
   if (mode) {
      output_a_on(); // sol A 'On'
   } else {
      output_b_on(); // sol B 'On'
   }
   // detect motion and direction
   do {
      uiY =  sensor_read(AD_SNR2);
      if (uiY > (uiX + 6)) {
         if (mode) // if sol A 'On'
            break;
         else {
            SSW_LOW_BYTE |= (1<<HYDRL_REVERSE);// hydraulics reversed flag
            break; // correct motion detected
         }
      }
      if (uiY < (uiX - 6)) {
         if (mode) { // if sol A 'On'
            SSW_LOW_BYTE |= (1<<HYDRL_REVERSE);// hydraulics reversed flag
            break; // correct motion detected
         } else
            break;
      }
   } while(TST_TASK1); // task timer still running?
   output_x_off(); // both sol off
   if (!TST_TASK1) // did timeout occur?
      return(1); // no movement detected
   else
      return(0); // movement detected
} // end  diag_motion
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: diag_ssword
//: Prototype: void diag_ssword(void)
//: Description: tests if fatal error flag should be set exclude hydr spd
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void diag_ssword(void) {
   // if any critical or fatal errors detected
   // don't include hydraulic fast/slow error
   if (uniw[FLG_SSW].word & ~((1<<HYDRL_FAST) | (1<<HYDRL_SLOW)))
      SET_FATAL_ERR; // set flag in flag.csb register
   else
      CLR_FATAL_ERR; // clear flag if no error found
} // diag_ssword

