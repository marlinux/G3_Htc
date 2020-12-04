//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: File: rmod.c
//: Company: Sunco Systems Inc.
//: Author: Marlin Unruh
//: Date: 19990922
//: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
//: File Description: handles the main functions of guidance control
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#include <avr/io.h>
#include <inttypes.h>
#include "htc.h"
#include "../shared/shared.h"

void Snr_Wand_Parameter(void);
void Snr_Fb_Speed(void);
void A_Solenoid(uint8_t mode);
void B_Solenoid(uint8_t mode);
void vlv_threshold(void);
uint8_t detect_motion(void);

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: run_module
//: Prototype: void run_module(void)
//: Description: main control funtion for the hitch guidance
//: Arg1: none
//: Returns: none
//: Notes:
//:   8 cycles per second
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void run_module(void) {
   int8_t snr_wd, snr_fb;
   int16_t ctl_position;
   if (TST_TASK0) {
      return;
	}
   // constant cycle time for correct speed calculations
   t1_task0(12); // 12 * 0.01 = 0.12 = 8.333Hz
   // main hitch controlling loop
	Snr_Wand_Parameter(); // test wand status
   // test if guidance should be active
   if (TST_SGNL_STAT) { // if wand active flag set
		vlv_threshold_enable = 1;
//		pwm_enable = 1;
	   // scale feedback sensor
      snr_fb = ( sensor_read(AD_SNR2) - SNR_CENTER); // read sensor and make signed
      ctl_position = ((snr_fb * unib[CTL_FDBK].byte) / 100);
      if (!TST_2ND_SGNL) { // which wand is active
         snr_wd = (sensor_read(AD_SNR1) - SNR_CENTER) ;
      } else {
         snr_wd = ( sensor_read(AD_SNR3) - SNR_CENTER) ;
      }
      ctl_position += snr_wd;
      unib[CTL_TARGET].byte = unib[CTL_BIAS].byte;
   } else {
      ctl_position = snr_fb = (sensor_read(AD_SNR2) - SNR_CENTER);
      unib[CTL_TARGET].byte = 0; // wands off feedback sensor target is zero
   }
   ctl_position /= 2; // computed position
   unib[CTL_POSITION].byte = ctl_position;
   // calculate window value
   unib[CTL_WIN_TMP].byte = ((unib[SNR_FB_SPEED].byte * unib[CTL_WIN_SCF].byte) / 100);
   if (unib[CTL_WIN_TMP].byte < unib[CTL_WINDOW].byte) {
      unib[CTL_WIN_TMP].byte = unib[CTL_WINDOW].byte;
	}
   // do solenoid control analyzes
   if (ctl_position < ((int8_t)unib[CTL_TARGET].byte - unib[CTL_WIN_TMP].byte)) {
		
		ctl_position = (ctl_position * 10 / 7); // scale

		OCR1A = ((~(ctl_position)) + unib[VLV_A_OPEN].byte);
      A_Solenoid(1);
	} else {
      A_Solenoid(0);
	}
   if (ctl_position > ((int8_t)unib[CTL_TARGET].byte + unib[CTL_WIN_TMP].byte)) {
		
		ctl_position = (ctl_position * 10 / 7); // scale
		
		OCR1A = ((ctl_position) + unib[VLV_B_OPEN].byte);
      B_Solenoid(1);
	} else {
      B_Solenoid(0);
	}
	//------------------------------------------------------------

	if (!TST_SGNL_STAT) {
		if (vlv_threshold_enable) {
			if (!(portd_shdw & 0xC0)) {
				vlv_threshold(); // run function to find open/closed threshold on PWM valve
				vlv_threshold_enable = 0;
			}
		}
	}

} // run_module
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: sensor_scale_factor
//: Prototype: void sensor_scale_factor(uint8_t sensor)
//: Description: (max - min)[RANGE] * 64 / SNR_RANGE
//: Arg1: sensor value to scale
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void sensor_scale_factor(uint8_t sensor) {
   snr.scf[sensor] =
   (((uniw[SNR_MAX + sensor].word - uniw[SNR_MIN + sensor].word) * 64) / SNR_RANGE);
   if (snr.scf[sensor] % SNR_RANGE)
      snr.scf[sensor]++;
} // sensor_scale_factor
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: sensor_read
//: Prototype: uint8_t sensor_read(uint8_t sensor)
//: Description: read sensor, test min and max
//: Arg1:
//: Returns:
//: Notes:
//:   this function tests the min and max value of the sensor to; 1 test
//:   if the sensor is connected. 2 changes polarity (invert reading) if
//:   needed. calls wand averaging or feedback speed depending on the
//:   sensor.
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint8_t sensor_read(uint8_t sensor) {
   uint16_t snr_value16;
   uint8_t snr_value8;
   uint8_t shifted = 1;
   uint8_t   cnt = sensor;
   while(cnt--)
      shifted = (shifted << 1);
   // read a/d converter on selected sensor channel
   snr_value16 = uniw[SNR_RAW + sensor].word = ad_read(sensor);// get reading
   // test if sensor are connected
   if ((snr_value16 < 40) || (snr_value16 > 984)) { // is sensor value too low or high
      SSW_HIGH_BYTE |= ((1<<SNR_PW) << sensor); // set correct sensor error flag
      CSW_HIGH_BYTE |= _FATAL_ERR; // set fatal error flag
   }
   // test max and min values
   snr_value16 =  sensor_min_max(sensor, snr_value16);
   // scale sensor value
   snr_value16 -= uniw[SNR_MIN + sensor].word; // align with zero
   snr_value16 = ((snr_value16 * 64) / snr.scf[sensor]);
   snr_value8 = snr_value16;
   // test if sensor value needs to be negated
   if (shifted & unib[SNR_POLARITY].byte)
      snr_value8 = (SNR_RANGE - snr_value8);
   snr_value8 += OFFSET;
   unib[SNR_READING + sensor].byte = snr_value8; // save with offset added
   if (sensor != AD_SNR2) // if ~ feedback sensor then average else test speed
      snr_value8 =  sensor_average(snr_value8);
   else
      Snr_Fb_Speed(); // compute speed of sensor
   return(snr_value8);
} // sensor_read
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: sensor_min_max
//: Prototype: uint16_t sensor_min_max(uint8_t sensor, uint16_t snr_value)
//: Description: tests if sensor value is out of the calibrated range
//: Arg1: sensor (0-2)
//: Arg2: a/d sensor reading
//: Returns: tested sensor value
//: Notes:
//:   tests the raw max min values to detect if new calibration is needed
//:   truncates value if out of range
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint16_t sensor_min_max(uint8_t sensor, uint16_t snr_value) {
   uint8_t shift;
   // test against min and max values
   if (snr_value <= 512) {
      if (snr_value < uniw[SNR_MIN + sensor].word) { // test min
         uniw[SNR_MIN + sensor].word = snr_value; // sub 2 and save
         sensor_scale_factor(sensor); // rescale sensor
      }
      if (snr_value < (uniw[SNR_MIN + sensor].word + 5)) {
         shift = (1<<sensor); // shift to sensor position
         flag.min_touch |= shift; // raise flag
      }
   } else {
      if (snr_value > uniw[SNR_MAX + sensor].word) { // test max
         uniw[SNR_MAX + sensor].word = snr_value; // add 2 and save
         sensor_scale_factor(sensor); // rescale sensor
      }
      if (snr_value > (uniw[SNR_MAX + sensor].word - 5)) {
         shift = (1<<sensor); // shift to sensor position
         flag.max_touch |= shift; // raise flag
      }
   }
   return(snr_value);
} //  sensor_min_max
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: Snr_Wand_Parameter
//: Prototype: void Snr_Wand_Parameter(void)
//: Description: test wand control flags and sets flags as needed
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void Snr_Wand_Parameter(void) {
   static uint8_t carbon_copy, toggle_temp, shifter;
   // test mercury switch status
   if (unib[CTL_MERCURY_OPT].byte) {
      SET_MERCURY; // reset flag bit
   } else {
      if (PIND & (1<<PD2))  // read input of mercury switch
         CLR_MERCURY; // reset flag bit
      else
         SET_MERCURY; // set flag bit
   }
   // test if any bits have changed from last lap
   if (carbon_copy != (uniw[FLG_CSW].byte[0] &
            (_SGNL_CTL | _2ND_SGNL | _MERCURY))) {
      SET_SGNL_CHANGE; // set flag that wand status has changed
      t1_wand(uniw[CTL_SGNL_DELAY].word);
      carbon_copy = (uniw[FLG_CSW].byte[0] &
            (_SGNL_CTL | _2ND_SGNL | _MERCURY));
   }
   // test if unit should toggle from one wand to the other
   if (unib[SNR_STAT].byte & 0x04) { // test if second wand is present
      if (TST_MERCURY ^ (toggle_temp & _MERCURY)) {
         if (++shifter > 24) {
            if (!TST_MERCURY) { // if mercury switch is low then toggle 2nd wand
               TGL_2ND_SGNL;
               pdu1_out16((_VAR | _WRITE | _SIZE16), FLG_CSW,
                     0x0002, uniw[FLG_CSW].word);
            }
            shifter = 0;
            toggle_temp = TST_MERCURY;
         }
      } else
         shifter = 0;
   }
   if (TST_SGNL_CTL && TST_MERCURY && !(TST_SGNL_DELAY)) {
      SET_SGNL_STAT; // turn wand status on
   } else {
      CLR_SGNL_STAT; // turn wand status off
   }
} // Snr_Wand_Parameter
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: sensor_average
//: Prototype: uint8_t sensor_average(uint8_t snr_value)
//: Description: wand sensor averaging
//: Arg1: sensor value
//: Returns: averaged value
//: Notes:
//:   if new sensor is averaged then the averaging array is filled w/ the
//:   new value otherwise normal averaging continues
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint8_t sensor_average(uint8_t snr_value) {
   static uint8_t old_value;
   int16_t temp;
   if (TST_SGNL_CHANGE) {
      CLR_SGNL_CHANGE;
      old_value = snr_value;
      return snr_value;
   }
   temp = (snr_value - old_value);
   temp *= unib[CTL_SGNL_AVERAGE].byte;
   temp /= 32;
   old_value += (uint8_t)temp;
   return old_value;
} //  sensor_average
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: Snr_Fb_Speed
//: Prototype: void Snr_Fb_Speed(void)
//: Description: feedback sensor speed calculations
//: Arg1: none
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void Snr_Fb_Speed(void) {
   unib[SNR_FB_SPEED].byte = (unib[SNR_READING + AD_SNR2].byte - unib[SNR_OLD + AD_SNR2].byte);
   unib[SNR_OLD + AD_SNR2].byte = unib[SNR_READING + AD_SNR2].byte;
   // always positive value
   if ((int8_t)unib[SNR_FB_SPEED].byte < 0) // test if negative
      unib[SNR_FB_SPEED].byte = ~unib[SNR_FB_SPEED].byte;
} // Snr_Fb_Speed
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: sensor_calibrate
//: Prototype: void sensor_calibrate(void)
//: Description: sensor calibration function
//: Arg1: none
//: Returns: none
//: Notes:
//:   raise the SET_SNR_CALI flag, also raise which sensor is to be
//:   calibrated bit 2,1,0. remains in this function until sensor value
//:   min max spread is greater than 180, this insures the operator has
//:   performed moving the sensors.
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void sensor_calibrate(void) {
   static uint8_t sensor;
   uint8_t tmp, ocr1a_tmp;
   uint16_t snr_tmp;
	wdt_shutdown();
	ocr1a_tmp = OCR1A; // store OCR1A value on entry
	OCR1A = 204;
   if (!(unib[SNR_CALIB_RQ].byte & 0x40)) { // test if first pass
      output_x_off(); // turn both outputs off if on
      diag_sensor(); // run sensor diagnostics first
      if (unib[SNR_STAT].byte & unib[SNR_CALIB_RQ].byte) {
         sensor = 0;
         tmp = 1;
         while(!(tmp & unib[SNR_CALIB_RQ].byte)) {
            tmp = tmp << 1; // left shift
            sensor++;
         }
         unib[SNR_CALIB].byte &= ~tmp; // clear only the requested sensors flag, retain the rest
         uniw[SNR_MIN + sensor].word = 1024;// greatest possible value
         uniw[SNR_MAX + sensor].word = 0; // least possible value
         unib[SNR_CALIB_RQ].byte = (tmp | 0xC0); // raise flag to prevent running thru here again
      }
   } else {
      snr_tmp = uniw[SNR_RAW + sensor].word = ad_read(sensor);
      // compare against min and max values
      if (snr_tmp > uniw[SNR_MAX + sensor].word) { // test max
         uniw[SNR_MAX + sensor].word = snr_tmp; // set new value
          sensor_scale_factor(sensor);
      }
      if (snr_tmp < uniw[SNR_MIN + sensor].word) { // test min
         uniw[SNR_MIN + sensor].word = snr_tmp; // set new value
          sensor_scale_factor(sensor);
      }
      if ((uniw[SNR_MAX + sensor].word - uniw[SNR_MIN + sensor].word) > 180)
         unib[SNR_CALIB].byte |= (unib[SNR_CALIB_RQ].byte & 0x07);
   }
	OCR1A = ocr1a_tmp; // restore OCR1A value on entry
   ini_wdt(5); // set watchdog timer for 0.49s timeout
} //  sensor_calibrate
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: A_Solenoid
//: Prototype: void A_Solenoid(uint8_t mode)
//: Description: controls A soleniod and tests hydraulic conditions
//: Arg1: mode (0=Off, non-zero = On)
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void A_Solenoid(uint8_t mode) {
   static uint8_t cycle_counter, start_position;
   uint8_t current_position;
   if (mode) {
      if (!(portd_shdw & (1<<_SOL_A))) { // drv A not ON yet?

			if (OCR1A < unib[VLV_A_OPEN].byte) {
				OCR1A = (unib[VLV_A_OPEN].byte + 10);
			}
			
         start_position = unib[SNR_OLD].byte = unib[SNR_READING + AD_SNR2].byte;
         output_a_on(); // solenoid A on routine
      } else {
         if (cycle_counter & 0x80) { // test if counter is < 128
            //output_a_off(); // turn off solenoid A
            return; // then exit routine
         }
         if (++cycle_counter == 8) { // 1 sec.
            if (start_position < (SNR_RANGE - 6)) {
               current_position = unib[SNR_READING + AD_SNR2].byte;
               if (current_position > (start_position + 4)) {
                  CLR_MOTION; // motion detected
                  SSW_LOW_BYTE &= ~(1<<HYDRL_REVERSE); // clear reversed hydraulics
                  SSW_LOW_BYTE &= ~(1<<HYDRL_OFF); // clear hydraulics off
               } else {
                  SET_MOTION; // no motion detected
               }
            }
         }
      }
   } else {
      output_x_off();
   }
} // A_Solenoid
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: B_Solenoid
//: Prototype: void B_Solenoid(uint8_t mode)
//: Description: controls B soleniod and tests hydraulic conditions
//: Arg1: mode (0=Off, non-zero = On)
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void B_Solenoid(uint8_t mode) {
   static uint8_t cycle_counter, start_position;
   uint8_t current_position;
   if (mode) {
		if (!(portd_shdw & (1<<_SOL_B))) { // drv B not ON yet?

			if (OCR1A < unib[VLV_B_OPEN].byte) {
				OCR1A = (unib[VLV_B_OPEN].byte + 10);
			}

         start_position = unib[SNR_OLD].byte = unib[SNR_READING + AD_SNR2].byte;
         output_b_on(); // solenoid B on routine
      } else {
         if (cycle_counter & 0x80) { // test if counter is < 128
            //output_b_off(); // turn off solenoid B
            return; // then exit routine
         }
         if (++cycle_counter == 8) { // 1 sec.
            if (start_position > (0 + 6)) {
               current_position = unib[SNR_READING + AD_SNR2].byte;
               if (current_position < (start_position - 4)) {
                  CLR_MOTION; // motion detected
                  SSW_LOW_BYTE &= ~(1<<HYDRL_REVERSE); // clear reversed hydraulics
                  SSW_LOW_BYTE &= ~(1<<HYDRL_OFF); // clear hydraulics off
               } else {
                  SET_MOTION; // no motion detected
               }
            }
         }
      }
   } else {
      output_b_off(); // solenoid B off
   }
} // B_Solenoid
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: output_a_on
//: Prototype: void output_a_on(void)
//: Description: turns on output a
//: Arg1: none
//: Returns: none
//: Notes:
//:   this should be the only place is this software package that the A
//:   output is turned on
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void output_a_on(void) {
   output_b_off(); // insure output b is off
   PORTD |= (1<<_SOL_A); // turn on output A
   portd_shdw |= (1<<_SOL_A); // used to detect if the physical port pins
                         // are changed by timeout or shorted and shut off
} // output_a_on
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: output_a_off
//: Prototype: void output_a_off(void)
//: Description: turns off output a
//: Arg1: none
//: Returns: none
//: Notes:
//:   this should be the only place is this software package that the A
//:   output is turned off
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void output_a_off(void) {
   PORTD &= ~(1<<_SOL_A); // solenoid A off
   portd_shdw &= ~(1<<_SOL_A); // portd shadow register
} // output_a_off
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: output_b_on
//: Prototype: void output_a_on(void)
//: Description: turns on output b
//: Arg1: none
//: Returns: none
//: Notes:
//:   this should be the only place is this software package that the B
//:   output is turned on
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void output_b_on(void) {
   output_a_off(); // insure output a is off
   PORTD |= (1<<_SOL_B); // turn on output B
   portd_shdw |= (1<<_SOL_B); // used to detect if the physical port pins
                        // are changed by timeout or shorted and shut off
} // output_b_on
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: output_b_off
//: Prototype: void output_b_off(void)
//: Description: turns off output b
//: Arg1: none
//: Returns: none
//: Notes:
//:   this should be the only place is this software package that the B
//:   output is turned off
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void output_b_off(void) {
   PORTD &= ~(1<<_SOL_B); // solenoid B off
   portd_shdw &= ~(1<<_SOL_B); // portd shadow register
} // output_b_off
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: output_x_off
//: Prototype: void output_x_off(void)
//: Description: turns off all outputs
//: Arg1:
//: Returns:
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void output_x_off(void) {
   PORTD &= ~((1<<_SOL_A)|(1<<_SOL_B)|(1<<_SOL_C)|(1<<_SOL_D));
   portd_shdw &= ~((1<<_SOL_A)|(1<<_SOL_B)|(1<<_SOL_C)|(1<<_SOL_D));
} // output_x_off


//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void vlv_threshold(void) {
	output_x_off(); // insure both solenoids are off
	t1_task0(50); // one second
	while (TST_TASK0) {nop();} // delay to let the hydraulics settle
	OCR1A = 40;
	output_a_on();
	do {
		OCR1A = OCR1A + 5;
	} while (!(detect_motion()));
	unib[VLV_A_OPEN].byte = OCR1A;
	//-----------------------------------------
	output_x_off(); // insure both solenoids are off
	t1_task0(50); // one second
	while (TST_TASK0) {nop();} // delay to let the hydraulics settle
	OCR1A = 40;
	output_b_on();
	do {
		OCR1A = OCR1A + 5;
	} while (!(detect_motion()));
	unib[VLV_B_OPEN].byte = OCR1A;
	output_x_off();

//	FLASH_OFF;
//	t1_task0(200); // one second
//	while (TST_TASK0) {nop();} // delay to let the hydraulics settle
//	FLASH_ON;
}


//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint8_t detect_motion(void) {
	uint16_t start, reading;
	start = ad_read(AD_SNR2); // read feedback sensor for start point
	t1_task0(75); // delay .5 of a second.
	while (TST_TASK0) nop();
	reading = ad_read(AD_SNR2); // read feedback sensor for start point
	if (reading < (start - 2)) {
		return(1);
	}
	if (reading > (start + 2)) {
		return(1);
	}
	return(0);
}
