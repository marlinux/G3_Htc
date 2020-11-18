;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;: File: general.s
;: Company: Sunco Systems Inc.
;: Author: Marlin Unruh
;: Date: unknown
;: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
;: File Description: general functions written in assembly langauge
;::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#define __ASSEMBLER__ 1
#define __SFR_OFFSET 0
#include <avr/io.h>

.global int32_rshift
.global int32_int8

.section .text
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- int32_rshift
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func int32_rshift
int32_rshift:
   push r16;
   push r17;
   push r18;
   push r19;
   push r30;
   push r31;
   movw r30, r24 ; place pointer in the Z register
   ldd r16, Z+0 ; get LSB
   ldd r17, Z+1 ; get
   ldd r18, Z+2 ; get
   ldd r19, Z+3 ; get MSB
   and r22, r22 ; set flags
xa1:
   breq xa2
   asr r19 ; logical shift right
   ror r18 ; rotate right thur carry
   ror r17 ; rotate right thur carry
   ror r16 ; rotate right thur carry
   dec r22 ; shift counter
   rjmp xa1
xa2:
   std Z+0, r16 ; put LSB
   std Z+1, r17 ; put
   std Z+2, r18 ; put
   std Z+3, r19 ; put MSB
   movw  r24, r30 ; pass pointer
   pop r31;
   pop r30;
   pop r19;
   pop r18;
   pop r17;
   pop r16;
   ret ; return from routine
.endfunc
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
;:- int32_int8
;:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
.func int32_int8
int32_int8:
   push r16;
   push r17;
   push r18;
   push r19;
   push r30;
   push r31;
   movw r30, r24 ; place pointer in the Z register
   ldd r16, Z+0 ; LSB
   ldd r17, Z+1 ;
   ldd r18, Z+2 ;
   ldd r19, Z+3 ; MSB
   sbrc r19, 7 ; test for positive or negative number
   rjmp xb1
   ; positive number
   mov r22, r16 ; copy original
   andi r22, 0x80 ; test MSB (signed number)
   or r22, r17 ; find if any bits are high
   or r22, r18 ;
   or r22, r19 ;
   breq xb2
   ldi r16, 0x7F ; larges posititve value possible
   clr r17 ; clear
   clr r18 ; clear
   clr r19 ; clear
   rjmp xb2 ; done / exit
xb1:
   ; negative number
   mov r22, r16 ; copy original
   ori r22, 0x7F ; test MSB (signed number)
   and r22, r17 ; find if any bit are low
   and r22, r18
   and r22, r19
   com r22 ; complement to test
   breq xb2
   ldi r16, 0x80 ; largest negative value possible
   ser r17 ; set as a real 32 bit value with ceiling at 8 bits
   ser r18
   ser r19
xb2:
   std Z+0, r16 ; put LSB
   std Z+1, r17 ; put
   std Z+2, r18 ; put
   std Z+3, r19 ; put MSB
   mov r24, r30 ; return the pointer
   pop r31;
   pop r30;
   pop r19;
   pop r18;
   pop r17;
   pop r16;
   ret ; return from routine
.endfunc


