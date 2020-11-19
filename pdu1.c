//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: File: pdu1.c
//: Company: Sunco Systems Inc.
//: Author: Marlin Unruh
//: Date: 20010228
//: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
//: File Description: communication layer for MCP2515 (ISO 11783) PDU
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
#include <avr/io.h>
#include <inttypes.h>
#include "htc.h"
#include "../shared/shared.h"
#include "../shared/mcp2515.h"

/* Pdu1
 * buff[0]  = processed/unprocessed flag
 * buff[1]  = pf
 * buff[2]  = priority
 * buff[3]  = destination
 * buff[4]  = source
 * buff[5]  = byte in CAN data field
 * buff[6]  = content
 * buff[7]  = address
 * buff[8]  = mask low
 * buff[9]  = mask high
 * buff[10] = data low
 * buff[11] = data high
 */

#define DEST      3
#define SOURCE    4
#define BYTES     5
#define CONTENT   6
#define ADDRESS   7
#define MASK_L    8
#define MASK_H    9
#define DATA_L    10
#define DATA_H    11
#define CONTENT2  8
#define ADDRESS2  10
#define TOKEN     12

void pdu1_in_port(uint8_t* buff);
void pdu1_in_var(uint8_t* buff);
void pdu1_in_ee(uint8_t* buff);

//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: pdu1_read
//: Prototype: void pdu1_read_var(uint8_t content, uint8_t address)
//: Description: used to read an 8 or 16 bit variable from the 'other'
//:   controller if the receipt flag was sent then this function returns
//:   when the flag.mcp flag receipt is cleared
//: Arg1: content (data type)
//: Arg2: address
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void pdu1_read_var(uint8_t content, uint8_t address) {
   content &= 0x7C; // insure var is selected only
   do {
      pdu1_out16((content & _READ), address, (content | _WRITE), address);
      if (!(content & _TOKEN)) // test if receipt is requested
         break;
   } while(pdu1_token());
} // end of pdu1_read
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: pdu1_out8
//: Prototype: void pdu1_out8(uint8_t content, uint8_t address,
//:   uint8_t mask, uint8_t data)
//: Description: takes 8 bit data and calls pdu1_out16 function
//: Arg1: content (data type)
//: Arg2: address
//: Arg3: mask
//: Arg4: data
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void pdu1_out8(uint8_t content, uint8_t address, uint8_t mask, uint8_t data) {
   pdu1_out16((content & _SIZE8), address, mask, data);
} // end of pdu1_out8
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: pdu1_out16
//: Prototype: void pdu1_out16(uint8_t content, uint8_t address,
//:   uint16_t va, uint16_t vb)
//: Description:
//: Var1: content: instuction byte
//: Var2: address: target address
//: Var3: va: mask/destination instruction byte
//: Var4: vb: data/destination address
//:   sets receipt flag in flag.mcp register if receipt is needed
//: Returns: none
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void pdu1_out16(uint8_t content, uint8_t address, uint16_t va, uint16_t vb) {
   uint8_t* buff = wr_buff[0].var8;
   if (flag.int1) // test if called within the int1 interrupt handler
      buff += (sizeof(wr_buff[0])); // use buff[1] for interrupts
   if (content & _TOKEN) // only if token is requested
      if (!(content & _WRITE)) // only if a read operation
         flag_mcp |= _TOKEN; // set token flag
   buff[1] = 0xEF; // content byte
   buff[2] = 0x02; // set priority
   buff[DEST] = CAB_MOD; // cab controller address
   buff[BYTES] = 6; // number of data bytes
   buff[CONTENT] = (content & 0x1F); // content
   buff[ADDRESS] = address; // address
   buff[MASK_L] = (uint8_t)va; // variable a low byte
   buff[MASK_H] = va >> 8; // variable a high byte
   buff[DATA_L] = (uint8_t)vb; // variable b low byte
   buff[DATA_H] = vb >> 8; // variable b high byte
   while(mcp_put_msg(buff));
} // end of pdu1_out16
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: pdu1_in
//: Prototype: void pdu1_in(void)
//: Description: this function determines which read or write function
//:   should be called
//: Arg1: none
//: Returns: none
//: Notes:
//:   this function between read request or write operation. determain
//:   the data type as in ram, eeprom, ports, etc. starts cab comm
//:   watchdog timer
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void pdu1_in(void) {
   uint8_t * const iobase = (uint8_t*)0x00;
   uint8_t* buff = rd_buff[0].var8;
   uint16_t tmp = 0; // temporary register
   uint8_t content;
   if (!(buff[0])) { // test buff0 for unprocessed message
      buff += sizeof(rd_buff[0]); // point to 2nd buffer
      if (!(buff[0])) // test buff1 for unprocessed message
         return; // no message pending
   }
   content = (buff[CONTENT] & _DA_MASK); // get msg content
   // CAN bus watchdog timer for comms.
   // test if source is the cab module
   if ((buff[SOURCE] == CAB_MOD) && (buff[DEST] == HITCH_MOD)) {
      t1_comm_cab(200); // reset watchdog timer (approx. 2 seconds)
      if (buff[CONTENT] & _WRITE) { // test if write?
         if (content == _VAR) // variable related read modify writes
             pdu1_in_var(buff);
         else if (content == _PORT) //  port related read modify writes
             pdu1_in_port(buff);
         else if (content == _E2) //  eeprom related read modify writes
             pdu1_in_ee(buff);
         if (buff[CONTENT] & _TOKEN) // test if receipt is requested
            flag_mcp &= ~_TOKEN; // reset receipt flag in flag_mcp variables
      } else { // read and WRITE to other controller
         if (content == _VAR) { // variable related read
            if (buff[CONTENT] & _SIZE16) {
               tmp = uniw[buff[ADDRESS]].word;
            } else {
               tmp = unib[buff[ADDRESS]].byte;
            }
         } else if (content == _PORT) { // port related read
            tmp = *(iobase + buff[ADDRESS]);
         } else if (content == _E2) { // eeprom related read
            tmp = e2_read(buff[ADDRESS]); // get variable from eeprom
         } else if (content == _AD) { // a/d converter related read
            tmp = ad_read(buff[ADDRESS] & 0x07); // get reading of a/d channel
         }
         pdu1_out16((buff[CONTENT2] | _WRITE), buff[ADDRESS2],
               0xFFFF, tmp); // write
      }
   }
   buff[0] = 0; // make buffer availiable again
   flag_mcp--; // minus one for this message removal
} // end  pdu1_in
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: pdu1_in_var
//: Prototype: void pdu1_in_var(uint8_t* buff)
//: Description: handles variable write operations (8 or 16 bit)
//: Arg1: pointer to buffer containing the value
//: Returns: none
//: Notes:
//:   a mask is supplied and applied so only the bits corresponding with
//:   the mask will be altered.
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void pdu1_in_var(uint8_t* buff) {
   union {
      uint8_t byte[2];
      uint16_t word;
   } tmp;
   uint8_t address = buff[ADDRESS];
   if (buff[CONTENT] & _SIZE16) { // if word size data
      if (address < sizeof(uniw)) {
         tmp.word = uniw[address].word; // get variable word
         tmp.byte[0] &= ~buff[MASK_L]; // inverted mask low byte AND
         tmp.byte[0] |= (buff[MASK_L] & buff[DATA_L]); // get data low byte
         tmp.byte[1] &= ~buff[MASK_H]; // inverted mask high byte
         tmp.byte[1] |= (buff[MASK_H] & buff[DATA_H]); // get data high byte
         uniw[address].word = tmp.word; // write to variable
      }
   } else { // byte size
      if (address < sizeof(unib)) {
         tmp.byte[0] = unib[address].byte; // get variable word
         tmp.byte[0] &= ~buff[MASK_L]; // inverted mask low byte AND
         tmp.byte[0] |= (buff[MASK_L] & buff[DATA_L]); // get data low byte
         unib[address].byte = tmp.byte[0]; // write to variable
      }
   }
} // end of  pdu1_in_Var
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: pdu1_in_port
//: Prototype: void pdu1_in_port(uint8_t* buff)
//: Description: handles port writing operations
//: Arg1: pointer to buffer containing the value
//: Returns: none
//: Notes:
//:   a mask is supplied and applied so only the bits corresponding with
//:   the mask will be altered.
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void pdu1_in_port(uint8_t* buff) {
   uint8_t tmp;
   uint8_t * const iobase = (uint8_t*)0x00;
   uint8_t address;
   address = buff[ADDRESS]; // get address of port
   if (address < 0x5D) { // exclude SREG and Stack Pointer Registers
      tmp = *(iobase + address); // read i/o space
      tmp &= ~buff[MASK_L]; // inverted mask low byte AND
      tmp |= (buff[MASK_L] & buff[DATA_L]); // get data low byte
      *(iobase + address) = tmp; // write i/o space
   }
} // end of  pdu1_in_port
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: pdu1_in_ee
//: Prototype: void pdu1_in_ee(uint8_t* buff)
//: Description: handles eeprom write operations (16 bit only)
//: Arg1: pointer to the buffer containing the data
//: Returns: none
//: Notes:
//:   a mask is supplied and applied so only the bits corresponding with
//:   the mask will be altered.
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
void pdu1_in_ee(uint8_t* buff) {
   union { uint8_t byte[2]; uint16_t word; } tmp;
   uint8_t address = buff[ADDRESS];
   tmp.word = e2_read(address); // get variable word
   tmp.byte[0] &= ~buff[MASK_L]; // inverted mask low byte AND
   tmp.byte[0] |= (buff[MASK_L] & buff[DATA_L]); // get data low byte
   tmp.byte[1] &= ~buff[MASK_H]; // inverted mask high byte
   tmp.byte[1] |= (buff[MASK_H] & buff[DATA_H]); // get data high byte
   e2_write(address, tmp.word); // write to variable
} // end of  pdu1_in_ee
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: Function: pdu1_token
//: Prototype: uint8_t pdu1_token(void)
//: Description: loops in this function until the receipt flag goes down
//: Arg1: none
//: Returns: 0=token received before token comm timeout occurred
//:          1=token comm timeout occurred before token was received
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
uint8_t pdu1_token(void) {
   t1_comm_token(5);
   while(TST_COMM_TOKEN) {
      if (!(flag_mcp & _TOKEN)) // wait for flag to reset
         break;
   }
   if (TST_COMM_TOKEN)
      return(0);
   else
      return(1);
} // end of pdu1_token

