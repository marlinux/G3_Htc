//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
//: File: htc.h
//: Company: Sunco Systems Inc.
//: Author: Marlin Unruh
//: Date: 20010302
//: Compiler: AVR-GCC 3.4.5 WinAVR 20060125
//: File Description: hitch module header file
//:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

// #define DEBUG 1
//
// software version data is located in specific.s under version
//
#define wdr()           __asm__ volatile("wdr")
#define nop()           __asm__ volatile("nop")
#define DEBUG_FLASHER   for(;;) { PORTC ^= (1<<PC1); t1_delay(5);}
#define RXF_PORT        PINC // RX Buffer full
#define FLASHER         PORTC ^=  (1<<PC1)
#define FLASH_ON        PORTC &= ~(1<<PC1)
#define FLASH_OFF       PORTC |=  (1<<PC1)
#define _SOL_A          PD6 // port pin for SOL A (Q1-black plug)
#define _SOL_B          PD7 // port pin for SOL B (Q2-white plug)
#define _SOL_C          PD1 // port pin for SOL C
#define _SOL_D          PD0 // port pin for SOL D
#define _SOL_AB_ENABLE  PORTD |=  (1<<PD5)
#define _SOL_AB_DISABLE PORTD &= ~(1<<PD5)
#define _SOL_CD_ENABLE  PORTD |=  (1<<PD4)
#define _SOL_CD_DISABLE PORTD &= ~(1<<PD4)
#define CAN_CS_T        PORTC &= ~(1<<PC2) // CAN bus CS true
#define CAN_CS_F        PORTC |=  (1<<PC2) // CAN bus CS false
#define TOIE2_ENABLE    TIMSK |=  (1<<TOIE2) // timer 2 overflow interrupt
#define TOIE2_DISABLE   TIMSK &= ~(1<<TOIE2) // timer 2 overflow interrupt
#define OCIE2_ENABLE    TIMSK |= (1<<OCIE2) // timer 2 compare interrupt
#define OCIE2_DISABLE   TIMSK &= ~(1<<OCIE2) // timer 2 compare interrupt
#define OCIE1A_ENABLE   TIMSK |=  (1<<OCIE1A)
#define OCIE1A_DISABLE  TIMSK &= ~(1<<OCIE1A)
#define OCIE1B_ENABLE   TIMSK |=  (1<<OCIE1B)
#define OCIE1B_DISABLE  TIMSK &= ~(1<<OCIE1B)
#define INT1_ENABLE     GICR |= (1<<INT1) // enable external int1
#define INT1_DISABLE    GICR &= ~(1<<INT1) // disable external int1
#define INT1_TST        PIND & (1<<PD3) // interrupt line from MCP2520
#define INT0_ENABLE     GICR |= (1<<INT0) // enable external int0
#define INT0_DISABLE    GICR &= ~(1<<INT0) // disable external int0
#define TRUE            1
#define FALSE           (!TRUE)
#define TARGET_12V      570 // low voltage threshold (12.0v)
#define AMPS_MIN        287 // (410) approx. 1.75 amps
// **** Sensor Calibration Request Register ****
// **** uniw[SNR_CALIB_RQ].byte[1] ****
#define _SNR_CALI       (1<<7)
#define CLR_SNR_CALI    unib[SNR_CALIB_RQ].byte &= ~_SNR_CALI
#define SET_SNR_CALI    unib[SNR_CALIB_RQ].byte |= _SNR_CALI
#define TST_SNR_CALI    (unib[SNR_CALIB_RQ].byte & _SNR_CALI)
// ---- Timer0 Related Flag Def ----
// ---- flag.tmr1a Register ----
#define _SGNL_DELAY     (1<<4) // wand active timer
#define _TASK1          (1<<3) // task 1 timer
#define _TASK0          (1<<2) // task 0 timer
#define _OUTPUT         (1<<1) // output timer for solenoids
#define _DELAY          (1<<0) // delay timer
#define SET_SGNL_DELAY  flag.tmr1a |= _SGNL_DELAY
#define CLR_SGNL_DELAY  flag.tmr1a &= ~_SGNL_DELAY
#define TST_SGNL_DELAY  (flag.tmr1a & _SGNL_DELAY)
#define SET_TASK1       flag.tmr1a |= _TASK1
#define CLR_TASK1       flag.tmr1a &= ~_TASK1
#define TST_TASK1       (flag.tmr1a & _TASK1)
#define SET_TASK0       flag.tmr1a |= _TASK0
#define CLR_TASK0       flag.tmr1a &= ~_TASK0
#define TST_TASK0       (flag.tmr1a & _TASK0)
#define SET_OUTPUT      flag.tmr1a |= _OUTPUT
#define CLR_OUTPUT      flag.tmr1a &= ~_OUTPUT
#define TST_OUTPUT      (flag.tmr1a & _OUTPUT)
#define SET_DELAY       flag.tmr1a |= _DELAY
#define CLR_DELAY       flag.tmr1a &= ~_DELAY
#define TST_DELAY       (flag.tmr1a & _DELAY)
// ---- flag.tmr1b ----
#define _COMM_CAB       (1<<2) // watchdog timer flag for cab comms
#define _COMM_TOKEN     (1<<1) // comm token timer
#define _COMM_OUT       (1<<0) // comm output timer
#define SET_COMM_CAB    flag.tmr1b |= _COMM_CAB
#define CLR_COMM_CAB    flag.tmr1b &= ~_COMM_CAB
#define TST_COMM_CAB    (flag.tmr1b & _COMM_CAB)
#define SET_COMM_OUT    flag.tmr1b |= _COMM_OUT
#define CLR_COMM_OUT    flag.tmr1b &= ~_COMM_OUT
#define TST_COMM_OUT    (flag.tmr1b & _COMM_OUT)
#define SET_COMM_TOKEN  flag.tmr1b |= _COMM_TOKEN
#define CLR_COMM_TOKEN  flag.tmr1b &= ~_COMM_TOKEN
#define TST_COMM_TOKEN  (flag.tmr1b & _COMM_TOKEN)

#ifdef master_header
/********************************************************************
   ***** THIS SECTION IS TO BE INCLUDED ONLY ONCE *****
********************************************************************/
extern const uint8_t PROGMEM    _firmware_version_[4] ; // defined in specific.s
extern volatile uint8_t flag_mcp;
extern uint8_t tmp_buff[8];
struct {
   uint8_t avr[8];
   uint16_t scf[3];
} snr ;
volatile struct { // volatile struct {
   uint8_t tmr1a; // timer 1 register a flags
   uint8_t tmr1b; // timer 1 register b flags
   uint8_t int1; // flag for int1 in process
   uint8_t min_touch; // flag register for min reached on sensors
   uint8_t max_touch; // flag register for max reached on sensors
} flag ;
volatile struct {
   uint8_t comm_cab; // watchdog timer for cab comms
   uint8_t comm_out; // communications timer
   uint8_t comm_token; // comm token timer
   uint8_t delay; // delay counter/timer
   uint8_t task0; // task 0 counter/timer
   uint16_t task1; // task 1 counter/timer
   uint16_t wand_delay; // wand active timer
   uint16_t output; // timer for solenoid outputs
} tmr1 ;
volatile uint8_t portd_shdw; // copy of port d
volatile uint8_t vlv_threshold_enable; // flags to control when vlv_threshold() is run
// ---- main ----
void debug_flasher(void);
void ini_device(void);
void ini_ddr_port(void);
void ini_t1(void);
void ini_t2(void);
void ini_spi_port(void);
void ini_wdt(uint8_t prescale);
void ini_var(void);
void ini_sync(void);
void ini_defaults(char mode);
void ini_canbus(void);
void reset(void);
void stand_alone(void);
void shut_down(void);
void wdt_shutdown(void);
// ---- run_module ----
void run_module(void);
void sensor_scale_factor(uint8_t sensor);
uint8_t sensor_read(uint8_t sensor);
uint16_t sensor_min_max(uint8_t sensor, uint16_t snr_value);
uint8_t sensor_average(uint8_t snr_value);
void sensor_calibrate(void);
void output_a_on(void);
void output_b_on(void);
void output_a_off(void);
void output_b_off(void);
void output_x_off(void);
// ---- pdu1.c ----
void pdu1_read_var(uint8_t content, uint8_t address);
void pdu1_out8(uint8_t content, uint8_t address, uint8_t mask, uint8_t data);
void pdu1_out16(uint8_t content, uint8_t address, uint16_t va, uint16_t vb);
void pdu1_in(void);
uint8_t pdu1_token(void);
// ---- _diag.c ----
void diag_flags(uint8_t flags);
void diag_system(void);
void diag_solenoid(void);
void diag_sensor(void);
void diag_hydrl(void);
// ---- pph.c ----
uint16_t ad_read(uint8_t ch);
uint16_t e2_read(uint16_t addr);
void e2_write(uint16_t addr, uint16_t data);
void t1_delay(uint8_t duration);
void t1_task0(uint8_t duration);
void t1_task1(uint16_t duration);
void t1_wand(uint16_t duration);
void t1_comm_cab(uint8_t duration);
void t1_comm_token(uint8_t duration);
// ---- mcp.c ----
void mcp_config(uint8_t mode);
void mcp_reset(void);
void mcp_baud(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
void mcp_filters(uint8_t addr, uint8_t sid8, uint8_t sid0);
void mcp_ctrl(void);
void mcp_get_msg(uint8_t rx_flags);
uint8_t mcp_put_msg(uint8_t *buff);
void mcp_error_handler(void);
// ---- spi.c ----
uint8_t spi_mcp_read_status(void);
void spi_mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data);
uint8_t spi_mcp_read(uint8_t addr);
void spi_mcp_write(uint8_t addr, uint8_t data);
void spi_mcp_seq_read(uint8_t *buff, uint8_t address, uint8_t count);
void spi_mcp_seq_write(uint8_t *buff, uint8_t address, uint8_t count);
uint8_t spi_transfer(uint8_t data);
// ----- assmbly.s -----
uint8_t* int32_rshift(uint8_t* buff, uint8_t count);
uint8_t* int32_int8(uint8_t* buff);

#else
/********************************************************************
   Define all variables and prototypes below as external
********************************************************************/
// ---- variables ----
extern volatile uint8_t flag_mcp;
extern uint8_t tmp_buff[8];
extern struct {
   uint8_t avr[8];
   uint16_t scf[3];
} snr ;
extern volatile struct {
   uint8_t tmr1a; // timer 1 register a flags
   uint8_t tmr1b; // timer 1 register b flags
   uint8_t int1; // flag for int1 in process
   uint8_t min_touch; // flag register for min reached on sensors
   uint8_t max_touch; // flag register for max reached on sensors
} flag ;
extern volatile struct {
   uint8_t comm_cab; // watchdog timer for cab comms
   uint8_t comm_out; // communications timer
   uint8_t comm_token; // comm token timer
   uint8_t delay; // delay counter/timer
   uint8_t task0; // task 0 counter/timer
   uint16_t task1; // task 1 counter/timer
   uint16_t wand_delay; // wand active timer
   uint16_t output; // timer for solenoid outputs
} tmr1 ;
extern volatile uint8_t portd_shdw; // copy of port d
extern volatile uint8_t vlv_threshold_enable; // flags to control when vlv_threshold() is run
// ---- main ----
extern void debug_flasher(void);
extern void ini_device(void);
extern void ini_ddr_port(void);
extern void ini_t1(void);
extern void ini_t2(void);
extern void ini_spi_port(void);
extern void ini_canbus(void);
extern void ini_wdt(uint8_t prescale);
extern void wdt_shutdown(void);
extern void ini_var(void);
extern void ini_sync(void);
extern void ini_defaults(char mode);
extern void stand_alone(void);
extern void shut_down(void);
// ---- run_module ----
extern void run_module(void);
extern void sensor_scale_factor(uint8_t sensor);
extern uint8_t sensor_read(uint8_t sensor);
extern uint16_t sensor_min_max(uint8_t sensor, uint16_t snr_value);
extern uint8_t sensor_average(uint8_t snr_value);
extern void sensor_calibrate(void);
extern void output_a_on(void);
extern void output_b_on(void);
extern void output_a_off(void);
extern void output_b_off(void);
extern void output_x_off(void);
// ---- pdu1.c ----
extern void pdu1_read_var(uint8_t content, uint8_t address);
extern void pdu1_out8(uint8_t content, uint8_t address, uint8_t mask, uint8_t data);
extern void pdu1_out16(uint8_t content, uint8_t address, uint16_t va, uint16_t vb);
extern void pdu1_in(void);
extern uint8_t pdu1_token(void);
// ---- _diag.c ----
extern void diag_flags(uint8_t flags);
extern void diag_system(void);
extern void diag_solenoid(void);
extern void diag_sensor(void);
extern void diag_hydrl(void);
// ---- pph.c ----
extern uint16_t ad_read(uint8_t ch);
extern uint16_t e2_read(uint16_t addr);
extern void e2_write(uint16_t addr, uint16_t data);
extern void t1_delay(uint8_t duration);
extern void t1_comm(uint8_t duration);
extern void t1_task0(uint8_t duration);
extern void t1_task1(uint16_t duration);
extern void t1_wand(uint16_t duration);
extern void t1_comm_cab(uint8_t duration);
extern void t1_comm_token(uint8_t duration);
// ---- mcp.c ----
extern void mcp_config(uint8_t mode);
extern void mcp_reset(void);
extern void mcp_baud(uint8_t cnf1, uint8_t cnf2, uint8_t cnf3);
extern void mcp_filters(uint8_t addr, uint8_t sid8, uint8_t sid0);
extern void mcp_ctrl(void);
extern void mcp_get_msg(uint8_t rx_flags);
extern uint8_t mcp_put_msg(uint8_t *buff);
extern void mcp_error_handler(void);
// ---- spi_mcp.c ----
extern uint8_t spi_mcp_read_status(void);
extern void spi_mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data);
extern uint8_t spi_mcp_read(uint8_t addr);
extern void spi_mcp_write(uint8_t addr, uint8_t data);
extern void spi_mcp_seq_read(uint8_t *buff, uint8_t address, uint8_t count);
extern void spi_mcp_seq_write(uint8_t *buff, uint8_t address, uint8_t count);
extern uint8_t spi_transfer(uint8_t data);
// ----- assmbly.s -----
extern uint8_t* int32_rshift(uint8_t* buff, uint8_t count);
extern uint8_t* int32_int8(uint8_t* buff);
#endif
/***** end of header file *******************************************/
