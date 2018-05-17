#ifndef BMS_MASTER_H
#define BMS_MASTER_H

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>



/*----- Macro Definitions -----*/
/* FLAGS */
// Sensing Flags
#define BSPD_CURRENT            0b00000001
#define READ_VALS_FLAG          0b00000010
#define UNDER_VOLTAGE           0b00000100
#define OVER_VOLTAGE            0b00001000
#define SOFT_OVER_VOLTAGE       0b00010000
#define OVER_TEMP               0b00100000
#define OPEN_SHUTDOWN           0b01000000

// Relay Flags
#define AIRS_CLOSED             0b00000001
#define IMD_TRIPPED             0b00000010


/* LEDs */
#define LED_1                   PB5
#define LED_2                   PB6
#define LED_3                   PC0
#define LED_1_PORT              PORTB
#define LED_2_PORT              PORTB
#define LED_3_PORT              PORTC

#define RJ45_LED_ORANGE         PC4
#define RJ45_LED_GREEN          PC5
#define RJ45_ORANGE_PORT        PORTC
#define RJ45_GREEN_PORT         PORTC

/* I/O */
#define RELAY_PIN               PB2
#define RELAY_PORT              PORTB
#define FAN_PIN                 PC1
#define FAN_PORT                PORTC

/* MUX defs */
#define MUX_CHANNELS            5       // Number of cell channels to iterate through
#define MUX_1_ADDRESS           0x49    // I2C Addresses for the two AMUXes on each board.
#define MUX_2_ADDRESS           0x48

/* LTC68xx defs */
#define TOTAL_IC                8 // Number of LTC6804 boards in the stack

#define ENABLED                 1
#define DISABLED                0
#define CELL_CHANNELS           12
#define AUX_CHANNELS            6
#define STAT_CHANNELS           4
#define CELL                    1
#define AUX                     2
#define STAT                    3

/* CAN */
#define BROADCAST_BMS_MASTER    0
#define BROADCAST_BMS_VOLTAGE   1
#define BROADCAST_BMS_TEMP      2


/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the ltc6811
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
extern uint16_t cell_codes[TOTAL_IC][CELL_CHANNELS];
/*!<
  The cell codes will be stored in the cell_codes[][12] array in the following format:
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
 ****/

extern uint16_t aux_codes[TOTAL_IC][AUX_CHANNELS];
/*!<
  The GPIO codes will be stored in the aux_codes[][6] array in the following format:
  |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
  |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
  |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
  */

extern uint16_t cell_temperatures[TOTAL_IC][CELL_CHANNELS];
/*!<
  The cell temperatures will be stored in the cell_temperatures[][12] array in the following format:
  |  cell_temperatures[0][0]| cell_temperatures[0][1] |  cell_temperatures[0][2]|    .....     |  cell_temperatures[0][11]|  cell_temperatures[1][0] | cell_temperatures[1][1]|  .....   |
  |-------------------------|-------------------------|-------------------------|--------------|--------------------------|--------------------------|------------------------|----------|
  |IC1 Cell 1               |IC1 Cell 2               |IC1 Cell 3               |    .....     |  IC1 Cell 12             |IC2 Cell 1                |IC2 Cell 2              | .....    |
 ****/

extern uint16_t discharge_status[TOTAL_IC];
/*!<
  Whether each cell is discharging will be stored in the discharge_status[12] array in the following format:
  |discharge_status[0]  |discharge_status[1]  |discharge_status[2]  |    .....     | discharge_status[TOTAL_IC]  |
  |---------------------|---------------------|---------------------|--------------|-----------------------------|
  |IC1 Discharge[11:0]  |IC2 Discharge[11:0]  |IC3 Discharge[11:0]  |    .....     | IC_TOTAL_IC Discharge[11:0] |
 ****/



/*----- Timers -----*/
void init_read_timer(void);
void init_fan_pwm(uint8_t duty_cycle);

/*----- Read Voltages -----*/
uint8_t read_all_voltages(void);

/*----- Read Temperatures -----*/
uint8_t read_all_temperatures(void);

/*----- Transmit Information -----*/
void transmit_voltages(void);
void transmit_temperatures(void);
void transmit_discharge_status(void);

/*----- Discharge -----*/
void enable_discharge(uint8_t ic, uint8_t cell);
void disable_discharge(uint8_t ic, uint8_t cell);

/*----- Multiplexer Helper Functions -----*/
void set_mux_channel(uint8_t total_ic, uint8_t i2c_address, uint8_t channel);
void mux_disable(uint8_t total_ic, uint8_t i2c_address);


/* Register Configuration for Communication with LTC6804 */
uint8_t tx_cfg[TOTAL_IC][6];
uint8_t rx_cfg[TOTAL_IC][6];

uint8_t clock_prescale;

#endif
