#include <avr/io.h>
#include <avr/pgmspace.h> //TODO: Determine if this is necessary
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include "LTC_defs.h"
#include "can_api.h"
#include "main.h"

// OEM defs
volatile uint8_t FLAGS = 0x00;

#define BSPD_CURRENT        0b00000001
#define READ_VALS           0b00000010
#define UNDER_VOLTAGE       0b00000100
#define OVER_VOLTAGE        0b00001000
#define SOFT_OVER_VOLTAGE   0b00010000
#define OVER_TEMP           0b00100000
#define OPEN_SHDN           0b00101100
#define AIRS_CLOSED         0b01000000
#define IMD_TRIPPED         0b10000000

#define PROG_LED_1 PB5
#define PROG_LED_2 PB6
#define PROG_LED_3 PC0

#define IMD_SENSE_PIN      PC7
#define IMD_SENSE_PIN_REG  PINC

// MUX defs
#define MUX_CHANNELS 6
#define MUX1_ADDRESS 0x49
#define MUX2_ADDRESS 0x48

//LTC68xx defs
#define TOTAL_IC 6

#define ENABLED 1
#define DISABLED 0
#define CELL_CHANNELS 12
#define AUX_CHANNELS 6
#define STAT_CHANNELS 4
#define CELL 1
#define AUX 2
#define STAT 3

#define LED_ORANGE PC4
#define LED_GREEN PC5
#define EXT_LED_PORT PORTC

uint8_t can_recv_msg[2];
uint8_t timer_counter;

//ADC Command Configurations
const uint8_t ADC_OPT = ADC_OPT_DISABLED; // See ltc6811_daisy.h for Options
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; // See ltc6811_daisy.h for Options
const uint8_t ADC_DCP = DCP_DISABLED; // See ltc6811_daisy.h for Options
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; // See ltc6811_daisy.h for Options
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; // See ltc6811_daisy.h for Options
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; // See ltc6811_daisy.h for Options

const uint8_t total_ic = TOTAL_IC; //number of BMS slaves on the daisy chain


//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 36000;//35900; // Over voltage threshold ADC Code. LSB = 0.0001
const uint16_t SOFT_OV_THRESHOLD = 36200;//35500; //Soft over-voltage for discharge
const uint16_t UV_THRESHOLD = 22000; //20000;// Under voltage threshold ADC Code. LSB = 0.0001

//Thermistor voltage times this number must be greater than measured bus voltage (times 1000 for AVR)
const uint16_t THERM_V_FRACTION = 3807; //Maximum fraction is 99/26 = 3.807 (1% top thermistors, therm 26K at 58 C)



/******************************************************
 *** Global Battery Variables received from 681x commands
 These variables store the results from the ltc6811
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
uint16_t cell_codes[TOTAL_IC][CELL_CHANNELS];
/*!<
  The cell codes will be stored in the cell_codes[][12] array in the following format:
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |
 ****/

uint16_t aux_codes[TOTAL_IC][AUX_CHANNELS];
/*!<
  The GPIO codes will be stored in the aux_codes[][6] array in the following format:
  |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
  |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
  |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |
  */

uint16_t cell_temperatures[TOTAL_IC][CELL_CHANNELS];
/*!<
  The cell temperatures will be stored in the cell_temperatures[][12] array in the following format:
  |  cell_temperatures[0][0]| cell_temperatures[0][1] |  cell_temperatures[0][2]|    .....     |  cell_temperatures[0][11]|  cell_temperatures[1][0] | cell_temperatures[1][1]|  .....   |
  |-------------------------|-------------------------|-------------------------|--------------|--------------------------|--------------------------|------------------------|----------|
  |IC1 Cell 1               |IC1 Cell 2               |IC1 Cell 3               |    .....     |  IC1 Cell 12             |IC2 Cell 1                |IC2 Cell 2              | .....    |
 ****/

uint16_t discharge_status[TOTAL_IC];
/*!<
  Whether each cell is discharging will be stored in the discharge_status[12] array in the following format:
  |discharge_status[0]  |discharge_status[1]  |discharge_status[2]  |    .....     | discharge_status[TOTAL_IC]  |
  |---------------------|---------------------|---------------------|--------------|-----------------------------|
  |IC1 Discharge[11:0]  |IC2 Discharge[11:0]  |IC3 Discharge[11:0]  |    .....     | IC_TOTAL_IC Discharge[11:0] |
 ****/

/* Config register for comms with LTC6804 */
uint8_t tx_cfg[TOTAL_IC][6];
uint8_t rx_cfg[TOTAL_IC][6];

int main (void)
{
    sei(); //allow interrupts
    // Set PB5,PB6,PC0 to output
    DDRB |= _BV(PB5) | _BV(PB6) | _BV(PB7) | _BV(PB4) | _BV(PB2);
    DDRB &= ~_BV(PB3); //BSPD Current Sense
    DDRC |= _BV(PC0) | _BV(PC4) | _BV(PC5); //program LED
    PORTC &= ~(_BV(LED_ORANGE) | _BV(LED_GREEN) | _BV(PROG_LED_3));

    PORTB |= _BV(PB2); //close relay
    EXT_LED_PORT |= _BV(LED_ORANGE);

    //Pin Change Interrupts
    PCICR |= _BV(PCIE0); //enable 0th mask register for interrupts
    PCMSK0 |= _BV(PCINT3); //enable interrupts for INT3

    //Init ltc6804 config register
    init_cfg();

    //CAN init
    CAN_init(CAN_ENABLED);
    CAN_wait_on_receive(0, CAN_IDT_AIR_CONTROL, CAN_IDT_AIR_CONTROL_L, CAN_IDM_single);

    // BMS Open Relay Message
    CAN_wait_on_receive(4, 0x02, 0x01, CAN_IDM_single);

    //Read Timer init
    init_read_timer();

    //PWM init
    init_fan_pwm(0x20);

    //Watchdog init
    wdt_enable(WDTO_8S);
    //wdt_disable();

    // SPI init
    init_spi_master();

    // Slave Select
    DDRD |= _BV(PD3);
    PORTD |= _BV(PD3); //set master one high
    PORTB |= _BV(PB4); //set slave one low

    // Read LTC 6804 Config
    // uint8_t rx_cfg[total_ic][8];

    //PORTB |= _BV(PROG_LED_1);

    //Initialize temp and voltage values
    //uint8_t tmp = read_all_voltages();
    //tmp += read_all_temperatures();

    uint8_t test_msg[8] =  {0,0,0,0,0,0,0,0};
    CAN_transmit(0, 0x13, 8, test_msg);


    while(1) {

        //PORTB &= ~(_BV(PROG_LED_1) | _BV(PROG_LED_2)); //Turn off status LEDs
        //PORTC &= ~_BV(PROG_LED_3);
        /*
         * Open Shutdown Circuit: matches UNDER_VOLTAGE, OVER_VOLTAGE, OVER_TEMP
         */

        if (FLAGS & OPEN_SHDN) {
            PORTB &= ~_BV(PB2); //open relay
            //EXT_LED_PORT &= ~_BV(LED_ORANGE);
        }

        if (FLAGS & UNDER_VOLTAGE) { //Set LED D7, PB5
            PORTB |= _BV(PROG_LED_1); 
        } else {
            PORTB &= ~_BV(PROG_LED_1);
        }

        if (FLAGS & OVER_VOLTAGE) { //Set LED D8, PB6
            PORTB |= _BV(PROG_LED_2);
        } else {
            PORTB &= ~_BV(PROG_LED_2);
        }

        if (FLAGS & OVER_TEMP) { //Set LED D9, PC0
            PORTC |= _BV(PROG_LED_3);
        } else {
            PORTC &= ~_BV(PROG_LED_3);
        }

        if (FLAGS & AIRS_CLOSED){
            EXT_LED_PORT |= _BV(LED_ORANGE);
        } else {
            EXT_LED_PORT &= ~_BV(LED_ORANGE);
        }

        if (can_recv_msg[1] == 0x99){
            _delay_ms(5);
            uint8_t flags_msg[1] = {FLAGS};
            CAN_transmit(1,0x19,1,flags_msg);
            _delay_ms(5);
            CAN_transmit(1, 0x18, 2, can_recv_msg);
            can_recv_msg[1] =  0x55; 
            _delay_ms(5);
        }

        if (FLAGS & READ_VALS) { 
            EXT_LED_PORT ^= _BV(LED_GREEN);
            uint8_t error = 0;
            error += read_all_voltages();
            error += read_all_temperatures();
            o_ltc6811_wrcfg(TOTAL_IC, tx_cfg);
            //Probably want to do something with error in the future
            transmit_voltages();
            //update discharge transistors
            transmit_temperatures();
            transmit_discharge_status();

            //uint8_t test_msg[8] =  {1,2,3,4,5,6,7,8};
            //CAN_transmit(0, 0x13, 8, test_msg);
            
            FLAGS &= ~READ_VALS;
            CAN_wait_on_receive(0, CAN_IDT_AIR_CONTROL, CAN_IDT_AIR_CONTROL_L, CAN_IDM_single);
        }

        if (bit_is_set(IMD_SENSE_PIN_REG, IMD_SENSE_PIN)) {
            FLAGS |= IMD_TRIPPED;
        } else {
            FLAGS &= ~IMD_TRIPPED;
        }

        wdt_reset();
    }

}

//ISRs//////////////////////////////////////////////////////////////////////////

ISR(CAN_INT_vect){
    // Check first MOb (AIR Control)
    CANPAGE = (0 << MOBNB0);
    if (bit_is_set(CANSTMOB, RXOK)) {
        volatile uint8_t msg = CANMSG; //grab the first byte of the CAN message
        msg = CANMSG;
        can_recv_msg[0] =  msg; 
        can_recv_msg[1] =  0x99; 
        

        if (msg == 0xFF) {
            FLAGS |= AIRS_CLOSED;
            //EXT_LED_PORT |= _BV(LED_ORANGE);
        } else {
            FLAGS &= ~(AIRS_CLOSED);
            //EXT_LED_PORT &= ~_BV(LED_ORANGE);
        }

        //setup to receive again
        CANSTMOB = 0x00;
        CAN_wait_on_receive(0, CAN_IDT_AIR_CONTROL, CAN_IDT_AIR_CONTROL_L, CAN_IDM_single);
    }

    // BMS Open Relay message
    CANPAGE = (4 << MOBNB0);
    if (bit_is_set(CANSTMOB, RXOK)) {
        volatile uint8_t msg = CANMSG;

        if (msg != 0x00) {
            FLAGS |= OPEN_SHDN;
        }
        
        CANSTMOB = 0x00;
        CAN_wait_on_receive(4, 0x02, 0x01, CAN_IDM_single);
    }
}

ISR(PCINT0_vect)
{
    if (bit_is_set(PINB, PB3)){
        FLAGS |= BSPD_CURRENT;
    }
    else {
        FLAGS &= ~BSPD_CURRENT;
    }

}

// ISR(TIMER1_OVF_vect)
// {
//     FLAGS |= READ_VALS;
// }

ISR(TIMER0_COMPA_vect)
{
    if (timer_counter >= 50) {
        FLAGS |= READ_VALS;
        timer_counter = 0;
    }
    timer_counter ++;
}


//TRANSMIT VALUES///////////////////////////////////////////////////////////////

/*!<
  Cell Voltages will be transmitted in this CAN message, LSB = 0.0001:
  |           msg[0] |           msg[1] |           msg[2] |           msg[3] |    .....     |            msg[6] |            msg[7] |
  |------------------|------------------|------------------|------------------|--------------|-------------------|-------------------|
  |IC/Segment number |first cell index  |msg Cell 1 High   |msg Cell 1 Low    |    .....     |msg Cell 3 High    |msg Cell 3 Low     |
 ****/
void transmit_voltages(void)
{
    //Declare message variable out here
    uint8_t msg[8];
    for (uint8_t i = 0; i < TOTAL_IC; i++) {//Iterate through ICs
        msg[0] = i; //
        for (uint8_t j = 0; j < 4; j++) { //4 messages per IC
            uint8_t idx = j * 3;
            msg[1] = idx;
            for (uint8_t k = 0; k < 3; k++) { //3 cells per message
                uint16_t cell_voltage = cell_codes[i][idx + k];
                msg[2+k*2] = (uint8_t)(cell_voltage >> 8); //High byte
                msg[3+k*2] = (uint8_t)cell_voltage;  //Low byte
            }

            CAN_transmit(1, 0x13, 8, msg);
            _delay_ms(5);
        }
    }

}



/*!<
  Cell temperatures will be transmitted in this CAN message as 16 bit ints, LSB = 0.0001:
  |           msg[0] |           msg[1] |           msg[2] |           msg[3] |           msg[4] |           msg[5] |           msg[6] |           msg[7] |
  |------------------|------------------|------------------|------------------|------------------|------------------|------------------|------------------|
  |IC/Segment number |first cell index  |msg Cell 1 High   |msg Cell 1 Low    |msg Cell 2 High   |msg Cell 2 low    |VREF2 High        | VREF2 Low        |
 ****/
void transmit_temperatures(void)
{
    //Declare message variable out here
    uint8_t msg[8];
    for (uint8_t i = 0; i < TOTAL_IC; i++) {//Iterate through ICs
        msg[0] = i; //segment
        uint16_t vref2 = aux_codes[i][5]; //vref2 for the segment
        for (uint8_t j = 0; j < 6; j++) { //6 messages per IC
            uint8_t idx = j * 2;
            msg[1] = idx;
            for (uint8_t k = 0; k < 2; k++) { //2 cells per message
                uint16_t cell_temp = cell_temperatures[i][idx + k];
                msg[2+k*2] = (uint8_t)(cell_temp >> 8); //High byte
                msg[3+k*2] = (uint8_t)cell_temp;  //Low byte
            }
            msg[6] = (uint8_t)(vref2 >> 8); //vref2 High byte
            msg[7] = (uint8_t)(vref2); //vref2 Low byte

            CAN_transmit(2, 0x14, 8, msg);
            _delay_ms(5);
        }
    }
}

/*!<
  Cell Discharge Status will be transmitted in this CAN message, LSB 
  |           msg[0] |           msg[1] |           msg[2] |           msg[3] |    .....     |            msg[5] |            msg[6] |
  |------------------|------------------|------------------|------------------|--------------|-------------------|-------------------|
  |IC group (0 or 1) |IC1 Status High   |IC1 Status Low    |IC2 Status High   |    .....     |msg Cell 3 High    |IC 3 Status Low    |
 ****/
void transmit_discharge_status(void)
{
    //Declare message variable out here
    uint8_t msg[7] = {0,0,0,0,0,0,0};
    for (uint8_t i = 0; i < 2; i++) {//Send two different messages
        msg[0] = i; //
        for (uint8_t k = 0; k < 3; k++) { //3 ICs per message
            uint16_t disch_status = discharge_status[(i*3) + k];
            msg[1+k*2] = (uint8_t)(disch_status >> 8); //High byte
            msg[2+k*2] = (uint8_t)disch_status;  //Low byte
        }

        CAN_transmit(3, 0x15, 7, msg);
        _delay_ms(5);
    }

}

//READ VALUES TIMER/////////////////////////////////////////////////////////////

void init_read_timer(void) {
    TCCR0A &= ~(_BV(WGM11) | _BV(WGM10)); //set timer in CTC mode with reset on match with OCR1A
    TCCR0B &= ~(_BV(CS11));
    TCCR0B |= _BV(CS10);
    TCCR0B |= _BV(CS12)  | _BV(WGM12); //Set prescaler to 1/1024

    TIMSK0 |= _BV(OCIE0A); // Enable CTC compare match
    OCR0A |= 0xFF; //timer compare value
}


//FAN PWM //////////////////////////////////////////////////////////////////////

void init_fan_pwm(uint8_t duty_cycle)
{
    //Output compare pin is OC1B, so we need OCR1B as our counter
    TCCR1B |= _BV(CS00); //Clock prescale set to max speed
    TCCR1B |= _BV(WGM12);
    TCCR1A |= _BV(COM1B1) | _BV(WGM10); // Fast PWM 8-bit mode
    TCCR1A &= ~_BV(COM1B0); // Set on match, clear on top
    DDRC |= _BV(PC1); //Enable

    OCR1B = (uint8_t) duty_cycle;
}

//VOLTAGE MEASUREMENT///////////////////////////////////////////////////////////

uint8_t read_all_voltages(void) // Start Cell ADC Measurement
{
    uint8_t error = 0;

    wakeup_sleep(TOTAL_IC);

    o_ltc6811_adcv(ADC_CONVERSION_MODE,ADC_DCP,CELL_CH_TO_CONVERT);
    o_ltc6811_pollAdc();
    error = o_ltc6811_rdcv(0,TOTAL_IC,cell_codes); //Parse ADC measurements

    for (uint8_t i = 0; i < TOTAL_IC; i++) {
        
        for (uint8_t j = 0; j < CELL_CHANNELS; j++) {
            disable_discharge(i+1, j+1); //IC and Cell are 1-indexed}
            if (cell_codes[i][j] > OV_THRESHOLD) {
                FLAGS |= OVER_VOLTAGE;
                error += 1;
            }

            if (cell_codes[i][j] > SOFT_OV_THRESHOLD) {
                FLAGS |= SOFT_OVER_VOLTAGE;
                
                if (FLAGS & AIRS_CLOSED){
                    enable_discharge(i+1, j+1); //IC and Cell are 1-indexed
                } 
            } 

            if (cell_codes[i][j] < UV_THRESHOLD) {
                FLAGS |= UNDER_VOLTAGE;
                error += 1;
            }
        }
    }
    if (error == 0) {
        //upon successful execution clear flags
        FLAGS &= ~(OVER_VOLTAGE | UNDER_VOLTAGE);
    }

    return error;
}

//TEMPERATURE MEASUREMENT///////////////////////////////////////////////////////

uint8_t read_all_temperatures(void) // Start thermistor ADC Measurement
{
    uint8_t error = 0;
    //wakeup_sleep(TOTAL_IC); //only needed if not called directly after reading voltages
    //Iterate through first mux
    mux_disable(TOTAL_IC, MUX2_ADDRESS);
    for (uint8_t i = 0; i < MUX_CHANNELS; i++) {

        //Changing channel over I2C is going to be tricky
        set_mux_channel(TOTAL_IC, MUX1_ADDRESS, i);
        //_delay_us(50); //TODO: This is a blatant guess

        o_ltc6811_adax(MD_7KHZ_3KHZ, AUX_CH_ALL); //start ADC measurement
        o_ltc6811_pollAdc(); //Wait on ADC measurement (Should be quick)
        error = o_ltc6811_rdaux(0,TOTAL_IC,aux_codes); //Parse ADC measurements
        for (uint8_t j = 0; j < TOTAL_IC; j++) {
            cell_temperatures[j][i*2] = aux_codes[j][0]; //Store temperatures
            uint32_t _cell_temp_mult = (uint32_t)(aux_codes[j][0]) * THERM_V_FRACTION;
            uint32_t _cell_ref_mult = (uint32_t)(aux_codes[j][5]) * 1000;
            if (_cell_temp_mult < _cell_ref_mult) {
                FLAGS |= OVER_TEMP;
                error += 1;
            }
        }
    }
    mux_disable(TOTAL_IC, MUX1_ADDRESS);
    //Iterate through second mux
    for (uint8_t i = 0; i < MUX_CHANNELS; i++) {

        //Changing channel over I2C is going to be tricky
        set_mux_channel(TOTAL_IC, MUX2_ADDRESS, i);
        //_delay_us(50); //TODO: This is a blatant guess

        o_ltc6811_adax(MD_7KHZ_3KHZ , AUX_CH_ALL); //start ADC measurement
        o_ltc6811_pollAdc(); //Wait on ADC measurement (Should be quick)
        error = o_ltc6811_rdaux(0,TOTAL_IC,aux_codes); //Parse ADC measurements
        if (i == 5) { //ignore the thermistor that is shorted low
            aux_codes[3][0] = aux_codes[3][5] - 1;
        }
        for (uint8_t j = 0; j < TOTAL_IC; j++) {
            cell_temperatures[j][i*2 + 1] = aux_codes[j][0]; //Store temperatures
            uint32_t _cell_temp_mult = (uint32_t)(aux_codes[j][0]) * THERM_V_FRACTION;
            uint32_t _cell_ref_mult = (uint32_t)(aux_codes[j][5]) * 1000;
            if (_cell_temp_mult < _cell_ref_mult) {
                FLAGS |= OVER_TEMP;
                error += 1;
            }
        }
    }

    if (error == 0) {
        //upon successful execution clear flags
        FLAGS &= ~OVER_TEMP;
    }

    return error;
}

//SPI functions ////////////////////////////////////////////////////////////////

void init_spi_master(void)
{
    // Setup SPI IO pins (MOSI and SCK)
    DDRB |= _BV(PB1) | _BV(PB7);

    // Enable SPI
    SPCR |=  _BV(SPE) | _BV(MSTR) | _BV(SPR0);

    // Enable alternate SPI (MISO_A, MOSI_A, etc)
    // Take this out if you are using standard pins!
    //MCUCR |= _BV(SPIPS);
}


uint8_t spi_message(uint8_t msg)
{
    // Set message
    SPDR = msg;

    // Wait for transmission to finish
    while(!(SPSR & (1<<SPIF)));

    return SPDR;
}

uint8_t spi_write_array(uint8_t tx_data[], uint8_t tx_len)
{
    for (uint8_t i = 0; i < tx_len; i++)
    {
        spi_message(tx_data[i]);
    }
    return 0;
}

/*
   Writes and read a set number of bytes using the SPI port.
   */
void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
        uint8_t tx_len, //length of the tx data arry
        uint8_t *rx_data,//Input: array that will store the data read by the SPI port
        uint8_t rx_len //Option: number of bytes to be read from the SPI port
        )
{
    uint8_t i;
    for (i = 0; i < tx_len; i++)
    {
        spi_message(tx_Data[i]);
    }

    for (i = 0; i < rx_len; i++)
    {
        rx_data[i] = (uint8_t)spi_message(0x00);
    }

}

//MUX HELPER FUNCTIONS//////////////////////////////////////////////////////////

void set_mux_channel(uint8_t total_ic, uint8_t i2c_address, uint8_t channel)
{
    uint8_t command = 0x08 | channel;
    write_i2c(total_ic, i2c_address, command, 0, 0); //(total_ic, address, command, data, data_length)
}

void mux_disable(uint8_t total_ic, uint8_t i2c_address)
{
    uint8_t command = 0x00;
    write_i2c(total_ic, i2c_address, command, 0, 0); //(total_ic, address, command, data, data_length)
}


//ltc 6804 COMMUNICATION////////////////////////////////////////////////////////

/*!***********************************
  \brief Initializes the configuration array
 **************************************/
void init_cfg(void)
{
    uint16_t uv_val = (UV_THRESHOLD/16)-1;
    uint16_t ov_val = (OV_THRESHOLD/16);
    for (int i = 0; i<TOTAL_IC; i++)
    {
        tx_cfg[i][0] = 0xFC | ADC_OPT;
        tx_cfg[i][1] = (uint8_t)(uv_val&0xFF) ;
        tx_cfg[i][2] = (uint8_t)((ov_val&0x00F)|((uv_val&0xF00)>>8)) ;
        tx_cfg[i][3] = (uint8_t)((ov_val&0xFF0)>>4);
        tx_cfg[i][4] = 0x00 ;
        tx_cfg[i][5] = 0x00 ;

    }

}


/*
   Generic wakeup command to wake isoSPI up out of idle
   */
void wakeup_idle(uint8_t total_ic)
{
    uint8_t i;
    for (i=0; i<total_ic+1; i++)
    {
        PORTB &= ~_BV(PB4); //set CS low
        _delay_us(2); //Guarantees the isoSPI will be in ready mode
        PORTB |= _BV(PB4); //set CS high
    }
}

//Generic wakeup commannd to wake the ltc6813 from sleep
void wakeup_sleep(uint8_t total_ic)
{
    for (int i =0; i<total_ic+1; i++)
    {
        PORTB &= ~_BV(PB4); //set CS low
        _delay_us(300); // Guarantees the ltc6813 will be in standby
        PORTB |= _BV(PB4); //set CS high
    }
}

/*
   Reads configuration registers of a ltc6811 daisy chain
   */
void o_ltc6811_rdcfg(uint8_t total_ic, //Number of ICs in the system
        uint8_t r_config[][8] //A two dimensional array that the function stores the read configuration data.
        )
{
    const uint8_t BYTES_IN_REG = 8;

    uint8_t cmd[4];
    uint8_t *rx_data;
    //int8_t pec_error = 0;
    //uint16_t data_pec;
    //uint16_t received_pec;

    rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));

    cmd[0] = 0x00;
    cmd[1] = 0x02;
    cmd[2] = 0x2b;
    cmd[3] = 0x0A;

    wakeup_idle(total_ic); //This will guarantee that the ltc6811 isoSPI port is awake.

    PORTB &= ~_BV(PB4); //set CS low
    spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into
    PORTB |= _BV(PB4); //set CS high                         //rx_data[] array

    // for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each ltc6811 in the daisy chain and packs the data
    // {
    //   //into the r_config array as well as check the received Config data
    //   //for any bit errors

    //   for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    //   {
    //     r_config[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
    //   }

    //   received_pec = (r_config[current_ic][6]<<8) + r_config[current_ic][7];
    //   data_pec = pec15_calc(6, &r_config[current_ic][0]);
    //   if (received_pec != data_pec)
    //   {
    //     pec_error = -1;
    //   }
    // }

    // free(rx_data);
    // return(pec_error);
}

/*
   This command will write the configuration registers of the ltc6811-1s
   connected in a daisy chain stack. The configuration is written in descending
   order so the last device's configuration is written first.
   */
void o_ltc6811_wrcfg(uint8_t total_ic, //The number of ICs being written to
        uint8_t config[][6] //A two dimensional array of the configuration data that will be written
        )
{
    const uint8_t BYTES_IN_REG = 6;
    const uint8_t CMD_LEN = 4+(8*total_ic);
    uint8_t *cmd;
    uint16_t cfg_pec;
    uint8_t cmd_index; //command counter

    cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));


    cmd[0] = 0x00;
    cmd[1] = 0x01;
    cmd[2] = 0x3d;
    cmd[3] = 0x6e;


    cmd_index = 4;
    for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each ltc6811 in daisy chain, this loops starts with
    {
        // the last IC on the stack. The first configuration written is
        // received by the last IC in the daisy chain

        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
        {
            // current_byte is the byte counter

            cmd[cmd_index] = config[current_ic-1][current_byte];            //adding the config data to the array to be sent
            cmd_index = cmd_index + 1;
        }

        cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);   // calculating the PEC for each ICs configuration register data
        cmd[cmd_index] = (uint8_t)(cfg_pec >> 8);
        cmd[cmd_index + 1] = (uint8_t)cfg_pec;
        cmd_index = cmd_index + 2;
    }


    wakeup_idle(total_ic);                                 //This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.

    PORTB &= ~_BV(PB4); //set CS low
    spi_write_array(cmd, CMD_LEN);
    PORTB |= _BV(PB4); //set CS high
    free(cmd);
}

//This sets a discharge bit in the configuration register. Cell and IC are one-indexed
void enable_discharge(uint8_t ic, uint8_t cell)
{
    if (cell<9)
    {
        tx_cfg[ic-1][4] |= (1<<(cell-1));
    }
    else if (cell < 13)
    {
        tx_cfg[ic-1][5] |= (1<<(cell-9));
    }
    discharge_status[ic-1] |= (1 << (cell-1));
}

//This unsets a discharge bit in the configuration register
void disable_discharge(uint8_t ic, uint8_t cell)
{
    if (cell<9)
    {
        tx_cfg[ic-1][4] &= ~(1<<(cell-1));
    }
    else if (cell < 13)
    {
        tx_cfg[ic-1][5] &= ~(1<<(cell-9));
    }
    discharge_status[ic-1] &= ~(1 << (cell-1));
}

//This function will block operation until the ADC has finished it's conversion
uint32_t o_ltc6811_pollAdc(void)
{
    uint32_t counter = 0;
    uint8_t finished = 0;
    uint8_t current_time = 0;
    uint8_t cmd[4];
    uint16_t cmd_pec;


    cmd[0] = 0x07;
    cmd[1] = 0x14;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.

    PORTB &= ~_BV(PB4); //set CS low
    spi_write_array(cmd,4);

    while ((counter<200000)&&(finished == 0))
    {
        current_time = spi_message(0xFF);
        if (current_time>0)
        {
            finished = 1;
        }
        else
        {
            counter = counter + 10;
        }
    }

    PORTB |= _BV(PB4); //set CS high

    return counter;
}

/*
   Starts cell voltage conversion
   */
void o_ltc6811_adcv(
        uint8_t MD, //ADC Mode
        uint8_t DCP, //Discharge Permit
        uint8_t CH //Cell Channels to be measured
        )
{
    uint8_t cmd[4];
    uint16_t cmd_pec;
    uint8_t md_bits;

    md_bits = (MD & 0x02) >> 1;
    cmd[0] = md_bits + 0x02;
    md_bits = (MD & 0x01) << 7;
    cmd[1] =  md_bits + 0x60 + (DCP<<4) + CH;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);


    //wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
    PORTB &= ~_BV(PB4); //set CS low
    spi_write_array(cmd, 4);
    PORTB |= _BV(PB4); //set CS high

}

/*
 * Reads and parses the ltc6811 cell voltage registers.
 */
uint8_t o_ltc6811_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
        uint8_t total_ic, // the number of ICs in the system
        uint16_t cell_codes[][CELL_CHANNELS] // Array of the parsed cell codes
        )
{

    const uint8_t NUM_RX_BYT = 8;
    const uint8_t BYT_IN_REG = 6;
    const uint8_t CELL_IN_REG = 3;
    const uint8_t NUM_CV_REG = 4;

    uint8_t *cell_data;
    uint8_t pec_error = 0;
    uint16_t parsed_cell;
    uint16_t received_pec;
    uint16_t data_pec;
    uint8_t data_counter=0; //data counter
    cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

    if (reg == 0)
    {

        for (uint8_t cell_reg = 1; cell_reg<NUM_CV_REG+1; cell_reg++)                   //executes once for each of the ltc6811 cell voltage registers
        {
            data_counter = 0;
            o_ltc6811_rdcv_reg(cell_reg, total_ic,cell_data );                //Reads a single Cell voltage register

            for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every ltc6811 in the daisy chain
            {
                // current_ic is used as the IC counter

                for (uint8_t current_cell = 0; current_cell<CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
                {
                    // loops once for each of the 3 cell voltage codes in the register

                    parsed_cell = cell_data[data_counter] + (cell_data[data_counter + 1] << 8);//Each cell code is received as two bytes and is combined to
                    // create the parsed cell voltage code

                    cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = parsed_cell;
                    data_counter = data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
                    //must increment by two for each parsed cell code
                }

                received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
                //after the 6 cell voltage data bytes
                data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
                if (received_pec != data_pec)
                {
                    pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
                    //are detected in the serial data
                }
                data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
                //must be incremented by 2 bytes to point to the next ICs cell voltage data
            }
        }
    }

    else
    {

        o_ltc6811_rdcv_reg(reg, total_ic,cell_data);
        for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)        // executes for every ltc6811 in the daisy chain
        {
            // current_ic is used as the IC counter

            for (uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++)  // This loop parses the read back data into cell voltages, it
            {
                // loops once for each of the 3 cell voltage codes in the register

                parsed_cell = cell_data[data_counter] + (cell_data[data_counter+1]<<8); //Each cell code is received as two bytes and is combined to
                // create the parsed cell voltage code

                cell_codes[current_ic][current_cell + ((reg - 1) * CELL_IN_REG)] = 0x0000FFFF & parsed_cell;
                data_counter= data_counter + 2;                       //Because cell voltage codes are two bytes the data counter
                //must increment by two for each parsed cell code
            }

            received_pec = (cell_data[data_counter] << 8 )+ cell_data[data_counter + 1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
            //after the 6 cell voltage data bytes
            data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
            if (received_pec != data_pec)
            {
                pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
                //are detected in the serial data
            }
            data_counter= data_counter + 2;                       //Because the transmitted PEC code is 2 bytes long the data_counter
            //must be incremented by 2 bytes to point to the next ICs cell voltage data
        }
    }


    free(cell_data);
    return(pec_error);
}

//Read the raw data from the ltc6811 cell voltage register
void o_ltc6811_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
        uint8_t total_ic, //the number of ICs in the
        uint8_t *data //An array of the unparsed cell codes
        )
{
    const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
    uint8_t cmd[4];
    uint16_t cmd_pec;

    if (reg == 1)     //1: RDCVA
    {
        cmd[1] = 0x04;
        cmd[0] = 0x00;
    }
    else if (reg == 2) //2: RDCVB
    {
        cmd[1] = 0x06;
        cmd[0] = 0x00;
    }
    else if (reg == 3) //3: RDCVC
    {
        cmd[1] = 0x08;
        cmd[0] = 0x00;
    }
    else if (reg == 4) //4: RDCVD
    {
        cmd[1] = 0x0A;
        cmd[0] = 0x00;
    }
    else if (reg == 5) //4: RDCVE
    {
        cmd[1] = 0x09;
        cmd[0] = 0x00;
    }
    else if (reg == 6) //4: RDCVF
    {
        cmd[1] = 0x0B;
        cmd[0] = 0x00;
    }


    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);


    PORTB &= ~_BV(PB4); //set CS low
    spi_write_read(cmd,4,data,(REG_LEN*total_ic));
    PORTB |= _BV(PB4); //set CS high


}

//Start a GPIO and Vref2 Conversion
void o_ltc6811_adax(
        uint8_t MD, //ADC Mode
        uint8_t CHG //GPIO Channels to be measured)
    )
{
    uint8_t cmd[4];
    uint16_t cmd_pec;
    uint8_t md_bits;

    md_bits = (MD & 0x02) >> 1;
    cmd[0] = md_bits + 0x04;
    md_bits = (MD & 0x01) << 7;
    cmd[1] = md_bits + 0x60 + CHG ;

    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
    PORTB &= ~_BV(PB4); //set CS low
    spi_write_array(cmd,4);
    PORTB |= _BV(PB4); //set CS low

}

/*
   The function is used
   to read the  parsed GPIO codes of the ltc6811. This function will send the requested
   read commands parse the data and store the gpio voltages in aux_codes variable
   */
int8_t o_ltc6811_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
        uint8_t total_ic,//the number of ICs in the system
        uint16_t aux_codes[][AUX_CHANNELS]//A two dimensional array of the gpio voltage codes.
        )
{

    const uint8_t NUM_RX_BYT = 8;
    const uint8_t BYT_IN_REG = 6;
    const uint8_t GPIO_IN_REG = 3;
    const uint8_t NUM_GPIO_REG = 2;
    uint8_t *data;
    uint8_t data_counter = 0;
    int8_t pec_error = 0;
    uint16_t parsed_aux;
    uint16_t received_pec;
    uint16_t data_pec;
    data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

    if (reg == 0)
    {

        for (uint8_t gpio_reg = 1; gpio_reg<NUM_GPIO_REG+1; gpio_reg++)                 //executes once for each of the ltc6811 aux voltage registers
        {
            data_counter = 0;
            o_ltc6811_rdaux_reg(gpio_reg, total_ic,data);                 //Reads the raw auxiliary register data into the data[] array

            for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every ltc6811 in the daisy chain
            {
                // current_ic is used as the IC counter


                for (uint8_t current_gpio = 0; current_gpio< GPIO_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
                {
                    // loops once for each of the 3 gpio voltage codes in the register

                    parsed_aux = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
                    // create the parsed gpio voltage code

                    aux_codes[current_ic][current_gpio +((gpio_reg-1)*GPIO_IN_REG)] = parsed_aux;
                    data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter
                    //must increment by two for each parsed gpio voltage code

                }

                received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
                //after the 6 gpio voltage data bytes
                data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
                if (received_pec != data_pec)
                {
                    pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
                    //are detected in the received serial data
                }

                data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
                //must be incremented by 2 bytes to point to the next ICs gpio voltage data
            }


        }

    }
    else
    {

        o_ltc6811_rdaux_reg(reg, total_ic, data);
        for (int current_ic = 0 ; current_ic < total_ic; current_ic++)            // executes for every ltc6811 in the daisy chain
        {
            // current_ic is used as an IC counter


            for (int current_gpio = 0; current_gpio<GPIO_IN_REG; current_gpio++)    // This loop parses the read back data. Loops
            {
                // once for each aux voltage in the register

                parsed_aux = (data[data_counter] + (data[data_counter+1]<<8));        //Each gpio codes is received as two bytes and is combined to
                // create the parsed gpio voltage code
                aux_codes[current_ic][current_gpio +((reg-1)*GPIO_IN_REG)] = parsed_aux;
                data_counter=data_counter+2;                      //Because gpio voltage codes are two bytes the data counter
                //must increment by two for each parsed gpio voltage code
            }

            received_pec = (data[data_counter]<<8) + data[data_counter+1];         //The received PEC for the current_ic is transmitted as the 7th and 8th
            //after the 6 gpio voltage data bytes
            data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
            if (received_pec != data_pec)
            {
                pec_error = -1;                               //The pec_error variable is simply set negative if any PEC errors
                //are detected in the received serial data
            }

            data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
            //must be incremented by 2 bytes to point to the next ICs gpio voltage data
        }
    }
    free(data);
    return (pec_error);
}

/*
   The function reads a single GPIO voltage register and stores thre read data
   in the *data point as a byte array. This function is rarely used outside of
   the ltc6811_rdaux() command.
   */
void o_ltc6811_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
        uint8_t total_ic, //The number of ICs in the system
        uint8_t *data //Array of the unparsed auxiliary codes
        )
{
    const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
    uint8_t cmd[4];
    uint16_t cmd_pec;


    if (reg == 1)     //Read back auxiliary group A
    {
        cmd[1] = 0x0C;
        cmd[0] = 0x00;
    }
    else if (reg == 2)  //Read back auxiliary group B
    {
        cmd[1] = 0x0e;
        cmd[0] = 0x00;
    }
    else if (reg == 3)  //Read back auxiliary group B
    {
        cmd[1] = 0x0D;
        cmd[0] = 0x00;
    }
    else if (reg == 4)  //Read back auxiliary group B
    {
        cmd[1] = 0x0F;
        cmd[0] = 0x00;
    }
    else          //Read back auxiliary group A
    {
        cmd[1] = 0x0C;
        cmd[0] = 0x00;
    }

    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);


    //wakeup_idle(total_ic); //This will guarantee that the ltc6811 isoSPI port is awake, this command can be removed.

    PORTB &= ~_BV(PB4); //set CS low
    spi_write_read(cmd,4,data,(REG_LEN*total_ic));
    PORTB |= _BV(PB4); //set CS high

}

/*
   Writes the COMM registers of a ltc6811 daisy chain
   */
void wrcomm(uint8_t total_ic, //The number of ICs being written to
        uint8_t comm[][6] //A two dimensional array of the comm data that will be written
        )
{
    const uint8_t BYTES_IN_REG = 6;
    const uint8_t CMD_LEN = 4+(8*total_ic);
    uint8_t *cmd;
    uint16_t comm_pec;
    uint16_t cmd_pec;
    uint8_t cmd_index; //command counter

    cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

    cmd[0] = 0x07;
    cmd[1] = 0x21;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    cmd_index = 4;
    for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each ltc6811 in daisy chain, this loops starts with
    {
        // the last IC on the stack. The first configuration written is
        // received by the last IC in the daisy chain

        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
        {
            // current_byte is the byte counter

            cmd[cmd_index] = comm[current_ic-1][current_byte];            //adding the config data to the array to be sent
            cmd_index = cmd_index + 1;
        }
        comm_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &comm[current_ic-1][0]);    // calculating the PEC for each ICs configuration register data
        cmd[cmd_index] = (uint8_t)(comm_pec >> 8);
        cmd[cmd_index + 1] = (uint8_t)comm_pec;
        cmd_index = cmd_index + 2;
    }

    wakeup_idle(total_ic);                                 //This will guarantee that the ltc6811 isoSPI port is awake.This command can be removed.
    PORTB &= ~_BV(PB4); //set CS low
    spi_write_array(cmd, CMD_LEN);
    PORTB |= _BV(PB4); //set CS high
    free(cmd);
}

/*
   Shifts data in COMM register out over ltc6811 SPI/I2C port
   */
void stcomm(void)
{

    uint8_t cmd[4];
    uint16_t cmd_pec;

    cmd[0] = 0x07;
    cmd[1] = 0x23;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
    PORTB &= ~_BV(PB4);
    spi_write_array(cmd, 4);         //Read the configuration data of all ICs on the daisy chain into
    for (int i = 0; i<9; i++)
    {
        spi_message(0xFF);
    }
    PORTB |= _BV(PB4);

}

/*
   Reads COMM registers of a ltc6811 daisy chain
   */
int8_t rdcomm(uint8_t total_ic, //Number of ICs in the system
        uint8_t r_comm[][8] //A two dimensional array that the function stores the read configuration data.
        )
{
    const uint8_t BYTES_IN_REG = 8;

    uint8_t cmd[4];
    uint8_t *rx_data;
    int8_t pec_error = 0;
    uint16_t cmd_pec;
    uint16_t data_pec;
    uint16_t received_pec;

    rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));

    cmd[0] = 0x07;
    cmd[1] = 0x22;
    cmd_pec = pec15_calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);

    //wakeup_idle (); //This will guarantee that the ltc6811 isoSPI port is awake. This command can be removed.
    PORTB &= ~_BV(PB4); //set CS low
    spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into
    PORTB |= _BV(PB4); //set CS high

    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each ltc6811 in the daisy chain and packs the data
    {
        //into the r_comm array as well as check the received Config data
        //for any bit errors
        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
        {
            r_comm[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
        }
        received_pec = (r_comm[current_ic][6]<<8) + r_comm[current_ic][7];
        data_pec = pec15_calc(6, &r_comm[current_ic][0]);
        if (received_pec != data_pec)
        {
            pec_error = -1;
        }
    }

    free(rx_data);
    return(pec_error);
}

void write_i2c(uint8_t total_ic, uint8_t address, uint8_t command, uint8_t *data, uint8_t data_len)
{
    //Serial.println("In write_i2c.");
    uint8_t START = 0x60;
    //    uint8_t STOP = 0x01;
    //    uint8_t ACK = 0x00;
    uint8_t NACK = 0x08;
    uint8_t BLANK = 0x00;
    uint8_t NO_TRANSMIT = 0x70;
    uint8_t NACK_STOP = 0x09;


    uint8_t loop_count;
    uint8_t remainder = 0;
    uint8_t transmitted_bytes = 0;
    uint8_t data_counter = 0;
    uint8_t comm[total_ic][6];
    // uint8_t rx_comm[1][8]; // TODO: Uncomment if used
    if (((data_len)%3) == 0)
    {
        loop_count = ((data_len)/3);
    }
    else
    {
        loop_count = ((data_len)/3);
        remainder = data_len - (loop_count*3);
        loop_count++;
    }

    address = address << 1; // convert 7 bit address to 8 bits

    for(uint8_t i=0; i<total_ic; i++){
        comm[i][0] = START;//NO_TRANSMIT; //
        comm[i][1] = NACK_STOP;//BLANK ; //
        comm[i][2] = START | (address >> 4); //
        comm[i][3] = (address<<4) | NACK ; //
        comm[i][4] = BLANK | (command >>4);
        comm[i][5] = (command<<4) | NACK;
    }

    if (loop_count == 0) { // if there is no data, free up the bus
        for(uint8_t i=0; i<total_ic; i++){
            comm[i][5] = (command<<4) | NACK_STOP;
        }
        //Serial.println("Adding NACK_STOP since there is no data");
    }

    //  Serial.print("comm: ");
    //  for(uint8_t i=0; i < 6; i++){
    //    Serial.print(comm[0][i],HEX);
    //    Serial.print(", ");
    //  }
    //  Serial.println();



    wrcomm(total_ic,comm);
    stcomm();
    //rdcomm(total_ic,rx_comm);
    //EXT_LED_PORT |= _BV(LED_ORANGE);

    //  Serial.print("rx_comm: ");
    //  for(uint8_t i=0; i < 6; i++){
    //    Serial.print(rx_comm[0][i],HEX);
    //    Serial.print(", ");
    //  }
    //  Serial.println();
    //
    //  Serial.println("sent command");

    transmitted_bytes = 0;
    for (uint8_t i=0; i<loop_count; i++)
    {
        if ((i == (loop_count-1)) && (remainder != 0)) //need to pad becuase we don't have multiple of 3 data bytes
        {
            for (uint8_t k=0; k<remainder; k++)
            {
                comm[0][transmitted_bytes] = BLANK + (data[data_counter] >> 4); //
                if (k!=(remainder-1))comm[0][transmitted_bytes+1] = (data[data_counter]<<4) | NACK ; //
                else comm[0][transmitted_bytes+1] = (data[data_counter]<<4) | NACK_STOP;
                data_counter++;
                transmitted_bytes = transmitted_bytes +2;
            }
            for (uint8_t k=0; k<(3-remainder); k++)
            {
                comm[0][transmitted_bytes] = NO_TRANSMIT; //
                comm[0][transmitted_bytes+1] =  BLANK ; //
                transmitted_bytes = transmitted_bytes + 2;
            }
            for(uint8_t i=1; i<total_ic; i++){
                for(uint8_t j=0; j<6; j++){
                    comm[i][j] = comm[0][j];
                }
            }
            wrcomm(1,comm);
            stcomm();
        }
        else                                      //don't need to pad because we have a multiple of 3 data bytes
        {
            for (uint8_t k=0; k<3; k++)
            {
                comm[0][k*2] = BLANK + (data[data_counter] >> 4); //
                if (k!=2){comm[0][(k*2)+1] = (data[data_counter]<<4) | NACK ;} //
                else if(remainder!=0){ comm[0][(k*2)+1] = (data[data_counter]<<4) | NACK ;}
                else { comm[0][(k*2)+1] = (data[data_counter]<<4) | NACK_STOP ;}
                data_counter++;
            }
            for(uint8_t i=1; i<total_ic; i++){
                for(uint8_t j=0; j<6; j++){
                    comm[i][j] = comm[0][j];
                }
            }
            wrcomm(1,comm);
            stcomm();
        }
    }

}

/* TODO: Remove if never used
void read_i2c( uint8_t total_ic , uint8_t address, uint8_t command, uint8_t *data, uint8_t data_len)
{
    uint8_t START = 0x60;
    uint8_t ACK = 0x00;
    uint8_t NACK = 0x08;
    uint8_t BLANK = 0x00;
    uint8_t NO_TRANSMIT = 0x70;
    uint8_t NACK_STOP = 0x09;

    uint8_t loop_count;
    uint8_t transmitted_bytes =0;
    uint8_t remainder=0;
    uint8_t data_counter = 0;
    uint8_t rx_data = 0;

    uint8_t comm[1][6];
    uint8_t rx_comm[1][8];
    if (((data_len)%3)==0)
    {
        loop_count = ((data_len)/3);
    }
    else
    {
        loop_count = ((data_len)/3);
        remainder = data_len - (loop_count*3);
        loop_count++;
    }

    address = address << 1; //convert 7 bit address to 8 bits

    comm[0][0] = START + (address >> 4); //
    comm[0][1] = ((address)<<4) + NACK ; //
    comm[0][2] = BLANK + (command >>4);
    comm[0][3] = (command<<4) + NACK;
    comm[0][4] = START + (address >> 4); //
    comm[0][5] = (address<<4) + 0x10 + ACK ; //

    for(uint8_t i=1; i<total_ic; i++){
        for(uint8_t j=0; j<6; j++){
            comm[i][j] = comm[0][j];
        }
    }
    wrcomm(total_ic,comm);
    rdcomm(total_ic,rx_comm);
    stcomm();

    for (int i =0; i<loop_count; i++)
    {
        if ((i == (loop_count-1)) && (remainder != 0))
        {
            for (int k=0; k<remainder; k++)
            {
                comm[0][transmitted_bytes] = BLANK + 0x0F; //
                if (k!=(remainder-1))comm[0][transmitted_bytes+1] = 0xF0 + ACK ; //
                else comm[0][transmitted_bytes+1] = 0xF0 + NACK_STOP;
                data_counter++;
                transmitted_bytes = transmitted_bytes +2;
            }
            for (int k=0; k<(3-remainder); k++)
            {
                comm[0][transmitted_bytes] = NO_TRANSMIT; //
                comm[0][transmitted_bytes+1] =  ACK ; //
                transmitted_bytes = transmitted_bytes + 2;
            }

            for(uint8_t i=1; i<total_ic; i++){
                for(uint8_t j=0; j<6; j++){
                    comm[i][j] = comm[0][j];
                }
            }
            wrcomm(1,comm);
            stcomm();
            rdcomm(1,rx_comm);
            data_counter=0;
            for (int k=0; k<remainder; k++)
            {
                data[rx_data] = ((rx_comm[0][data_counter]&0x0F)<<4)|((rx_comm[0][data_counter+1]&0xF0)>>4); //
                //Serial.println(data[rx_data],HEX);
                rx_data++;
                data_counter = data_counter+2;
            }
        }
        else
        {
            for (uint8_t k=0; k<3; k++)
            {
                comm[0][(k*2)] = BLANK + 0x0F; //
                if (k!=2)comm[0][(k*2)+1] = 0xF0 + ACK ; //
                else if(remainder!=0){ comm[0][(k*2)+1] = 0xF0 | ACK ;}
                else { comm[0][(k*2)+1] = 0xF0 | NACK_STOP ;}
                data_counter++;
            }
            for(uint8_t i=1; i<total_ic; i++){
                for(uint8_t j=0; j<6; j++){
                    comm[i][j] = comm[0][j];
                }
            }
            wrcomm(1,comm);
            stcomm();
            rdcomm(1,rx_comm);
            data_counter =0;
            for (int k=0; k<3; k++)
            {
                data[rx_data] = ((rx_comm[0][data_counter]&0x0F)<<4) | ((rx_comm[0][data_counter+1]&0xF0)>>4); //
                rx_data++;
                data_counter = data_counter+2;
            }
        }
    }

}
*/


/*
   Calculates  and returns the CRC15
   */
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
        uint8_t *data //Array of data that will be used to calculate  a PEC
        )
{
    uint16_t remainder,addr;

    remainder = 16;//initialize the PEC
    for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
    {
        addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
        remainder = (remainder<<8)^pgm_read_word_near(crc15Table+addr);
    }
    return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}
