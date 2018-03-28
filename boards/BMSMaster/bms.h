/*----- Macro Definitions -----*/
#define IMD_SENSE_PIN           PC7 //TODO change-->
#define IMD_SENSE_PORT          PORTC

/* FLAGS */
#define BSPD_CURRENT            0b00000001
#define READ_VALS_FLAG          0b00000001
#define UNDER_VOLTAGE           0b00000001
#define OVER_VOLTAGE            0b00000001
#define SOFT_OVER_VOLTAGE       0b00000001
#define OVER_TEMP               0b00000001
#define OPEN_SHUTDOWN           0b00000001
#define AIRS_CLOSED             0b00000001
#define IMD_TRIPPED             0b00000001

/* LEDs */
#define LED_1                   PB3 //TODO change -->
#define LED_2                   PB4
#define LED_3                   PB5
#define LED_1_PORT              PORTB
#define LED_2_PORT              PORTB
#define LED_3_PORT              PORTB

#define RJ45_LED_ORANGE         PC1
#define RJ45_LED_GREEN          PC2
#define RJ45_ORANGE_PORT        PORTC
#define RJ45_GREEN_PORT         PORTC

/* I/O */
#define RELAY_PIN               PB2 //TODO change -->
#define RELAY_PORT              PORTB
#define FAN_PIN                 PC1 //TODO change -->
#define FAN_PORT                PORTC

/* MUX defs */
#define MUX_CHANNELS            6 //TODO change -->
#define MUX_1_ADDRESS           0x49 // TODO WHAT DO THESE MEAN?
#define MUX_2_ADDRESS           0x48

/* LTC68xx defs */
#define TOTAL_IC                6 //TODO change -->

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
