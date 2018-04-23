int main(void);

void transmit_voltages(void);
void transmit_temperatures(void);
void transmit_discharge_status(void);

void init_read_timer(void);
void init_fan_pwm(uint8_t duty_cycle);
void init_cfg(void);

uint8_t read_all_voltages(void);
uint8_t read_all_temperatures(void);

void init_spi_master(void);
uint8_t spi_message(uint8_t msg);
uint8_t spi_write_array(uint8_t tx_data[], uint8_t tx_len);
void spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t* rx_data, uint8_t rx_len);

void set_mux_channel(uint8_t total_ic, uint8_t i2c_address, uint8_t channel);
void mux_disable(uint8_t total_ic, uint8_t i2c_address);

void init_cfg(void);

void wakeup_idle(uint8_t total_ic);
void wakeup_sleep(uint8_t total_ic);

void o_ltc6811_rdcfg(uint8_t total_ic, //Number of ICs in the system
                     uint8_t r_config[][8] //A two dimensional array that the function stores the read configuration data.
                 );
void o_ltc6811_wrcfg(uint8_t total_ic, //The number of ICs being written to
                uint8_t config[][6] //A two dimensional array of the configuration data that will be written
              );
void enable_discharge(uint8_t ic, uint8_t cell);
void disable_discharge(uint8_t ic, uint8_t cell);
uint32_t o_ltc6811_pollAdc(void);
void o_ltc6811_adcv(
  uint8_t MD, //ADC Mode
  uint8_t DCP, //Discharge Permit
  uint8_t CH //Cell Channels to be measured
);
uint8_t o_ltc6811_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // the number of ICs in the system
                     uint16_t cell_codes[][CELL_CHANNELS] // Array of the parsed cell codes
                 );
void o_ltc6811_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                   uint8_t total_ic, //the number of ICs in the
                   uint8_t *data //An array of the unparsed cell codes
                );
void o_ltc6811_adax(
  uint8_t MD, //ADC Mode
  uint8_t CHG //GPIO Channels to be measured)
);
int8_t o_ltc6811_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
                 uint8_t total_ic,//the number of ICs in the system
                 uint16_t aux_codes[][AUX_CHANNELS]//A two dimensional array of the gpio voltage codes.
             );
void o_ltc6811_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
                    uint8_t total_ic, //The number of ICs in the system
                    uint8_t *data //Array of the unparsed auxiliary codes
                );

void o_ltc6811_wrcfg(uint8_t total_ic, //The number of ICs being written to
                   uint8_t config[][6] //A two dimensional array of the configuration data that will be written
                  );

uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                uint8_t *data //Array of data that will be used to calculate  a PEC
            );

const uint16_t crc15Table[256] PROGMEM = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
                               0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
                               0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
                               0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
                               0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
                               0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
                               0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
                               0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
                               0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
                               0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
                               0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
                               0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
                               0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
                               0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
                               0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
                               0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
                               0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
                               0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
                               0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
                               0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
                               0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
                               0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
                               0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
                                         };

void wrcomm(uint8_t total_ic, //The number of ICs being written to
uint8_t comm[][6] //A two dimensional array of the comm data that will be written
);
void stcomm(void);
int8_t rdcomm(uint8_t total_ic, //Number of ICs in the system
    uint8_t r_comm[][8] //A two dimensional array that the function stores the read configuration data.
);
void write_i2c(uint8_t total_ic, uint8_t address, uint8_t command, uint8_t *data, uint8_t data_len);
void read_i2c( uint8_t total_ic , uint8_t address, uint8_t command, uint8_t *data, uint8_t data_len);
