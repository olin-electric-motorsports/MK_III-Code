#include "SPI_with_EEPROM.h"

void initSPI_master(uint8_t mode, uint8_t prescaler){
  //Set SS, SCK(PB5) and MOSI(PB3) to output
  DDRB |= _BV(PB5) | _BV(PB3) | _BV(PB2);
  //Set MISO(PB4) and then AVR slave-select to input with internal pull-up
  DDRB &= ~_BV(PB4);
  PORTB |= _BV(PB4);

  //Modes of Operations for SPI, Phases and Polarities
  //For Atmega328, see P219 of its datasheet for reference
  switch(mode){
    case 0:
      SPCR &= ~_BV(CPOL) & ~_BV(2);
    case 1:
      SPCR &= ~_BV(CPOL);
      SPCR |= _BV(2);
    case 2:
      SPCR &= ~_BV(2);
      SPCR |= _BV(CPOL);
    case 3:
      SPCR |= _BV(CPOL) | _BV(2);
  }

  switch(prescaler){
    case 2:
      SPSR |= 1 << SPI2X;
    case 4:
      break;
    case 8:
      SPCR |= (1 << SPR0);
      SPSR |= 1 << SPI2X;
    case 16:
      SPCR |= (1 << SPR0);
    case 32:
      SPSR |= (1 << SPI2X);
      SPCR |= (1 << SPR1);
    case 64:
      SPCR |= (1 << SPR1);
    case 128:
      SPCR |= (1 << SPR0) | (1 << SPR1);
  }

  //Master Mode, Enable SPI
  SPCR |= _BV(MSTR) | _BV(SPE);
}


void initSPI_slave(uint8_t mode){
  //Set Slave Select(SS, PB2), SCK(PB5) and MOSI(PB3) to input
  DDRB &= ~(_BV(PB2)) & ~(_BV(PB4)) & ~(_BV(PB3));
  //Set MISO(PB4) to output and then AVR slave-select to input with internal pull-up
  DDRB |= _BV(PB4);
  PORTB |= _BV(PB2);

  switch(mode){
    case 0:
      SPCR &= ~_BV(CPOL) & ~_BV(2);
    case 1:
      SPCR &= ~_BV(CPOL);
      SPCR |= _BV(2);
    case 2:
      SPCR &= ~_BV(2);
      SPCR |= _BV(CPOL);
    case 3:
      SPCR |= _BV(CPOL) | _BV(2);
  }

  //Master Mode, Enable SPI
  SPCR &= ~(_BV(MSTR));
  SPCR = | _BV(SPE);
}


void SPI_tradeByte(uint8_t byte){
  SPDR = byte;
  loop_until_bit_is_set(SPSR, SPIF);
  //Loop until SPIF0 is set
  //while((SPSR & _BV(SPIF)) & _BV(SPIF)){};
}

void EEPROM_address(uint16_t address){
  //Bit-shift 8 to the left to transmit the 8 MSBs
  SPI_tradeByte((uint8_t) (address >> 8));
  //Transmit the 8 LSBs
  SPI_tradeByte((uint8_t) address);
}

uint8_t EEPROMM_readStatus(){
  //Select the slave device
  SLAVE_SELECT;
  //Send EEPROM Status Register READ Instruction
  SPI_tradeByte(EEPROM_RDSR);
  SPI_tradeByte(0);
  //Deselect the slave device
  SLAVE_DESELECT;
  return SPDR;
}

uint8_t EEPROM_readByte(uint16_t address){
  //Select the slave device
  SLAVE_SELECT;
  //Send EEPROM READ Instruction
  SPI_tradeByte(EEPROM_READ);
  //Send the address to read from
  EEPROM_address(address);
  SPI_tradeByte(0);
  //Deselect the slave device
  SLAVE_DESELECT;
  return SPDR;
}

void EEPROM_writeEnable(){
  SLAVE_SELECT;
  SPI_tradeByte(EEPROM_WREN);
  SLAVE_DESELECT;
}

void EEPROM_writeDisable(){
  SLAVE_SELECT;
  SPI_tradeByte(EEPROM_WRDI);
  SLAVE_DESELECT;
}

void EEPROM_writeIn(){
  SLAVE_DESELECT;
  //loop until write-in is completed, i.e. when WIP is cleared
  while((EEPROMM_readStatus() & (0b00000001))){};
}

void EEPROM_writeSingleByte(uint16_t address, uint8_t byte){
  EEPROM_writeEnable();
  // Select the slave device
  SLAVE_SELECT;
  // Send EEPROM WRITE Instruction
  SPI_tradeByte(EEPROM_WRITE);
  // Send the address to write
  EEPROM_address(address);
  SPI_tradeByte(byte);
  // loop until the byte has been written
  EEPROM_writeIn();
}

void EEPROM_writeMax64Byte(uint16_t address, uint8_t byte){
  //Enable Write (outside this function)
  // Select the slave device (outside this function)
  // Send EEPROM WRITE Instruction
  SPI_tradeByte(EEPROM_WRITE);
  // Send the address to write
  EEPROM_address(address);
  SPI_tradeByte(byte);
}

//const char* EEPROM_readString(uint16_t address, size_t numByteToRead = 1);
void EEPROM_readStringToSerial(uint16_t address, uint16_t numByteToRead){
  // Select the slave device
  SLAVE_SELECT;
  // Send EEPROM READ Instruction
  SPI_tradeByte(EEPROM_READ);
  // Send the address to read from
  EEPROM_address(address);
  uint16_t address_count;
  uint16_t address_stop = (numByteToRead+address);
  for(address_count = address; address_count < address_stop; address_count++){
    // Send the address to read from
    SPI_tradeByte(0);
    Serial_transmitByte(SPDR);
    address++;
  }
  // Deselect the slave device
  SLAVE_DESELECT;
}

void EEPROM_writeString(uint16_t address, const char myString[]){
  EEPROM_writeEnable();
  SLAVE_SELECT;
  // Send EEPROM WRITE Instruction
  SPI_tradeByte(EEPROM_WRITE);
  // Send the address to write
  EEPROM_address(address);
  // Send the characters in myString one by one
  uint8_t i = 0;
  while(myString[i]){
    SPI_tradeByte(myString[i]);
    i++;
    // Break if more than 64 bytes are present
    if(i > 64){
      break;
    }
  }
  // Let EEPROM flash the bytes in
  EEPROM_writeIn();
}

void EEPROM_clearAll(){
  uint8_t byte_count;
  uint16_t current_address = 0;
  Serial_printString("Clearing EEPROM now begins. This might take a moment...\n");
  while(current_address <= EEPROM_MAX_BYTE){
    EEPROM_writeEnable();
    SLAVE_SELECT;
    EEPROM_address(current_address);
    for(byte_count = 0; byte_count < 64; byte_count++){
      SPI_tradeByte(0);
    }
    EEPROM_writeIn();
    current_address += 64;
  }
  Serial_printString("Clearing EEPROM successful...\n");
}
