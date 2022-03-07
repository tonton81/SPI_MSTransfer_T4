#include <SPI_MSTransfer_MASTER.h>
#include "Arduino.h"
#include "SPI.h"


SPI_MSTransfer_MASTER_FUNC SPI_MSTransfer_MASTER_OPT::SPI_MSTransfer_MASTER() {
}


SPI_MSTransfer_MASTER_FUNC void SPI_MSTransfer_MASTER_OPT::begin() {
#if defined(__IMXRT1062__)
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; /* LPSPI4 SDI (MISO) */
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; /* LPSPI4 SDO (MOSI) */
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; /* LPSPI4 SCK (CLK) */
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; /* LPSPI4 PCS0 (CS) */
#endif
}


SPI_MSTransfer_MASTER_FUNC void SPI_MSTransfer_MASTER_OPT::spi_assert() {
  port->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
  ::digitalWriteFast(cs_pin, LOW);
}


SPI_MSTransfer_MASTER_FUNC void SPI_MSTransfer_MASTER_OPT::spi_deassert() {
  ::digitalWriteFast(cs_pin, HIGH);
  port->endTransaction();
}


SPI_MSTransfer_MASTER_FUNC void SPI_MSTransfer_MASTER_OPT::pinMode(uint8_t pin, uint8_t state) {
  spi_assert();
  port->transfer16(0xDEAD);
  port->transfer16(slave_ID);
  port->transfer16(1);
  port->transfer16(0x1012);
  port->transfer16(((uint16_t)(pin << 8) | state));
  port->transfer16(((uint16_t)(pin << 8) | state)); /* can't checksum one word, so resend as checksum */
  for ( uint16_t i = 0, result = 0; i < 10; i++ ) {
    result = port->transfer16(0xFFFF);
    if ( result == 0xA5A5 ) break;
  }
  spi_deassert();
}


SPI_MSTransfer_MASTER_FUNC void SPI_MSTransfer_MASTER_OPT::digitalWrite(uint8_t pin, bool state) {
  spi_assert();
  port->transfer16(0xDEAD);
  port->transfer16(slave_ID);
  port->transfer16(1);
  port->transfer16(0x1010);
  port->transfer16(((uint16_t)(pin << 8) | state));
  port->transfer16(((uint16_t)(pin << 8) | state)); /* can't checksum one word, so resend as checksum */
  for ( uint16_t i = 0, result = 0; i < 10; i++ ) {
    result = port->transfer16(0xFFFF);
    if ( result == 0xA5A5 ) break;
  }
  spi_deassert();
}


SPI_MSTransfer_MASTER_FUNC int SPI_MSTransfer_MASTER_OPT::digitalRead(uint8_t pin) {
  spi_assert();
  port->transfer16(0xDEAD);
  port->transfer16(slave_ID);
  port->transfer16(1);
  port->transfer16(0x1011);
  port->transfer16(pin);
  port->transfer16(pin); /* can't checksum one word, so resend as checksum */

  uint32_t timeout = millis();
  uint16_t response[4] = { 0 }, checksum = 0;
  while ( millis() - timeout < 100 ) {
    if ( port->transfer16(0xFFFF) == 0xFFEA ) {
      response[0] = checksum = 0xFFEA;
      for (int i = 1; i < 4; i++) {
        response[i] = port->transfer16(0xFFFF);
        if ( i < 3 ) checksum ^= response[i];
      }
    }
  }
  spi_deassert();
  if ( checksum == response[3] ) return response[2];
  return -1;
}


SPI_MSTransfer_MASTER_FUNC void SPI_MSTransfer_MASTER_OPT::detectSlaves() {
  uint8_t slave_count = 0;
  spi_assert();
  port->transfer16(0xBEEF);
  Serial.println("\n  Detected slaves: ");
  for ( uint16_t i = 0, data = 0, start = 0; i < 10; i++ ) {
    data = port->transfer16(0xFFFF);
    if ( start ) {
      if ( data == 0xFFFF ) break;
      Serial.printf("    Slave %d --> ID: 0x%04X\n", ++slave_count, data);
    }
    else if ( data == 0xBEEF ) start = 1;
  }
  spi_deassert();
  if ( slave_count ) {
    Serial.printf("    Mode: %s\n\n", (slave_count > 1) ? "Daisy-Chained" : "Standalone");
    return;
  }
  Serial.printf("    No slaves detected, check connections.\n\n");
}


SPI_MSTransfer_MASTER_FUNC uint16_t SPI_MSTransfer_MASTER_OPT::transfer16(uint16_t *buffer, uint16_t length, uint16_t packetID) {
  uint16_t checksum = 0;
  bool crc_passed = 0, spi_fail = 0;
  spi_assert();
  port->transfer16(0xDEAD);
  port->transfer16(slave_ID);
  port->transfer16(length + 1);
  port->transfer16(0xFAF);
  port->transfer16(packetID);
  checksum ^= packetID;
  for ( uint16_t i = 0; i < length; i++ ) {
    port->transfer16(buffer[i]);
    checksum ^= buffer[i];
  }
  if ( port->transfer16(checksum) != 0xCC00 ) spi_fail = 1;
  for ( uint16_t i = 0, result = 0; i < 10; i++ ) {
    result = port->transfer16(0xFFFF);
    if ( result == 0xA5A5 ) { /* CRC PASSED */
      crc_passed = 1;
      break;
    }
    if ( result == 0xE0E0 ) { /* CRC FAILED */
      break;
    }
  }
  spi_deassert();
  if ( spi_fail ) return 0;
  if ( crc_passed ) return 1;
  return 0;
}
