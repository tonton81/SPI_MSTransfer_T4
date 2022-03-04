#include <SPI_MSTransfer_T4.h>
#include "Arduino.h"
#include "SPI.h"

#define SLAVE_CR spiAddr[4]
#define SLAVE_FCR spiAddr[22]
#define SLAVE_FSR spiAddr[23]
#define SLAVE_IER spiAddr[6]
#define SLAVE_CFGR0 spiAddr[8]
#define SLAVE_CFGR1 spiAddr[9]
#define SLAVE_TDR spiAddr[25]
#define SLAVE_RDR spiAddr[29]
#define SLAVE_SR spiAddr[5]
#define SLAVE_TCR_REFRESH spiAddr[24] = (2UL << 27) | LPSPI_TCR_FRAMESZ(16 - 1)
#define SLAVE_PORT_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x40394000 + (0x4000 * _portnum)))
#define SLAVE_PINS_ADDR volatile uint32_t *spiAddr = &(*(volatile uint32_t*)(0x401F84EC + (_portnum * 0x10)))


#define SPI_WAIT_STATE \
    while ( !(SLAVE_SR & (1UL << 9)) ) { /* FCF: Frame Complete Flag, set when PCS deasserts */ \
      if ( !(SLAVE_FSR & 0x1F0000) ) continue; /* wait for received data */ \
      if ( (SLAVE_SR & (1UL << 8)) ) { /* WCF set */
#define SPI_ENDWAIT_STATE \
      } \
    }

 
void lpspi4_slave_isr() {
  _LPSPI4->SPI_MSTransfer_SLAVE_ISR();
}


SPI_MSTransfer_T4_FUNC SPI_MSTransfer_T4_OPT::SPI_MSTransfer_T4() {
  if ( port == &SPI ) {
    _LPSPI4 = this;
    _portnum = 3;
    CCM_CCGR1 |= (3UL << 6);
    nvic_irq = 32 + _portnum;
    _VectorsRam[16 + nvic_irq] = lpspi4_slave_isr;

    /* Alternate pins not broken out on Teensy 4.0/4.1 for LPSPI4 */
    SLAVE_PINS_ADDR;
    spiAddr[0] = 0; /* PCS0_SELECT_INPUT */
    spiAddr[1] = 0; /* SCK_SELECT_INPUT */
    spiAddr[2] = 0; /* SDI_SELECT_INPUT */
    spiAddr[3] = 0; /* SDO_SELECT_INPUT */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* LPSPI4 SCK (CLK) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; /* LPSPI4 SDI (MISO) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; /* LPSPI4 SDO (MOSI) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* LPSPI4 PCS0 (CS) */
  } 
}


SPI_MSTransfer_T4_FUNC void SPI_MSTransfer_T4_OPT::swapPins(bool enable) {
  SLAVE_PORT_ADDR;
  SLAVE_CR &= ~LPSPI_CR_MEN; /* Disable Module */
  SLAVE_CFGR1 = (SLAVE_CFGR1 & 0xFCFFFFFF) | (enable) ? (3UL << 24) : (0UL << 24);
  SLAVE_CR |= LPSPI_CR_MEN; /* Enable Module */
}


SPI_MSTransfer_T4_FUNC void SPI_MSTransfer_T4_OPT::begin() {
  SLAVE_PORT_ADDR;
  SLAVE_CR = LPSPI_CR_RST; /* Reset Module */
  SLAVE_CR = 0; /* Disable Module */
  SLAVE_FCR = 0;//x10001; /* 1x watermark for RX and TX */
  SLAVE_IER = 0x1; /* RX Interrupt */
  SLAVE_CFGR0 = 0 | (1UL << 26);
  SLAVE_CFGR1 = 0;
  SLAVE_CR |= LPSPI_CR_MEN | LPSPI_CR_DBGEN; /* Enable Module, Debug Mode */
  SLAVE_SR = 0x3F00; /* Clear status register */
  SLAVE_TCR_REFRESH;
  SLAVE_TDR = 0x0; /* dummy data, must populate initial TX slot */
  NVIC_ENABLE_IRQ(nvic_irq);
}


SPI_MSTransfer_T4_FUNC void SPI_MSTransfer_T4_OPT::SPI_MSTransfer_SLAVE_ISR() {
  static uint16_t data[SPI_MST_DATA_BUFFER_MAX];
  memset(data, 0, sizeof(uint16_t)*SPI_MST_DATA_BUFFER_MAX);
  uint16_t buffer_pos = 0, len = 0, checksum = 0;
  bool detectOnce = 1;

  SLAVE_PORT_ADDR;

  SPI_WAIT_STATE
    data[buffer_pos] = SLAVE_RDR;
    if ( (data[0] != 0xDEAD) && (data[0] != 0xBEEF) ) {
      SLAVE_TDR = data[buffer_pos];
      buffer_pos = 0;
      continue;
    }
    else if ( data[0] == 0xBEEF ) { /* slave detection */
      if ( detectOnce && data[buffer_pos] == 0xFFFF ) {
        detectOnce = 0;
        SLAVE_TDR = slave_ID;
      }
      else SLAVE_TDR = data[buffer_pos];
    }
    else if ( data[1] != slave_ID ) {
      SLAVE_TDR = data[buffer_pos];
      buffer_pos = 0;
    }
    else SLAVE_TDR = 0xCC00;
    if ( data[2] ) len = data[2];

    if ( buffer_pos == (len + 4) ) {  /* received payload, check CRC */ 
      for ( int i = 4; i < len + 4; i++ ) checksum ^= data[i];
      if ( checksum == data[len + 4] ) { /* CRC GOOD */
        break;
      }
      else { /* CRC Failed */
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR = 0xE0E0;
        SPI_ENDWAIT_STATE
      }
    }
    buffer_pos++;
  SPI_ENDWAIT_STATE

  if ( checksum == data[len + 4] ) {
    if ( data[0] == 0xDEAD ) {
      /* ##################################################################### */
      /* ########################### FIRE & FORGET ########################### */
      /* ##################################################################### */
      if ( data[3] == 0xFAF ) {
        mstqueue.push_back(data, len + 5);
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR = 0xA5A5;
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### DIGITALWRITE ############################ */
      /* ##################################################################### */
      if ( data[3] == 0x1010 ) {
        ::digitalWriteFast(data[4] >> 8, data[4] & 0x1);
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR = 0xA5A5;
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### DIGITALREAD ############################# */
      /* ##################################################################### */
      if ( data[3] == 0x1011 ) {
        bool state = ::digitalReadFast(data[4]);
        uint16_t buffer[4] = { 0xFFEA, 0xCCDA, state, 0 }, send_pos = 0;
        for ( uint16_t i = 0; i < 3; i++ ) buffer[3] ^= buffer[i];
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR = buffer[send_pos];
          if ( ++send_pos > 3 ) send_pos = 0;
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### PINMODE ################################# */
      /* ##################################################################### */
      if ( data[3] == 0x1012 ) {
        ::pinMode(data[4] >> 8, (uint8_t)data[4]);
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR = 0xA5A5;
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ##################################################################### */
      /* ##################################################################### */

    } /* end of 0xDEAD CMD */




  }
  SLAVE_SR = 0x3F00; /* Clear remaining flags on exit */
  asm volatile ("dsb");
}

SPI_MSTransfer_T4_FUNC uint32_t SPI_MSTransfer_T4_OPT::events() {
  if ( mstqueue.size() ) {
    uint16_t data[mstqueue.length_front()];
    mstqueue.pop_front(data, (sizeof(data) >> 1));
    AsyncMST info; info.packetID = data[4];
    if ( _slave_handler != nullptr ) _slave_handler(data + 5, (sizeof(data) >> 1) - 6, info);
    return 1;
  }
  return 0;
}