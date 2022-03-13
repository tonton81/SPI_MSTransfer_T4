#include <SPI_MSTransfer_T4.h>
#include "Arduino.h"
#include "SPI.h"


extern void __attribute__((weak)) spi0_slave_isr(void) {
  _SPI0->SPI_MSTransfer_SLAVE_ISR();
}

extern void __attribute__((weak)) spi1_slave_isr(void) {
  _SPI1->SPI_MSTransfer_SLAVE_ISR();
}

extern void __attribute__((weak)) spi2_slave_isr(void) {
  _SPI2->SPI_MSTransfer_SLAVE_ISR();
}

extern void __attribute__((weak)) lpspi4_slave_isr() {
  _LPSPI4->SPI_MSTransfer_SLAVE_ISR();
}


SPI_MSTransfer_T4_FUNC void SPI_MSTransfer_T4_OPT::test() {
//Serial.println((uint32_t)&X,HEX);
//Serial.println((uint32_t)&spiAddr[0],HEX);
}


SPI_MSTransfer_T4_FUNC SPI_MSTransfer_T4_OPT::SPI_MSTransfer_T4() {

#if defined(KINETISL)
  if ( port == &SPI ) {
    _SPI0 = this;
    SIM_SCGC4 |= SIM_SCGC4_SPI0;
    SIM_SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTC;
    nvic_irq = IRQ_SPI0;
    spiAddr = &(*(volatile uint8_t*)(0x40076000));
    _VectorsRam[16 + nvic_irq] = spi0_slave_isr;
    CORE_PIN12_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    CORE_PIN10_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    CORE_PIN13_CONFIG = PORT_PCR_MUX(2);
  }
  if ( port == &SPI1 ) {
    _SPI1 = this;
    SIM_SCGC4 |= SIM_SCGC4_SPI1;
    SIM_SCGC5 |= SIM_SCGC5_PORTE;
    nvic_irq = IRQ_SPI1;
    spiAddr = &(*(volatile uint8_t*)(0x40077000));
    _VectorsRam[16 + nvic_irq] = spi1_slave_isr;
    CORE_PIN0_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    CORE_PIN5_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN6_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    CORE_PIN20_CONFIG = PORT_PCR_MUX(2);
  }

#elif defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( port == &SPI ) {
    _SPI0 = this;
    SIM_SCGC6 |= SIM_SCGC6_SPI0;
    nvic_irq = IRQ_SPI0;
    spiAddr = &(*(volatile uint32_t*)(0x4002C000));
    _VectorsRam[16 + nvic_irq] = spi0_slave_isr;
    CORE_PIN12_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    CORE_PIN10_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    CORE_PIN13_CONFIG = PORT_PCR_MUX(2);
  }
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  if ( port == &SPI1 ) {
    _SPI1 = this;
    SIM_SCGC6 |= SIM_SCGC6_SPI1;
    nvic_irq = IRQ_SPI1;
    spiAddr = &(*(volatile uint32_t*)(0x4002D000));
    _VectorsRam[16 + nvic_irq] = spi1_slave_isr;
    CORE_PIN1_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN0_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    CORE_PIN31_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    CORE_PIN32_CONFIG = PORT_PCR_MUX(2);
  }
  if ( port == &SPI2 ) {
    _SPI2 = this;
    SIM_SCGC3 |= SIM_SCGC3_SPI2;
    nvic_irq = IRQ_SPI2;
    spiAddr = &(*(volatile uint32_t*)(0x400AC000));
    _VectorsRam[16 + nvic_irq] = spi2_slave_isr;
    CORE_PIN53_CONFIG = PORT_PCR_MUX(2);
    CORE_PIN43_CONFIG = PORT_PCR_PE | PORT_PCR_PS | PORT_PCR_MUX(2);
    CORE_PIN52_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2);
    CORE_PIN51_CONFIG = PORT_PCR_MUX(2);
  }
#endif


#elif defined(__IMXRT1062__)
  if ( port == &SPI ) {
    _LPSPI4 = this;
    uint8_t _portnum = 3;
    spiAddr = &(*(volatile uint32_t*)(0x40394000 + (0x4000 * _portnum)));
    CCM_CCGR1 |= (3UL << 6);
    nvic_irq = 32 + _portnum;
    _VectorsRam[16 + nvic_irq] = lpspi4_slave_isr;

    /* Alternate pins not broken out on Teensy 4.0/4.1 for LPSPI4 */
    volatile uint32_t *spireg = &(*(volatile uint32_t*)(0x401F84EC + (_portnum * 0x10)));
    spireg[0] = 0; /* PCS0_SELECT_INPUT */
    spireg[1] = 0; /* SCK_SELECT_INPUT */
    spireg[2] = 0; /* SDI_SELECT_INPUT */
    spireg[3] = 0; /* SDO_SELECT_INPUT */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 0x3; /* LPSPI4 SCK (CLK) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 0x3; /* LPSPI4 SDI (MISO) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 0x3; /* LPSPI4 SDO (MOSI) */
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 0x3; /* LPSPI4 PCS0 (CS) */
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_01 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; /* LPSPI4 SDI (MISO) */
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_02 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; /* LPSPI4 SDO (MOSI) */
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_03 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; /* LPSPI4 SCK (CLK) */
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_PKE; /* LPSPI4 PCS0 (CS) */
  } 
#endif
}


SPI_MSTransfer_T4_FUNC void SPI_MSTransfer_T4_OPT::begin() {

#if defined(KINETISL)
  SLAVE_C1 = 0;
  SLAVE_C2 = 0x40;
  SLAVE_BR = 0;
  SLAVE_C1 = 0b11001100;

#elif defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  SLAVE_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
  SLAVE_MCR = 0x00000000;
  SLAVE_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
  SLAVE_CTAR0 = 0;
  SLAVE_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
  SLAVE_CTAR0 = SPI_CTAR_FMSZ(16 - 1);
  SLAVE_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
  SLAVE_MCR |= SPI_MCR_HALT | SPI_MCR_MDIS;
  SLAVE_CTAR0 = SLAVE_CTAR0 & ~(SPI_CTAR_CPOL | SPI_CTAR_CPHA);
  SLAVE_MCR &= ~SPI_MCR_HALT & ~SPI_MCR_MDIS;
  SLAVE_RSER = 0x00020000;

#elif defined(__IMXRT1062__)
  SLAVE_CR = LPSPI_CR_RST; /* Reset Module */
  SLAVE_CR = 0; /* Disable Module */
  SLAVE_FCR = 0;
  SLAVE_IER = 0x1; /* RX Interrupt */
  SLAVE_CFGR0 = 0;
  SLAVE_CFGR1 = LPSPI_CFGR1_OUTCFG;
  SLAVE_SR = 0x3F00; /* Clear status register */
  SLAVE_TCR_REFRESH;
  SLAVE_TDR(0x0); /* dummy data, must populate initial TX slot */
  SLAVE_CR |= LPSPI_CR_MEN | LPSPI_CR_DBGEN | LPSPI_CR_DOZEN; /* Enable Module, Debug Mode, Doze Mode */
#endif

  NVIC_ENABLE_IRQ(nvic_irq);
}


SPI_MSTransfer_T4_FUNC void SPI_MSTransfer_T4_OPT::SPI_MSTransfer_SLAVE_ISR() {
  static uint16_t data[SPI_MST_DATA_BUFFER_MAX];
  memset(data, 0, sizeof(uint16_t)*SPI_MST_DATA_BUFFER_MAX);
  uint16_t buffer_pos = 0, len = 0, checksum = 0;
  bool detectOnce = 1;

  SPI_WAIT_STATE
    if ( buffer_pos >= SPI_MST_DATA_BUFFER_MAX ) buffer_pos = 0;
    data[buffer_pos] = SLAVE_RDR;
    if ( (data[0] != 0xDEAD) && (data[0] != 0xBEEF) ) {
      SLAVE_TDR(data[buffer_pos]);
      buffer_pos = 0;
      continue;
    }
    else if ( data[0] == 0xBEEF ) { /* slave detection */
      if ( detectOnce && data[buffer_pos] == 0xFFFF ) {
        detectOnce = 0;
        SLAVE_TDR(slave_ID);
      }
      else {
        SLAVE_TDR(data[buffer_pos]);
      }
      buffer_pos++;
      continue;
    }
    else if ( data[1] != slave_ID ) {
      SLAVE_TDR(data[buffer_pos]);
      buffer_pos = 0;
    }
    else SLAVE_TDR(0xCC00);

    if ( data[2] ) len = data[2];

    if ( buffer_pos == (len + 4) ) {  /* received payload, check CRC */ 
      for ( int i = 4; i < len + 4; i++ ) checksum ^= data[i];
      if ( checksum == data[len + 4] ) { /* CRC GOOD */
        break;
      }
      else { /* CRC Failed */
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR(0xE0E0);
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
          SLAVE_TDR(0xA5A5);
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### ACCESS SLAVE'S QUEUE #################### */
      /* ##################################################################### */
      if ( data[3] == 0xF1A0 ) {
        if ( !smtqueue.size() ) {
          SPI_WAIT_STATE
            (void)SLAVE_RDR;
            SLAVE_TDR(0xAD00);
          SPI_ENDWAIT_STATE
        }
        else {
          uint16_t buf[smtqueue.length_front()] = { 0 }, pos = 0, command = 0;
          smtqueue.peek_front(buf, sizeof(buf) >> 1);
          SPI_WAIT_STATE
            command = SLAVE_RDR;
            SLAVE_TDR(0xAD00 | smtqueue.size());
            if ( command == 0xCEB6 ) {
              SPI_WAIT_STATE
                command = SLAVE_RDR;
                if ( pos >= (sizeof(buf) >> 1) ) pos = 0;
                SLAVE_TDR(buf[pos]);
                pos++;
                if ( command == 0xCE0A ) {
                  smtqueue.pop_front();
                  SPI_WAIT_STATE
                    command = SLAVE_RDR;
                    SLAVE_TDR(0xD632);
                  SPI_ENDWAIT_STATE
                }
              SPI_ENDWAIT_STATE
            }
          SPI_ENDWAIT_STATE
        }
      }
      /* ##################################################################### */
      /* ########################### DIGITALWRITE ############################ */
      /* ##################################################################### */
      if ( data[3] == 0x1010 ) {
        ::digitalWriteFast(data[4] >> 8, data[4] & 0x1);
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR(0xA5A5);
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### DIGITALREAD ############################# */
      /* ##################################################################### */
      if ( data[3] == 0x1011 ) {
        bool state = ::digitalReadFast(data[4]);
        uint16_t buffer[4] = { 0xFFEA, (sizeof(buffer) >> 1), state, 0 }, send_pos = 0;
        for ( uint16_t i = 0; i < 3; i++ ) buffer[3] ^= buffer[i];
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR(buffer[send_pos]);
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
          SLAVE_TDR(0xA5A5);
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### ANALOGREAD ############################## */
      /* ##################################################################### */
      if ( data[3] == 0x1013 ) {
        uint16_t val = ::analogRead(data[4]);
        uint16_t buffer[4] = { 0xFFEA, (sizeof(buffer) >> 1), val, 0 }, send_pos = 0;
        for ( uint16_t i = 0; i < 3; i++ ) buffer[3] ^= buffer[i];
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR(buffer[send_pos]);
          if ( ++send_pos > 3 ) send_pos = 0;
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### ANALOGREADRESOLUTION #################### */
      /* ##################################################################### */
      if ( data[3] == 0x1014 ) {
        ::analogReadResolution(data[4]);
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR(0xA5A5);
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### ANALOGWRITE ############################# */
      /* ##################################################################### */
      if ( data[3] == 0x1015 ) {
        analogWrite(data[4], data[5]);
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR(0xA5A5);
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ########################### ANALOGWRITERESOLUTION ################### */
      /* ##################################################################### */
      if ( data[3] == 0x1016 ) {
        ::analogWriteResolution(data[4]);
        SPI_WAIT_STATE
          (void)SLAVE_RDR;
          SLAVE_TDR(0xA5A5);
        SPI_ENDWAIT_STATE
      }
      /* ##################################################################### */
      /* ##################################################################### */
      /* ##################################################################### */

    } /* end of 0xDEAD CMD */

  }
  SPI_ISR_EXIT
}


SPI_MSTransfer_T4_FUNC uint16_t SPI_MSTransfer_T4_OPT::transfer16(uint16_t *buffer, uint16_t length, uint16_t packetID) {
  if ( smtqueue.size() == smtqueue.capacity() ) return 0;
  uint16_t data[7 + length], checksum = 0, data_pos = 0;
  data[data_pos] = 0xAA55; checksum ^= data[data_pos]; data_pos++; // HEADER
  data[data_pos] = sizeof(data) >> 1; checksum ^= data[data_pos]; data_pos++; // DATA SIZE
  data[data_pos] = 0x0000; checksum ^= data[data_pos]; data_pos++; // SUB SWITCH STATEMENT
  data[data_pos] = length; checksum ^= data[data_pos]; data_pos++;
  data[data_pos] = slave_ID; checksum ^= data[data_pos]; data_pos++;
  data[data_pos] = packetID; checksum ^= data[data_pos]; data_pos++;
  for ( uint16_t i = 0; i < length; i++ ) {
    data[data_pos] = buffer[i];
    checksum ^= data[data_pos];
    data_pos++;
  }
  data[data_pos] = checksum;
  smtqueue.push_back(data, data[1]);
  return packetID;
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
