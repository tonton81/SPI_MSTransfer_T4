#if !defined(_SPI_MSTransfer_T4_H_)
#define _SPI_MSTransfer_T4_H_

#include "Arduino.h"
#include "circular_buffer.h"
#include <functional>
#include <SPI.h>

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

#define SPI_WAIT_STATE \
    while ( !(SLAVE_SR & (1UL << 9)) ) { /* FCF: Frame Complete Flag, set when PCS deasserts */ \
      if ( !(SLAVE_FSR & 0x1F0000) ) continue; /* wait for received data */ \
      if ( (SLAVE_SR & (1UL << 8)) ) { /* WCF set */
#define SPI_ENDWAIT_STATE \
      } \
    }


#define SPI_MST_QUEUE_SLOTS 8
#define SPI_MST_DATA_BUFFER_MAX 250

struct AsyncMST {
  uint16_t packetID = 0;
  uint16_t slaveID = 0;
};

typedef void (*_slave_handler_ptr)(uint16_t* buffer, uint16_t length, AsyncMST info);
typedef std::function<void(AsyncMST info)> _detectPtr;

typedef void (*_SPI_ptr)();

#define SPI_MSTransfer_T4_CLASS template<SPIClass* port = nullptr, uint32_t slave_ID = -1>
#define SPI_MSTransfer_T4_FUNC template<SPIClass* port, uint32_t slave_ID>
#define SPI_MSTransfer_T4_OPT SPI_MSTransfer_T4<port, slave_ID>

extern SPIClass SPI;

class SPI_MSTransfer_T4_Base {
  public:
    virtual void SPI_MSTransfer_SLAVE_ISR();
};

static SPI_MSTransfer_T4_Base* _LPSPI4 = nullptr;

Circular_Buffer<uint16_t, (uint32_t)pow(2, ceil(log(SPI_MST_QUEUE_SLOTS) / log(2))), SPI_MST_DATA_BUFFER_MAX> mstqueue;

SPI_MSTransfer_T4_CLASS class SPI_MSTransfer_T4 : public SPI_MSTransfer_T4_Base {
  public:
    SPI_MSTransfer_T4();
    void begin();
    void swapPins(bool enable = 1);
    void onTransfer(_slave_handler_ptr handler) { _slave_handler = handler; }
    uint32_t events();

  private:
    SLAVE_PORT_ADDR;
    void SPI_MSTransfer_SLAVE_ISR();
    int _portnum = 0;
    uint32_t nvic_irq = 0;
    _slave_handler_ptr _slave_handler;
};

#include "SPI_MSTransfer_T4.tpp"
#endif
