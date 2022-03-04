#if !defined(_SPI_MSTransfer_T4_H_)
#define _SPI_MSTransfer_T4_H_

#include "Arduino.h"
#include "circular_buffer.h"
#include <functional>
#include <SPI.h>

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

//static SPI_MSTransfer_T4_Base* _LPSPI1 = nullptr;
//static SPI_MSTransfer_T4_Base* _LPSPI2 = nullptr;
//static SPI_MSTransfer_T4_Base* _LPSPI3 = nullptr;
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
    void SPI_MSTransfer_SLAVE_ISR();
    int _portnum = 0;
    uint32_t nvic_irq = 0;
    _slave_handler_ptr _slave_handler;
};

#include "SPI_MSTransfer_T4.tpp"
#endif