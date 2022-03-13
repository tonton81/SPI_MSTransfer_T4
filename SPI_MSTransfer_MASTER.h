#if !defined(_SPI_MSTransfer_MASTER_H_)
#define _SPI_MSTransfer_MASTER_H_

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

typedef void (*_master_handler_ptr)(uint16_t* buffer, uint16_t length, AsyncMST info);
typedef std::function<void(AsyncMST info)> _detectPtr;

#define SPI_MSTransfer_MASTER_CLASS template<SPIClass* port, uint8_t cs_pin, uint32_t slave_ID, uint32_t spi_speed = 2000000>
#define SPI_MSTransfer_MASTER_FUNC template<SPIClass* port, uint8_t cs_pin, uint32_t slave_ID, uint32_t spi_speed>
#define SPI_MSTransfer_MASTER_OPT SPI_MSTransfer_MASTER<port, cs_pin, slave_ID, spi_speed>

class SPI_MSTransfer_MASTER_Base {
  public:
};

Circular_Buffer<uint16_t, (uint32_t)pow(2, ceil(log(SPI_MST_QUEUE_SLOTS) / log(2))), SPI_MST_DATA_BUFFER_MAX> mstqueue;

SPI_MSTransfer_MASTER_CLASS class SPI_MSTransfer_MASTER : public SPI_MSTransfer_MASTER_Base {
  public:
    SPI_MSTransfer_MASTER();
    void begin();
    void onTransfer(_master_handler_ptr handler) { _master_handler = handler; }
    uint16_t transfer16(uint16_t *buffer, uint16_t length, uint16_t packetID);
    void detectSlaves();
    uint32_t events();
    void pinMode(uint8_t pin, uint8_t state);
    void digitalWrite(uint8_t pin, bool state);
    void digitalWriteFast(uint8_t pin, bool state) { return digitalWrite(pin, state); }
    int digitalRead(uint8_t pin);
    int digitalReadFast(uint8_t pin) { return digitalRead(pin); }
    void delayTransfers(uint16_t uS) { delayed_transfers = uS; }
    uint16_t analogRead(uint8_t pin);
    void analogReadResolution(uint8_t bits);
    void analogWriteResolution(uint8_t bits);
    void analogWrite(uint8_t pin, uint16_t val);

  private:
    void process_data(uint16_t *buffer, uint16_t length, uint16_t command);
    uint16_t spi_transfer16(uint16_t data);
    void spi_assert();
    void spi_deassert();
    int _portnum = 0;
    uint32_t nvic_irq = 0;
    _master_handler_ptr _master_handler;
    volatile uint16_t delayed_transfers = 0;
};

#include "SPI_MSTransfer_MASTER.tpp"
#endif
