#include "SPI.h"

#include "SPI_MSTransfer_MASTER.h"
SPI_MSTransfer_MASTER<&SPI, 10, 0x1234> mySPI1234;
SPI_MSTransfer_MASTER<&SPI, 10, 0x4567> mySPI4567;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(10, OUTPUT);
  digitalWrite(10, 1);
  mySPI1234.begin();
  mySPI4567.begin();
}

void loop() {

  static uint32_t t = millis();
  if ( millis() - t > 1000 ) {
    Serial.println(millis());

    uint16_t buf[5] = { 0xF1, 0xF2, 0xDEAD, 0xF4, 0xBEEF };
    uint16_t buf2[5] = { 0xBEEF, 0xF7, 0xF8, 0xF9, 0xDEAD };
    mySPI1234.transfer16(buf2, 5, random(0x1000, 0x8000));
    mySPI4567.transfer16(buf, 5, random(0x1000, 0x8000));

    static bool flip = 0;
    flip = !flip;
    mySPI1234.digitalWrite(6, flip);
    //    mySPI1234.pinMode(5, INPUT);
    bool moo = mySPI1234.digitalRead(6);
    Serial.print("State: "); Serial.println(moo);
    mySPI1234.detectSlaves();

    mySPI1234.pinMode(5, INPUT);
    t = millis();
  }

}
