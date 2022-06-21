#include "SPI_MSTransfer_T4/SPI_MSTransfer_T4.h"
SPI_MSTransfer_T4<&SPI, 0x4567> mySPI;

void myCB(uint16_t *buffer, uint16_t length, AsyncMST info) {
  for ( int i = 0; i < length; i++ ) {
    Serial.print(buffer[i]); Serial.print(" ");
  }
  Serial.print(" --> Length: "); Serial.print(length);
  Serial.print(" --> PacketID: "); Serial.println(info.packetID, HEX);
}
void setup() {
  Serial.begin(9600);
  mySPI.begin();
  mySPI.onTransfer(myCB);
}

void loop() {
  mySPI.events();
  static uint32_t t = millis();
  if ( millis() - t > 1000 ) {
    Serial.print("millis: "); Serial.println(millis());
    t = millis();
  }
  uint16_t buf2[5] = { 0xBEEF, 0xF7, 0xF8, 0xF9, 0xDEAD };
  mySPI.transfer16(buf2, 5, 0x4567);
}

