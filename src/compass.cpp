#include "compass.h"

//Constructor
Compass::Compass(uint8_t address) : deviceAddress(address){}


void Compass::initializeCompass(){
    Wire.begin();
    Wire.beginTransmission(deviceAddress);
    Wire.write(0x02); // select mode register
    Wire.write(0x00); // continuous measurement mode
    Wire.endTransmission();
}

void Compass::readCompass(int &x, int &y, int &z){
    Wire.beginTransmission(deviceAddress);
    Wire.write(0x03);
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 6);
    if(6 <= Wire.available()){
      x = Wire.read() << 8; //X msb
      x |= Wire.read(); //X lsb
      y = Wire.read() << 8; //Y msb
      y |= Wire.read(); //Y lsb
      z = Wire.read() << 8; //Z msb
      z |= Wire.read(); //Z lsb
  }
}
