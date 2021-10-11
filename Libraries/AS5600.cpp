

#include "Arduino.h"
#include "AS5600.h"
#include "Wire.h"


AMS_5600::AMS_5600()
{
  /* set i2c address */ 
  _ams5600_Address = 0x36;
 
  /* load register values*/
  /* c++ class forbids pre loading of variables */
  _zmco = 0x00;
  _zpos_hi = 0x01;
  _zpos_lo = 0x02;
  _mpos_hi = 0x03;
  _mpos_lo = 0x04;
  _mang_hi = 0x05;
  _mang_lo = 0x06;
  _conf_hi = 0x07;    
  _conf_lo = 0x08;
  _raw_ang_hi = 0x0c;
  _raw_ang_lo = 0x0d;
  _ang_hi = 0x0e;
  _ang_lo = 0x0f;
  _stat = 0x0b;
  _agc = 0x1a;
  _mag_hi = 0x1b;
  _mag_lo = 0x1c;
  _burn = 0xff;
}
/* mode = 0, output PWM, mode = 1 output analog (full range from 0% to 100% between GND and VDD*/
void AMS_5600::setOutPut(uint8_t mode){
    uint8_t config_status;
    config_status = readOneByte(_conf_lo);
    if(mode == 1){
        config_status = config_status & 0xcf;
    }else{
        config_status = config_status & 0xef;
    }
    writeOneByte(_conf_lo, lowByte(config_status)); 
}

int AMS_5600::getAddress()
{
  return _ams5600_Address; 
}


word AMS_5600::setMaxAngle(word newMaxAngle)
{
  word retVal;
  if(newMaxAngle == -1)
  {
    _maxAngle = getRawAngle();
  }
  else
    _maxAngle = newMaxAngle;

  writeOneByte(_mang_hi, highByte(_maxAngle));
  delay(2); 
  writeOneByte(_mang_lo, lowByte(_maxAngle)); 
  delay(2);         

  retVal = readTwoBytes(_mang_hi, _mang_lo);
  return retVal;
}

word AMS_5600::getMaxAngle()
{
  return readTwoBytes(_mang_hi, _mang_lo);
}


word AMS_5600::setStartPosition(word startAngle)
{
  if(startAngle == -1)
  {
    _rawStartAngle = getRawAngle();
  }
  else
    _rawStartAngle = startAngle;

  writeOneByte(_zpos_hi, highByte(_rawStartAngle));
  delay(2); 
  writeOneByte(_zpos_lo, lowByte(_rawStartAngle)); 
  delay(2);                
  _zPosition = readTwoBytes(_zpos_hi, _zpos_lo);
  
  return(_zPosition);
}

word AMS_5600::getStartPosition()
{
  return readTwoBytes(_zpos_hi, _zpos_lo);
}  


word AMS_5600::setEndPosition(word endAngle)
{
  if(endAngle == -1)
    _rawEndAngle = getRawAngle();
  else
    _rawEndAngle = endAngle;
 
  writeOneByte(_mpos_hi, highByte(_rawEndAngle));
  delay(2); 
  writeOneByte(_mpos_lo, lowByte(_rawEndAngle)); 
  delay(2);                
  _mPosition = readTwoBytes(_mpos_hi, _mpos_lo);
  
  return(_mPosition);
}

word AMS_5600::getEndPosition()
{
  word retVal = readTwoBytes(_mpos_hi, _mpos_lo);
  return retVal;
}  


word AMS_5600::getRawAngle()
{
  return readTwoBytes(_raw_ang_hi, _raw_ang_lo);
}


word AMS_5600::getScaledAngle()
{
  return readTwoBytes(_ang_hi, _ang_lo);
}


int AMS_5600::detectMagnet()
{
  int magStatus;
  int retVal = 0;
  /*0 0 MD ML MH 0 0 0*/
  /* MD high = AGC minimum overflow, Magnet to strong */
  /* ML high = AGC Maximum overflow, magnet to weak*/ 
  /* MH high = magnet detected*/ 
  magStatus = readOneByte(_stat);
  
  if(magStatus & 0x20)
    retVal = 1; 
  
  return retVal;
}


int AMS_5600::getMagnetStrength()
{
  int magStatus;
  int retVal = 0;
  /*0 0 MD ML MH 0 0 0*/
  /* MD high = AGC minimum overflow, Magnet to strong */
  /* ML high = AGC Maximum overflow, magnet to weak*/ 
  /* MH high = magnet detected*/ 
  magStatus = readOneByte(_stat);
  if(detectMagnet() ==1)
  {
      retVal = 2; /*just right */
      if(magStatus & 0x10)
        retVal = 1; /*to weak */
      else if(magStatus & 0x08)
        retVal = 3; /*to strong */
  }
  
  return retVal;
}


int AMS_5600::getAgc()
{
  return readOneByte(_agc);
}


word AMS_5600::getMagnitude()
{
  return readTwoBytes(_mag_hi, _mag_lo);  
}


int AMS_5600::getBurnCount()
{
  return readOneByte(_zmco);
}


int AMS_5600::burnAngle()
{
  int retVal = 1;
  _zPosition = getStartPosition();
  _mPosition = getEndPosition();
  _maxAngle  = getMaxAngle();
  
  if(detectMagnet() == 1)
  {
    if(getBurnCount() < 3)
    {
      if((_zPosition == 0)&&(_mPosition ==0))
        retVal = -3;
      else
        writeOneByte(_burn, 0x80);
    }
    else
      retVal = -2;
  } 
  else
    retVal = -1;
    
  return retVal;
}


int AMS_5600::burnMaxAngleAndConfig()
{
  int retVal = 1;
  _maxAngle  = getMaxAngle();
  
  if(getBurnCount() ==0)
  {
    if(_maxAngle*0.087 < 18)
      retVal = -2;
    else
      writeOneByte(_burn, 0x40);    
  }  
  else
    retVal = -1;
    
  return retVal;
}



int AMS_5600::readOneByte(int in_adr)
{
  int retVal = -1;
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(in_adr);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, 1);
 while(Wire.available() == 0);
  retVal = Wire.read();
  
  return retVal;
}

word AMS_5600::readTwoBytes(int in_adr_hi, int in_adr_lo)
{
  word retVal = -1;
 
  /* Read Low Byte */
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(in_adr_lo);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, 1);
  while(Wire.available() == 0);
  int low = Wire.read();
 
  /* Read High Byte */  
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(in_adr_hi);
  Wire.endTransmission();
  Wire.requestFrom(_ams5600_Address, 1);
  
  while(Wire.available() == 0);
  
  word high = Wire.read();
  
  high = high << 8;
  retVal = high | low;
  
  return retVal;
}

void AMS_5600::writeOneByte(int adr_in, int dat_in)
{
  Wire.beginTransmission(_ams5600_Address);
  Wire.write(adr_in); 
  Wire.write(dat_in);
  Wire.endTransmission();
}

/**********  END OF AMS 5600 CALSS *****************/
