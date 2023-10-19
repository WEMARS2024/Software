
#include "0_Core_Zero.h"


#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

#define I2C_SDA 5
#define I2C_SCL 18
TwoWire I2Cax = TwoWire(0);



//GPS/Acellreometers
static const int RXPin = 23, TXPin = 19;
static const uint32_t GPSBaud = 9600;


// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
#define BAUD_RATE 9600
EspSoftwareSerial::UART ss;

Adafruit_MPU6050 mpu;


//If bits are one then updated bit 0 is location,
unsigned int uiTime;
unsigned int uiDate;
unsigned int uiSatNumber;
unsigned int uiValidGPSData;

float fLat= 0.0;
float fLon = 0.0;
float fMPS = 0.0;
float fDegree = 0.0;
float fAlt = 0.0;
float fHDOP = 99.00;

float fXAccel = 0.0;
float fYAccel = 0.0;
float fZAccel = 0.0;
float fXGyro = 0.0;
float fYGyro = 0.0;
float fZGyro = 0.0;
float fTemperature = 0.0;

unsigned int uiInByte;
char strGPS[200];
char strIMU[200];

char strLat[15];
char strLon[15];
char strMPS[6];
char strDegree[6];
char strAlt[10];
char strHDOP[6];

char strXAccel[10];
char strYAccel[10];
char strZAccel[10];
char strXGyro[10];
char strYGyro[10];
char strZGyro[10];
char strTemperature[10];

const int ciGPS_IMUTimer =  200;

unsigned long ulPreviousMillis;
unsigned long ulCurrentMillis;


void setup()
{

  Serial.begin(115200);

  Core_ZEROInit();

  uiValidGPSData = 0;

  ss.begin(BAUD_RATE, EspSoftwareSerial::SWSERIAL_8N1, RXPin, TXPin);
  
   // Try to initialize!
  if (!mpu.begin(0x68, &I2Cax)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
    
}




void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);



   if (Serial.available() > 0)
   {
    // get incoming byte:
    uiInByte = Serial.read();
   }
  // Dispatch incoming characters
  while (ss.available() > 0)

    gps.encode(ss.read());
  
  
  if (gps.time.isValid())
  {
    if(gps.time.value() != 0)
    {
      uiValidGPSData |= 0x0001;
      uiTime = gps.time.value();
    }
    else
    {
      uiValidGPSData &= 0xFFFE;
    }
    
    
  }
  

  if (gps.location.isValid())
  {
    if((gps.location.lat() != 0.0)&&(gps.location.lng()) != 0.0)
     {
      uiValidGPSData |= 0x0002;
      fLat = gps.location.lat();
      fLon = gps.location.lng();
    }
    else
    {
      uiValidGPSData &= 0xFFFD;
    }
    
  }
  
  if (gps.date.isValid())
  {
    if(gps.date.value() != 0)
    {
      uiValidGPSData |= 0x0004;
      uiDate = gps.date.value();
    }
    else
    {
      uiValidGPSData &= 0xFFFB;
    }
    
  }
  
  if (gps.speed.isValid())
  {
    uiValidGPSData |= 0x0040;
    fMPS = gps.speed.mps();
  }
  else
  {
    uiValidGPSData &= 0xFFBF;
  }
  
  
  if (gps.course.isValid())
  {
    uiValidGPSData |= 0x0010;
    fDegree = gps.course.deg();
  }
  else
  {
    uiValidGPSData &= 0xFFEF;
  }
 
  
  if (gps.altitude.isValid())
  {
    uiValidGPSData |= 0x0020;
    fAlt = gps.altitude.meters();
  }
  else
  {
    uiValidGPSData &= 0xFFDF;
  }
  
  
  if (gps.satellites.isValid())
  {
    if(gps.satellites.value() != 0)
    {
      uiValidGPSData |= 0x0040;
      uiSatNumber = gps.satellites.value();
    }
    else
    {
      uiValidGPSData &= 0xFFBF;
    }
   
  }
  
  if (gps.hdop.isValid())
  {
    if(gps.hdop.hdop() != 99.99)
     {
      uiValidGPSData |= 0x0080;
      fHDOP = gps.hdop.hdop();
    }
    else
    {
      uiValidGPSData &= 0xFF7F;
    }
    
  }
  
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("WARNING: No GPS data.  Check wiring."));
    uiValidGPSData = 0x0000;
  }

  if(uiInByte == 'g')
  {
    Serial.println(strGPS);
    Serial.println(strIMU);
  }

  ulCurrentMillis = millis();
  if ((ulCurrentMillis - ulPreviousMillis) >= ciGPS_IMUTimer)
  {
    ulPreviousMillis = ulCurrentMillis;
    strcpy(strIMU,"0");
     fXAccel = a.acceleration.x;
     dtostrf( fXAccel, 4, 4, strXAccel);
     fYAccel = a.acceleration.y;
     dtostrf( fYAccel, 4, 4, strYAccel);
     fZAccel = a.acceleration.z;
     dtostrf( fZAccel, 4, 4, strZAccel);
     fXGyro = g.gyro.x;
     dtostrf( fXGyro, 4, 4, strXGyro);
     fYGyro = g.gyro.y;
     dtostrf( fYGyro, 4, 4, strYGyro);
     fZGyro = g.gyro.z;
     dtostrf( fZGyro, 4, 4, strZGyro);
     fTemperature = temp.temperature;
     dtostrf( fTemperature, 4, 4, strTemperature);
     sprintf(strIMU,"1,%s,%s,%s,%s,%s,%s,%s\n",strXAccel,strYAccel,strZAccel,strXGyro,strYGyro,strZGyro,strTemperature);

    if(uiValidGPSData == 0x007F)
    {
      strcpy(strGPS,"0");
      dtostrf( fLat, 3, 8, strLat);
      dtostrf( fLon, 3, 8, strLon);
      dtostrf( fMPS, 3, 2, strMPS);
      dtostrf( fDegree, 3, 2, strDegree);
      dtostrf( fAlt, 5, 2, strAlt);
      dtostrf( fHDOP, 2, 2, strHDOP);

      sprintf(strGPS,"1,%i,%s,%s,%i,%s,%s,%s,%i,%s\n",uiTime,strLat,strLon,uiDate,strMPS,strDegree,strAlt,uiSatNumber,strHDOP);
    }
    else
    {
      strcpy(strGPS,"0");
    }  
    
  }
 
}

unsigned int Get_GPS_Data_Size()
{
  if(strGPS[0] == '0')
  {
    return(0);
  }
  else
  {
    strcpy(strCAN_TxGPS,strGPS);
    return(sizeof(strGPS));
  }
}

unsigned int Get_IMU_Data_Size()
{
  if(strIMU[0] == '0')
  {
    return(0);
  }
  else
  {
    strcpy(strCAN_TxGPS,strIMU);
    return(sizeof(strIMU));
  }
}