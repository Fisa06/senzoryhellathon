/* #include <Arduino.h>
#include <Wire.h>

void setup()
{
  Wire.begin(20,21);
  Serial.begin(9600);
  while (!Serial);             
  Serial.println("\nI2C Scanner");
}

void loop()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
         Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      nDevices++;
     }
     else if (error==4)
     {
      Serial.print("Unknown error at address 0x");
      if (address<16)
         Serial.print("0");
      Serial.println(address,HEX);
     }    
    }
    if (nDevices == 0)
       Serial.println("No I2C devices found\n");
    else
       Serial.println("done\n");
    delay(5000);           // wait 5 seconds for next scan
} */

#include "DFRobot_AS3935_I2C.h"
#include "MQ135.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include <Adafruit_BMP280.h>

#ifdef ESP32
//#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
//#error Select ESP8266 board.
#endif
//********Kvalita Vzduchu***********

#define AirQuality  1
const int Power = 6;

//***********Blesky**************
#if defined(ESP32) || defined(ESP8266)
#define IRQ_PIN       0
#else
#define IRQ_PIN       2
#endif
// Antenna tuning capcitance (must be integer multiple of 8, 8 - 120 pf)
#define AS3935_CAPACITANCE   32
// Indoor/outdoor mode selection
#define AS3935_INDOORS       0
#define AS3935_OUTDOORS      1
#define AS3935_MODE          AS3935_INDOORS
// Enable/disable disturber detection
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1
#define AS3935_DIST          AS3935_DIST_EN
// I2C address
#define AS3935_I2C_ADDR       AS3935_ADD3
volatile int8_t AS3935IsrTrig = 0;

//**************Teplota***********
const int TempMain = 7;
OneWire oneWireDS(TempMain);
DallasTemperature temp(&oneWireDS);
//***********Vlhkost************
const int DHTpin = 17;

//**************UV index*************
const int UVread = 5;

//**************Dešť*************
const int Rain = 4;

//**************Půda*************
const int Soil = 3;

void AS3935_ISR();

DFRobot_AS3935_I2C  lightning0((uint8_t)IRQ_PIN, (uint8_t)AS3935_I2C_ADDR);
MQ135 airQuality = MQ135(AirQuality);

Adafruit_BMP280* bmp; // I2C

//Adafruit_SHT31 sht31 = Adafruit_SHT31();

float Readings[8] = {0,0,0,0,0,0,0,0}; //Teplota, Vlhkost, Bouřka[km], UVindex, KvalitaVzduchu, Dešť, Půda, Tlak


void setup()
{
  analogReadResolution(8);
  Serial.begin(115200);
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);
  Wire.begin(8,9);
  bmp = new Adafruit_BMP280(&Wire);
  if (!bmp->begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp->setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  delay(500);
  if (bmp->takeForcedMeasurement()) {
    // can now print out the new measurements
    Serial.print(F("Temperature = "));
    Serial.print(bmp->readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp->readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp->readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();
    delay(2000);
  } else {
    Serial.println("Forced measurement failed!");
  }
  delay(10000);
  // Autodetect is not working reliable, don't use the following line
  // dht.setup(17);
  // use this instead: 


 

}

void loop()
{
 /*  // It does nothing until an interrupt is detected on the IRQ pin.
  while (AS3935IsrTrig == 0) {delay(1);}
  delay(5);

  // Reset interrupt flag
  AS3935IsrTrig = 0;

  // Get interrupt source
  uint8_t intSrc = lightning0.getInterruptSrc();
  if (intSrc == 1){
    // Get rid of non-distance data
    uint8_t lightningDistKm = lightning0.getLightningDistKm();
    Readings[2] = lightningDistKm;
    Serial.println("Lightning occurs!");
    Serial.print("Distance: ");
    Serial.print(lightningDistKm);
    Serial.println(" km");

    // Get lightning energy intensity
    uint32_t lightningEnergyVal = lightning0.getStrikeEnergyRaw();
    Serial.print("Intensity: ");
    Serial.print(lightningEnergyVal);
    Serial.println("");
  }else if (intSrc == 2){
    Serial.println("Disturber discovered!");
  }else if (intSrc == 3){
    Serial.println("Noise level too high!");
  } */

  //Teplota

  if(bmp->takeForcedMeasurement()) {
    Readings[0] = bmp->readTemperature();
  }
  //Readings[0] = sht31.readTemperature();
  //Readings[0] = dht.getTemperature();


  //Vlhkost
  //Readings[1] = sht31.readHumidity();
  Readings[1] = 0.0;
  

  Readings[5] = analogRead(Rain);

  Readings[6] = analogRead(Soil);

  if(bmp->takeForcedMeasurement()) {
    Readings[7] = bmp->readPressure();
  }

  //UV
  int UVvalue = analogRead(UVread);
  if( UVvalue < 20){Readings[3] = 0;}
  else if(UVvalue < 46){Readings[3] = 1;}
  else if(UVvalue < 65){Readings[3] = 2;}
  else if(UVvalue < 83){Readings[3] = 3;}
  else if(UVvalue < 103){Readings[3] = 4;}
  else if(UVvalue < 124){Readings[3] = 5;}
  else if(UVvalue < 144){Readings[3] = 6;}
  else if(UVvalue < 162){Readings[3] = 7;}
  else if(UVvalue < 180){Readings[3] = 8;}
  else if(UVvalue < 200){Readings[3] = 9;}
  else if(UVvalue < 221){Readings[3] = 10;}
  else{Readings[3] = 11;}


  Readings[4] = airQuality.getPPM();

  Serial.println(Readings[0]);
  Serial.println(Readings[1]);
  Serial.println(UVvalue);
  Serial.println(Readings[4]);
}

/* void AS3935_ISR()
{
  AS3935IsrTrig = 1;
} */
