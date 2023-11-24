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

#include <Wire.h>
#include "DFRobot_AS3935_I2C.h"

volatile int8_t AS3935IsrTrig = 0;

#if defined(ESP32) || defined(ESP8266)
#define IRQ_PIN       0
#else
#define IRQ_PIN       2
#endif

// Antenna tuning capcitance (must be integer multiple of 8, 8 - 120 pf)
#define AS3935_CAPACITANCE   96

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

void AS3935_ISR();

DFRobot_AS3935_I2C  lightning0((uint8_t)IRQ_PIN, (uint8_t)AS3935_I2C_ADDR);

void setup()
{

  Serial.begin(115200);
  Serial.println("DFRobot AS3935 lightning sensor begin!");

  while (lightning0.begin() != 0){
    Serial.print(".");
  }
  lightning0.defInit();

  #if defined(ESP32) || defined(ESP8266)
    attachInterrupt(digitalPinToInterrupt(IRQ_PIN),AS3935_ISR,RISING);
  #else
    attachInterrupt(/*Interrupt No*/0,AS3935_ISR,RISING);
  #endif

  // Configure sensor
  lightning0.manualCal(AS3935_CAPACITANCE, AS3935_MODE, AS3935_DIST);
  // Enable interrupt (connect IRQ pin IRQ_PIN: 2, default)

//  Connect the IRQ and GND pin to the oscilloscope.
//  uncomment the following sentences to fine tune the antenna for better performance.
//  This will dispaly the antenna's resonance frequency/16 on IRQ pin (The resonance frequency will be divided by 16 on this pin)
//  Tuning AS3935_CAPACITANCE to make the frequency within 500/16 kHz ± 3.5%
//  lightning0.setLcoFdiv(0);
//  lightning0.setIRQOutputSource(3);

}

void loop()
{
  // It does nothing until an interrupt is detected on the IRQ pin.
  while (AS3935IsrTrig == 0) {delay(1);}
  delay(5);

  // Reset interrupt flag
  AS3935IsrTrig = 0;

  // Get interrupt source
  uint8_t intSrc = lightning0.getInterruptSrc();
  if (intSrc == 1){
    // Get rid of non-distance data
    uint8_t lightningDistKm = lightning0.getLightningDistKm();
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
  }
}
//IRQ handler for AS3935 interrupts
#if defined(ESP32) || defined(ESP8266)
void IRAM_ATTR AS3935_ISR()
#else
void AS3935_ISR()
#endif
{
  AS3935IsrTrig = 1;
}
