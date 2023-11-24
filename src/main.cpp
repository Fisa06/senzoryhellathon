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

#include "DHTesp.h" // Click here to get the library: http://librarymanager/All#DHTesp
#include "DFRobot_AS3935_I2C.h"
#include "MQ135.h"

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
//#error Select ESP8266 board.
#endif

#define AirQuality  1

#define IRQ_PIN    0
#define StormI2C    AS3935_ADD3
#define AS3935_MODE 1 // 0-indoor 1-outdoor
#define AS3935_DIST 1
#define AS3935_CAPACITANCE   96
volatile int8_t AS3935IsrTrig = 0;

void AS3935_ISR();

DHTesp dht;
DFRobot_AS3935_I2C  lightning0((uint8_t)IRQ_PIN, (uint8_t)StormI2C);
MQ135 airQuality = MQ135(AirQuality);



void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Status\tHumidity (%)\tTemperature (C)\t(F)\tHeatIndex (C)\t(F)");
  String thisBoard= ARDUINO_BOARD;
  Serial.println(thisBoard);

  // Autodetect is not working reliable, don't use the following line
  // dht.setup(17);
  // use this instead: 
  dht.setup(17, DHTesp::DHT22); // Connect DHT sensor to GPIO 17


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
  delay(dht.getMinimumSamplingPeriod());

  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();

  Serial.print(dht.getStatusString());
  Serial.print("\t");
  Serial.print(humidity, 1);
  Serial.print("\t\t");
  Serial.print(temperature, 1);
  Serial.print("\t\t");
  Serial.print(dht.toFahrenheit(temperature), 1);
  Serial.print("\t\t");
  Serial.print(dht.computeHeatIndex(temperature, humidity, false), 1);
  Serial.print("\t\t");
  Serial.println(dht.computeHeatIndex(dht.toFahrenheit(temperature), humidity, true), 1);
  delay(2000);


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

void AS3935_ISR()
{
  AS3935IsrTrig = 1;
}
