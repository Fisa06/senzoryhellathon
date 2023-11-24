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

// Teploměr a vlhkoměr DHT11/22
// připojení knihovny DHT
#include "DHT.h"
// nastavení čísla pinu s připojeným DHT senzorem
#define pinDHT 0
// odkomentování správného typu čidla
#define typDHT11 DHT11 // DHT 11
//#define typDHT22 DHT22 // DHT 22 (AM2302)
// inicializace DHT senzoru s nastaveným pinem a typem senzoru
DHT mojeDHT(pinDHT, typDHT11);
void setup() {
// komunikace přes sériovou linku rychlostí 9600 baud
Serial.begin(115200);
// zapnutí komunikace s teploměrem DHT
mojeDHT.begin();
pinMode(pinDHT, INPUT_PULLUP);
}
void loop() {
// pomocí funkcí readTemperature a readHumidity načteme
// do proměnných tep a vlh informace o teplotě a vlhkosti,
// čtení trvá cca 250 ms
float tep = mojeDHT.readTemperature();
float vlh = mojeDHT.readHumidity();
// kontrola, jestli jsou načtené hodnoty čísla pomocí funkce isnan
if (isnan(tep) || isnan(vlh)) {
 // při chybném čtení vypiš hlášku
 Serial.println("Chyba pri cteni z DHT senzoru!");
} else {
 // pokud jsou hodnoty v pořádku,
 // vypiš je po sériové lince
 Serial.print("Teplota: ");
 Serial.print(tep);
 Serial.print(" stupnu Celsia, ");
 Serial.print("vlhkost: ");
 Serial.print(vlh);
 Serial.println(" %");
}
// pauza pro přehlednější výpis
delay(2000);
}
