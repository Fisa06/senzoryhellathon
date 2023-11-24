
// Teploměr a vlhkoměr DHT11/22
// připojení knihovny DHT
/* #include "DHT.h"
// nastavení čísla pinu s připojeným DHT senzorem
#define pinDHT 0
// odkomentování správného typu čidla
#define typDHT11 DHT11 // DHT 11
//#define typDHT22 DHT22 // DHT 22 (AM2302)
// inicializace DHT senzoru s nastaveným pinem a typem senzoru
DHT myDHT(pinDHT, typDHT11);
void setup() {
// komunikace přes sériovou linku rychlostí 9600 baud
Serial.begin(115200);
// zapnutí komunikace s teploměrem DHT
myDHT.begin();
pinMode(pinDHT, INPUT_PULLUP);
}
void loop() {
// pomocí funkcí readTemperature a readHumidity načteme
// do proměnných tep a vlh informace o teplotě a vlhkosti,
// čtení trvá cca 250 ms
float tep = myDHT.readTemperature();
float vlh = myDHT.readHumidity();
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
} */


#include "DS18B20.h"
#include "onewire.h"
/*
  send message to every sensor on the bus to take a reading
*/
void broadcastConvert() {
  //broadcast that temp conversions should begin, all at once so saves time
  onewireInit();
  onewireWriteByte(0xCC);
  onewireWriteByte(0x44);

  while (1) {
    if (onewireReadBit())
      break;
  }
}

/*
  retrieve temperatures from sensors
*/
float getTemperature(unsigned char* address) {
  //get temperature from the device with address address
  float temperature;
  unsigned char scratchPad[9] = {0,0,0,0,0,0,0,0,0};

  onewireInit();
  onewireWriteByte(0x55);
  unsigned char i;
  for (i = 0; i < 8; i++)
    onewireWriteByte(address[i]);
  onewireWriteByte(0xBE);

  for (i = 0; i < 2; i++) {
    scratchPad[i] = onewireReadByte();
  }
  onewireInit();
  temperature = ((scratchPad[1] * 256) + scratchPad[0])*0.0625;

  return temperature;
}

int getTemperatureInt(unsigned char* address) {
  //get temperature from the device with address address
  int temperature;
  unsigned char scratchPad[9] = {0,0,0,0,0,0,0,0,0};

  onewireInit();
  onewireWriteByte(0x55);
  unsigned char i;
  for (i = 0; i < 8; i++)
    onewireWriteByte(address[i]);
  onewireWriteByte(0xBE);

  for (i = 0; i < 2; i++) {
    scratchPad[i] = onewireReadByte();
  }
  onewireInit();
  temperature = ((scratchPad[1] * 256) + scratchPad[0]);

  return temperature;
}

/*
  retrieve address of sensor and print to terminal
*/
void printSingleAddress() {
  onewireInit();
  //attach one sensor to port 25 and this will print out it's address
  unsigned char address[8]= {0,0,0,0,0,0,0,0};
  onewireWriteByte(0x33);
  unsigned char i;
  for (i = 0; i<8; i++)
    address[i] = onewireReadByte();
  for (i = 0; i<8; i++)
    printf("0x%x,",address[i]);
  
  //check crc
  unsigned char crc = onewireCRC(address, 7);
  printf("crc = %x \r\n",crc);
}
