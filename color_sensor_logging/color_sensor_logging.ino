#include <Wire.h>
#include <Wireling.h>
#include "Adafruit_TCS34725.h"
#include <SD.h>
#include <SPI.h>

bool DEBUG = true;
#define COLOR_SENSOR_PIN 1
#define DELAY_MS 10
const int SD_CHIP_SELECT = 10;

/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();

#if defined(ARDUINO_ARCH_AVR)
  #define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#endif

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

// Variables to hold the values the sensor reads
uint16_t r, g, b, c, colorTemp, lux;

// LOG FILE NAMES
char DataFile[]="051621.csv";
char hold[]="98";

void setup(void) {
  SerialMonitorInterface.begin(9600);
  
  // MICROSD SETUP
  SerialMonitorInterface.print("Initializing SD card.");
  if (!SD.begin(SD_CHIP_SELECT)) {
    SerialMonitorInterface.println("Card failed, or not present.");
    return;
  }
  SerialMonitorInterface.println("Card initialized.");

  // Loop over for different trails
  while(SD.exists((char*)DataFile)){
    hold[1]+=1;
    if(hold[1]>9+48){
      hold[0]+=1;
      hold[1]=(0+48);
    }
    if(hold[0]>9+48){
      hold[0]=(0+48);
    }
    DataFile[1]=hold[1];
    DataFile[0]=hold[0];
  }
  SerialMonitorInterface.print("Launch file: ");
  SerialMonitorInterface.print("*");
  for(int i=0;i<10;i++)
    SerialMonitorInterface.print(DataFile[i]);
  SerialMonitorInterface.println("*");

  File Log = SD.open((const char*)DataFile, FILE_WRITE);
  if (Log) {
    Log.println("Time(ms),Color Temp(K),Lux,R,G,B,C");
    Log.close();
  }

  // COLOR SENSOR SETUP
  Wire.begin();
  Wireling.begin();

  Wireling.selectPort(COLOR_SENSOR_PIN);

  if (!tcs.begin()) {
    SerialMonitorInterface.println("No TCS34725 found on Pin 1");
    while (1);
  }

//   LEDon();
}

void loop(void) {
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);
  float rprime, gprime, bprime;
  rprime = (float) r;
  gprime = (float) g;
  bprime = (float) b;
  tcs.getRGB(&rprime, &gprime, &bprime);
  if (DEBUG) {
      SerialMonitorInterface.print("Color Temp: "); SerialMonitorInterface.print(colorTemp); SerialMonitorInterface.print(" K, ");
      SerialMonitorInterface.print("Lux: "); SerialMonitorInterface.print(lux, DEC); SerialMonitorInterface.print(", ");
      SerialMonitorInterface.print("R: "); SerialMonitorInterface.print(rprime, DEC); SerialMonitorInterface.print(", ");
      SerialMonitorInterface.print("G: "); SerialMonitorInterface.print(gprime, DEC); SerialMonitorInterface.print(", ");
      SerialMonitorInterface.print("B: "); SerialMonitorInterface.print(bprime); SerialMonitorInterface.print(", ");
      SerialMonitorInterface.print("Clr: "); SerialMonitorInterface.print(c, DEC);
      SerialMonitorInterface.println(" ");
  }
  File Log = SD.open((const char*)DataFile, FILE_WRITE);
  if (Log) {
      Log.print(millis());
      Log.print(",");
      Log.print(colorTemp);
      Log.print(",");
      Log.print(lux, DEC);
      Log.print(",");
      Log.print(rprime, DEC);
      Log.print(",");
      Log.print(gprime, DEC);
      Log.print(",");
      Log.print(bprime, DEC);
      Log.print(",");
      Log.println(c, DEC);
      Log.close();
  }
  
  delay(DELAY_MS);
}

// Turn Wireling LEDs on
void LEDon() {
  tcs.setInterrupt(true);
}

// Turn Wireling LEDs off
void LEDoff() {
  tcs.setInterrupt(false);
}
