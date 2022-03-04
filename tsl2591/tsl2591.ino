/* TSL2591 Digital Light Sensor */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */

#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <Wireling.h>
#include <SD.h>

// Example for demonstrating the TSL2591 library - public domain!

// connect SCL to I2C Clock
// connect SDA to I2C Data
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

// PARAMETERS -----------------------------------------
#define INITIAL_SYNC_PERIOD 700000
#define PHASE_SHIFT_FACTOR 15
#define PERIOD_CHANGE_FACTOR 4
#define MAX_CORRECTION (INITIAL_SYNC_PERIOD / 4)
float period = INITIAL_SYNC_PERIOD;
#define PERIOD_TOLERANCE (INITIAL_SYNC_PERIOD / 2)

#define LED_PIN A0
#define COLOR_ORDER GRB
#define LED_DURATION 2000
int brightness = 1;

#define LDR_THRESHOLD 0.0009
#if defined(ARDUINO_ARCH_AVR)
  #define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#endif

const int SD_CHIP_SELECT = 10;
int flashes_so_far = 0;

// LOG FILE NAMES
char DataFile[]="TLS259.csv";
char hold[]="98";

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

CRGB leds[1];

void blink() {
    leds[0] = CRGB(0,0,0);
    FastLED.show();
    flashes_so_far += 1;
}

void unblink() {
    leds[0] = CRGB(0,0,0);
    FastLED.show();
}

// SUPPLEMENTARY ----------------------
boolean ledOn = false;
unsigned long ledTriggerTime = 0;
unsigned long lastReportTime = 0;

unsigned long getPhase(){
    unsigned long now = micros();
    if (now < ledTriggerTime) return 0;
    unsigned long phase = now - ledTriggerTime;
    return phase;
}

struct filter {
  unsigned long int current;
  unsigned long int sum;
  unsigned long int filtered;
};

filter peers;

void updateFilter(struct filter *value, unsigned int sampleCount){
  value->sum += value->current - value->filtered;
  value->filtered = value->sum / sampleCount;
  value->current = 0;
}


unsigned long INIT_TIME;

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  SerialMonitorInterface.println(F("------------------------------------"));
  SerialMonitorInterface.print  (F("Sensor:       ")); SerialMonitorInterface.println(sensor.name);
  SerialMonitorInterface.print  (F("Driver Ver:   ")); SerialMonitorInterface.println(sensor.version);
  SerialMonitorInterface.print  (F("Unique ID:    ")); SerialMonitorInterface.println(sensor.sensor_id);
  SerialMonitorInterface.print  (F("Max Value:    ")); SerialMonitorInterface.print(sensor.max_value); SerialMonitorInterface.println(F(" lux"));
  SerialMonitorInterface.print  (F("Min Value:    ")); SerialMonitorInterface.print(sensor.min_value); SerialMonitorInterface.println(F(" lux"));
  SerialMonitorInterface.print  (F("Resolution:   ")); SerialMonitorInterface.print(sensor.resolution, 4); SerialMonitorInterface.println(F(" lux"));  
  SerialMonitorInterface.println(F("------------------------------------"));
  SerialMonitorInterface.println(F(""));
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  //tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  tsl.setGain(TSL2591_GAIN_MAX);
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  // tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  SerialMonitorInterface.println(F("------------------------------------"));
  SerialMonitorInterface.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      SerialMonitorInterface.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      SerialMonitorInterface.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      SerialMonitorInterface.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      SerialMonitorInterface.println(F("9876x (Max)"));
      break;
  }
  SerialMonitorInterface.print  (F("Timing:       "));
  SerialMonitorInterface.print((tsl.getTiming() + 1) * 100, DEC); 
  SerialMonitorInterface.println(F(" ms"));
  SerialMonitorInterface.println(F("------------------------------------"));
  SerialMonitorInterface.println(F(""));
}


/**************************************************************************/
/*
    Program entry point for the Arduino sketch
*/
/**************************************************************************/
void setup(void) 
{
  
  Wireling.begin();
  FastLED.addLeds<WS2812, LED_PIN, COLOR_ORDER>(leds, 1);
  FastLED.setBrightness(brightness);
  pinMode(LED_PIN, OUTPUT);

  Wireling.begin();
  Wireling.selectPort(3);
   // MICROSD SETUP
  SerialMonitorInterface.begin(9600);
  SerialMonitorInterface.println("Initializing SD card.");
  if (!SD.begin(SD_CHIP_SELECT)) {
    SerialMonitorInterface.println("Card failed, or not present.");
  }
  else {
    SerialMonitorInterface.println("Card initialized.");
  }
  File Log = SD.open((const char*)DataFile, FILE_WRITE);
  if (Log) {
    Log.println("Time(ms),Phase,Period,PeriodAdjustment,Lux");
    Log.close();
  }

  
  // SENSOR SETUP
  SerialMonitorInterface.println("Starting Adafruit TSL2591 Test!");
 
  if (tsl.begin()) 
  {
    SerialMonitorInterface.println("Found a TSL2591 sensor");
  }
  else 
  {
    SerialMonitorInterface.println("No sensor found ... check your wiring?");
    while (1);
  }
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Configure the sensor */
  configureSensor();
  INIT_TIME = micros();

  // Now we're ready to get readings ... move on to loop()!
}

/**************************************************************************/
/*
    Shows how to perform a basic read on visible, full spectrum or
    infrared light (returns raw 16-bit ADC values)
*/
/**************************************************************************/
void simpleRead(void)
{
  // Simple data read example. Just read the infrared, fullspecrtrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
  //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);

  SerialMonitorInterface.print("[ "); SerialMonitorInterface.print(millis()); SerialMonitorInterface.print(" ms ] ");
  SerialMonitorInterface.print("Luminosity: ");
  SerialMonitorInterface.println(x, DEC);
}

/**************************************************************************/
/*
    Show how to read IR and Full Spectrum at once and convert to lux
*/
/**************************************************************************/
float advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
//  SerialMonitorInterface.println(tsl.calculateLux(full, ir), 6);
//  SerialMonitorInterface.print("[ "); SerialMonitorInterface.print(millis()); SerialMonitorInterface.print(" ms ] ");
//  SerialMonitorInterface.print("IR: "); SerialMonitorInterface.print(ir);  SerialMonitorInterface.print("  ");
//  SerialMonitorInterface.print("Full: "); SerialMonitorInterface.print(full); SerialMonitorInterface.print("  ");
//  SerialMonitorInterface.print("Visible: "); SerialMonitorInterface.print(full - ir); SerialMonitorInterface.print("  ");
//  SerialMonitorInterface.print("Lux: "); SerialMonitorInterface.println(tsl.calculateLux(full, ir), 6);
  float lux = tsl.calculateLux(full, ir);
  if (isnan(lux)) {
      lux = 0.0;
  }
  //SerialMonitorInterface.print(millis()); SerialMonitorInterface.print(" "); SerialMonitorInterface.println(lux, 6);
  return lux;
}

/**************************************************************************/
/*
    Performs a read using the Adafruit Unified Sensor API.
*/
/**************************************************************************/
void unifiedSensorAPIRead(void)
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  SerialMonitorInterface.print("[ "); 
  SerialMonitorInterface.print(event.timestamp); 
  SerialMonitorInterface.print(" ms ] ");
  if ((event.light == 0) |
      (event.light > 4294966000.0) | 
      (event.light <-4294966000.0))
  {
    /* If event.light = 0 lux the sensor is probably saturated */
    /* and no reliable data could be generated! */
    /* if event.light is +/- 4294967040 there was a float over/underflow */
    //SerialMonitorInterface.println("Invalid data (adjust gain or timing)");
  }
  else
  {
    SerialMonitorInterface.print(event.light); SerialMonitorInterface.println(" lux");
  }
}


/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{ 
  //simpleRead(); 
  float lux = advancedRead();
  // unifiedSensorAPIRead();

  boolean received = lux > LDR_THRESHOLD;
  float adjustment = 0;
  if (received) {
    lastReportTime = micros();
    float timediff = micros() - ledTriggerTime;
    if (timediff < period / 2){ 
      // we are too early. 
      adjustment += (float)timediff;
     } else { 
       // we are too late
      adjustment += ((float)timediff - (float) period);
    }
    
    peers.current++;
  
    adjustment /= PHASE_SHIFT_FACTOR;
    adjustment /= max(1, (int)peers.filtered);
    adjustment = constrain(adjustment, -MAX_CORRECTION, MAX_CORRECTION);
    
    ledTriggerTime += long(adjustment);  
    
    period += adjustment / PERIOD_CHANGE_FACTOR;
    period = constrain(period, INITIAL_SYNC_PERIOD - PERIOD_TOLERANCE, INITIAL_SYNC_PERIOD + PERIOD_TOLERANCE);
  }

  unsigned long phase = getPhase();
  
  if (!ledOn && phase > period){
    // phase is done.
    ledOn = true;
    blink();
//    SerialMonitorInterface.print("Would blink: ");
//    SerialMonitorInterface.print("Phase: "); SerialMonitorInterface.print(phase);
//    SerialMonitorInterface.print("Period: "); SerialMonitorInterface.print(period);
//    SerialMonitorInterface.print("Adjustment: "); SerialMonitorInterface.println(adjustment);

    ledTriggerTime = micros();
    phase = getPhase();
     
    //reset the peer count
    updateFilter(&peers, 5);
  } 

  if (ledOn && phase > LED_DURATION){
    ledOn = false;
    unblink();
  }
  File Log = SD.open((const char*)DataFile, FILE_WRITE);
  if (Log) {
      unsigned long t = millis();
      Log.print(t, DEC);
      Log.print(",");
      Log.print(phase/1000, DEC);
      Log.print(",");
      Log.print((period/1000), DEC);
      Log.print(",");
      Log.print(adjustment/1000, 22);
      Log.print(",");
      Log.println(lux, 22);
 
      Log.close();
  }
}
