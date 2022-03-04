/* TSL2591 Digital Light Sensor */
/* Dynamic Range: 600M:1 */
/* Maximum Lux: 88K */

//#include <FastLED.h>
//#include <Wire.h>
#include <FireflyLED.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
//#include <Wireling.h>
#include <SD.h>

// Example for demonstrating the TSL2591 library - public domain!

// connect SCL to I2C Clock
// connect SDA to I2C Data
// connect Vin to 3.3-5V DC
// connect GROUND to common ground

int brightness = 1;
unsigned long INIT_TIME;

#define LDR_THRESHOLD 0.01
#if defined(ARDUINO_ARCH_AVR)
  #define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#endif

int get_number_of_flashes();
unsigned long get_interval_on();
unsigned long get_interval_off();
unsigned long get_random_interval();


FireflyLED::FireflyLED(int pin, unsigned long interval_on, unsigned long interval_off, int number_of_flashes)
{
    pinMode(pin, OUTPUT);
    this->pin_number = pin;
    this->interval_on = interval_on;
    this->interval_off = interval_off;
    this->interval_off_always = interval_off;
    this->interval_wait = get_random_interval() - ((interval_on + interval_off) * number_of_flashes);
    this->previousMillis = interval_off;
    this->state = LOW;
    this->charge = 0;
    this->charging_threshold = (interval_on + interval_off) * number_of_flashes; // refractory period. should this relate to the interflash-interval? or the female cadence?
    this->brightness = 1; //0-255 //MEGA only
    this->number_of_flashes = number_of_flashes;
    this->flash_counter = number_of_flashes;
    this->is_flashing = 0;
}

void FireflyLED::update_interval_wait() 
{   
    if (this->charge > this->charging_threshold) {
        this->interval_off = 0;
    }
}

int FireflyLED::get_flashing_state()
{
    return this->is_flashing;
}

int FireflyLED::get_number_of_flashes()
{
    return this->flash_counter;
}

void FireflyLED::increment_charge(boolean received)
{
    if (received == false) {
        this->charge += 1;
    }
    else {
        this->charge += 200;
    }
}

void FireflyLED::resample()
{
    this->interval_on = get_interval_on();
    this->interval_off = get_interval_off();
    this->interval_off_always = this->interval_off;
    this->number_of_flashes = get_number_of_flashes();
    this->charging_threshold = (interval_on + interval_off) * number_of_flashes; // refractory period. should this relate to the interflash-interval? or the female cadence?
    this->interval_wait = get_random_interval() - this->charging_threshold;
    this->flash_counter = this->number_of_flashes;
    this->interval_off = this->interval_wait;
}

void FireflyLED::check_and_write(unsigned long currentMillis) 
{ 
    if (this->state == LOW) { // if is off, wait interval_off
        if (float(currentMillis) - float(this->previousMillis) >= float(this->interval_off)) {
            // save the last time you blinked the LED
            this->previousMillis = currentMillis;
            // if the LED is off turn it on and vice-versa:
            this->state = HIGH;
            this->is_flashing = 1;
            this->charge = 0;
            SerialMonitorInterface.println("flash");
            analogWrite(this->pin_number, this->brightness); //MEGA only
            //digitalWrite(this->pin_number, this->state);
        }
    }
    else { // else wait interval_on
        if (float(currentMillis) - float(this->previousMillis) >= float(this->interval_on)) {
            // save the last time you blinked the LED
            this->previousMillis = currentMillis;
            // if the LED is off turn it on and vice-versa:
            this->state = LOW;
            this->is_flashing = 0;
            this->flash_counter -= 1;
            if (this->flash_counter == 0) {
                this->state = LOW;
                this->interval_off = this->interval_wait;
                this->flash_counter = this->number_of_flashes;
                this->resample();
            }
            else if (this->flash_counter <= this->number_of_flashes - 1) {
                this->interval_off = this->interval_off_always;
            }
            analogWrite(this->pin_number, 0); //MEGA only
            //digitalWrite(this->pin_number, this->state);
        } 
    }
}

//from data
unsigned long get_interval_on()
{   
//    randomSeed(millis());
//    int val = random(1, 76);
//    if (val <= 19) {
//        return 80;
//    }
//    else if (19 < val <= 57) {
//        return 100;
//    }
//    else {
//        return 120;
//    }
    return 80;
}

//from data
unsigned long get_interval_off()
{
//    randomSeed(millis());
//    int val = random(1, 43);
//    if (val <= 11) {
//        return 430;
//    }
//    else if (11 < val <= 33) {
//        return 470;
//    }
//    else { return 500; }
    return 430;
}

unsigned long get_random_interval() 
{
//    randomSeed(millis());
//    unsigned long currentM = millis();
//    if ((currentM - START_MILLIS) <= 600000) {
//        return 4000;
//    }
//    else if ((currentM - START_MILLIS) > 600000) { 
//        return 8000;
//    }
//    else if ((currentM - START_MILLIS) > 1200000) { 
//        return 12000;
//    }
//    else if ((currentM - START_MILLIS) > 1800000) { 
//        return 16000;
//    }
//    else if ((currentM - START_MILLIS) > 2400000) { 
//        return 20000;
//    }
//    else if ((currentM - START_MILLIS) > 3000000) { 
//        return 24000;
//    }
//    else if ((currentM - START_MILLIS) > 3600000) { 
//        return 28000;
//    }
//    else {return 32000;}
    return random(12000, 20000);
}

//from data
int get_number_of_flashes()
{   
    //randomSeed(millis());
    int retval = 5; //random(4, 7);
    return retval;
}

//instantiate FireflyLEDs
FireflyLED ff1(3, get_interval_on(), get_interval_off(), get_number_of_flashes());
FireflyLED myLEDs[1] = {ff1};
const int SD_CHIP_SELECT = 7;
int flashes_so_far = 0;

// LOG FILE NAMES
char DataFile[]="TLS259.csv";
char hold[]="98";

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)


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
  tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  //tsl.setGain(TSL2591_GAIN_MAX);
  
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
    Log.println("Time(ms),Lux,Received");
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
  else if (isinf(lux)) {
      lux = 0.0;
  }
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
  int count_flashing = 0;
  unsigned long currentMillis = millis();
  float lux = advancedRead();
  SerialMonitorInterface.print(millis()); SerialMonitorInterface.print(" "); SerialMonitorInterface.println(lux, 6);
  boolean received = lux > LDR_THRESHOLD;
    
  for (byte i = 0; i < sizeof(myLEDs) / sizeof(myLEDs[0]); i = i+1) {
      myLEDs[i].increment_charge(received);
      if (received) {
          if (myLEDs[i].get_number_of_flashes() == 5) {
              myLEDs[i].update_interval_wait();
          }
      }
      myLEDs[i].check_and_write(currentMillis);
    }
  File Log = SD.open((const char*)DataFile, FILE_WRITE);
  if (Log) {
      unsigned long t = millis();
      Log.print(t, DEC);
      Log.print(",");
      Log.print(lux, 22);
      Log.print(",");
      Log.println(int(received));
      Log.close();
  }
}
