#include <FastLED.h>
#include <Wire.h>
#include <Wireling.h>
#include <SD.h>

// PARAMETERS -----------------------------------------
#define INITIAL_SYNC_PERIOD 700000
#define PHASE_SHIFT_FACTOR 20
#define PERIOD_CHANGE_FACTOR 5
#define MAX_CORRECTION (INITIAL_SYNC_PERIOD / 4)
float period = INITIAL_SYNC_PERIOD;
#define PERIOD_TOLERANCE (INITIAL_SYNC_PERIOD / 2)

#define LED_PIN A0
#define COLOR_ORDER GRB
#define LED_DURATION 2000
int brightness = 1;

#define LDR_THRESHOLD 0.01

// SETUP: LDR -----------------------------------------
#define TSL2572_I2CADDR     0x39
#define   GAIN_1X 0
#define   GAIN_8X 1
#define  GAIN_16X 2
#define GAIN_120X 3

#define GAIN_DIVIDE_6 false
int gain_val = 0;
const int SD_CHIP_SELECT = 10;
int flashes_so_far = 0;

// LOG FILE NAMES
char DataFile[]="TEST41.csv";
char hold[]="98";

#if defined(ARDUINO_ARCH_AVR)
  #define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
  #define SerialMonitorInterface SerialUSB
#endif

void Tsl2572RegisterWrite(byte regAddr, byte regData) {
  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0x80 | regAddr);
  Wire.write(regData);
  Wire.endTransmission();
}

// Initializes the light sensor to be ready for output
void TSL2572Init(uint8_t gain) {
  Tsl2572RegisterWrite( 0x0F, gain );//set gain
  Tsl2572RegisterWrite( 0x01, 0x190 );//400 ms
  Tsl2572RegisterWrite( 0x00, 0x03 );//turn on
  if (GAIN_DIVIDE_6)
    Tsl2572RegisterWrite( 0x0D, 0x04 );//scale gain by 0.16
  if (gain == GAIN_1X)gain_val = 1;
  else if (gain == GAIN_8X)gain_val = 8;
  else if (gain == GAIN_16X)gain_val = 16;
  else if (gain == GAIN_120X)gain_val = 120;
}

float Tsl2572ReadAmbientLight() {
  uint8_t data[4];
  int c0, c1;
  float lux1, lux2, cpl;

  Wire.beginTransmission(TSL2572_I2CADDR);
  Wire.write(0xA0 | 0x14);
  Wire.endTransmission();
  Wire.requestFrom(TSL2572_I2CADDR, 4);
  for (uint8_t i = 0; i < 4; i++)
    data[i] = Wire.read();

  c0 = data[1] << 8 | data[0];
  c1 = data[3] << 8 | data[2];

  cpl = 51.87 * (float)gain_val / 60.0;
  if (GAIN_DIVIDE_6) cpl /= 6.0;
  lux1 = ((float)c0 - (1.87 * (float)c1)) / cpl;
  lux2 = ((0.63 * (float)c0) - (float)c1) / cpl;
  cpl = max(lux1, lux2);
  return max(cpl, 0.0);
}

// SETUP: LED --------------------------

CRGB leds[1];

void blink() {
    leds[0] = CRGB(0,255,0);
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

float listen(){
  float x = Tsl2572ReadAmbientLight();
  return x;
  // SerialMonitorInterface.print("Lux: ");
//  SerialMonitorInterface.println(x);  // DEBUG
//  boolean received = x > LDR_THRESHOLD;
//  if (received) lastReportTime = micros();
//  return received;
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

// ------------------------------------

unsigned long INIT_TIME;

void setup() {

   // MICROSD SETUP
  SerialMonitorInterface.print("Initializing SD card.");
  if (!SD.begin(SD_CHIP_SELECT)) {
    SerialMonitorInterface.println("Card failed, or not present.");
    return;
  }
  SerialMonitorInterface.println("Card initialized.");

//  // Loop over for different trails
//  while(SD.exists((char*)DataFile)){
//    hold[1]+=1;
//    if(hold[1]>9+48){
//      hold[0]+=1;
//      hold[1]=(0+48);
//    }
//    if(hold[0]>9+48){
//      hold[0]=(0+48);
//    }
//    DataFile[1]=hold[1];
//    DataFile[0]=hold[0];
//  }
  SerialMonitorInterface.print("Launch file: ");
  SerialMonitorInterface.print("*");
  for(int i=0;i<10;i++)
    SerialMonitorInterface.print(DataFile[i]);
  SerialMonitorInterface.println("*");

  File Log = SD.open((const char*)DataFile, FILE_WRITE);
  if (Log) {
    Log.println("Time(ms),Phase,Period,PeriodAdjustment,Lux,FlashesSoFar");
    Log.close();
  }

  Wireling.begin();
  FastLED.addLeds<WS2812, LED_PIN, COLOR_ORDER>(leds, 1);
  FastLED.setBrightness(brightness);
  pinMode(LED_PIN, OUTPUT);

  Wireling.begin();
  Wireling.selectPort(3);
  TSL2572Init(GAIN_120X);

  INIT_TIME = micros();
}

void loop() {
  float x = listen();
  boolean received = x > LDR_THRESHOLD;
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
      Log.print(millis());
      Log.print(",");
      Log.print(phase/1000, DEC);
      Log.print(",");
      Log.print((period/1000), DEC);
      Log.print(",");
      Log.print(adjustment/1000, 22);
      Log.print(",");
      Log.print(x, 22);
      Log.print(",");
      Log.println(flashes_so_far, DEC);
      Log.close();
  }
}
