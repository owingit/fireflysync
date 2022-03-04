#include <FastLED.h>
#include <Wire.h>
#include <Wireling.h>

// LDR -------

#define TSL2572_I2CADDR     0x39
#define   GAIN_1X 0
#define   GAIN_8X 1
#define  GAIN_16X 2
#define GAIN_120X 3

#define GAIN_DIVIDE_6 false
int gain_val = 0;

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
  Tsl2572RegisterWrite( 0x01, 0xED );//51.87 ms
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

// LED -----
#define LED_PIN A0
#define COLOR_ORDER GRB
unsigned int brightness = 128;
CRGB leds[1];

void blink() {
    leds[0] = CRGB(255,0,0);
    FastLED.show();
}

void unblink() {
    leds[0] = CRGB(0,0,0);
    FastLED.show();
}

// AUX -----
boolean ledOn = false;
unsigned long ledTriggerTime = 0;
unsigned long lastReportTime = 0;
#define LDR_THRESHOLD 5

unsigned long getPhase(){
    unsigned long now = micros();
    if (now < ledTriggerTime) return 0;
    unsigned long phase = now - ledTriggerTime;
    return phase;
}

boolean listen(){
  float x = Tsl2572ReadAmbientLight();
  // SerialMonitorInterface.print("Lux: ");
  // SerialMonitorInterface.println(x);  // DEBUG
  boolean received = x > LDR_THRESHOLD;
  if (received) lastReportTime = micros();
  return received;
}

unsigned long INIT_TIME;

// MODEL -----
float V = 0.5;
float I = 1; // 2.5;
float eta = 1;
float beta = 0.65;
int N = 10;


// -----------

void setup() {
  Wireling.begin();
  FastLED.addLeds<WS2812, LED_PIN, COLOR_ORDER>(leds, 1);
  FastLED.setBrightness(brightness);
  pinMode(LED_PIN, OUTPUT);

  Wireling.begin();
  Wireling.selectPort(3);
  TSL2572Init(GAIN_16X);

  INIT_TIME = micros();
}

void loop() {
  bool rcvd = listen();
  // the neighbor flashed
  SerialMonitorInterface.println(V);
  if (rcvd) {
    V = min(1, V + (beta / N));
  }
  else {
    float dV = I - eta * V;
    V = V + (dV / (5 * N));
  }
  SerialMonitorInterface.println(V);
  // once emitted, return;
  if (V >= 0.999) {
    blink();
    delay(200); // LED threshold
    V = 0;
    unblink();
    // delay(400);
  }

  delay(2);
}
