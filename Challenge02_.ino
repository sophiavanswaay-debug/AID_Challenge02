

#include <math.h>

// ---------------------------------------------------------------- settings --
#define LED_BAR_GROVE     1
#define LED_BAR_NEOPIXEL  2
#define LED_BAR_TYPE      LED_BAR_GROVE      // <- choose your LED bar here

const uint8_t PIN_MIC_RIGHT = 2;     // D2
const uint8_t PIN_MIC_LEFT  = 3;     // D3
const uint8_t PIN_LED_BAR   = 4;     // D4 (data)
const uint8_t PIN_LED_CLK   = 5;     // D5 (clock, Grove LED bar only)

const uint8_t NUM_LEDS      = 10;    // LEDs on the bar
const bool    REVERSE_BAR   = false; // set true if left/right appear swapped

const float   MIC_DISTANCE_M   = 0.16;   // distance between microphones
const float   SPEED_OF_SOUND   = 343.0;  // m/s at ~20 C
const long    DT_OFFSET_US     = 0;      // calibration offset (see header)

// Edge produced by the sound module when it hears a sound.
// Most KY-038 / KY-037 / LM393 modules: RISING. Some modules: FALLING.
const int     TRIGGER_EDGE     = RISING;

const unsigned long WAIT_SECOND_US = 2000;  // max wait for the 2nd mic 
const unsigned long REARM_MS       = 120;   // dead time: let echoes/ringing die out
const unsigned long HOLD_MS        = 1500;  // how long the LED stays on
const bool    ONE_MIC_MEANS_SIDE   = true;  // only one mic triggered -> that side
const float   SMOOTHING            = 0.5;   // 0 = no smoothing, 0.9 = very smooth
const unsigned long SMOOTH_WINDOW_MS = 400; // events closer than this are smoothed

// ------------------------------------------------------------- LED bar code --
#if LED_BAR_TYPE == LED_BAR_GROVE
  #include <Grove_LED_Bar.h>
  // Grove_LED_Bar(clockPin, dataPin, greenToRed)
  Grove_LED_Bar bar(PIN_LED_CLK, PIN_LED_BAR, 0);
#else
  #include <Adafruit_NeoPixel.h>
  Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED_BAR, NEO_GRB + NEO_KHZ800);
#endif

void barBegin() {
#if LED_BAR_TYPE == LED_BAR_GROVE
  bar.begin();
  bar.setBits(0);
#else
  strip.begin();
  strip.setBrightness(60);
  strip.clear();
  strip.show();
#endif
}

// Light one LED (index 0 = left-most). index < 0 switches everything off.
void barShow(int index) {
  if (index >= 0 && REVERSE_BAR) index = NUM_LEDS - 1 - index;
// #if LED_BAR_TYPE == LED_BAR_GROVE
  bar.setBits(index < 0 ? 0 : (1u << index));

}

// Start-up animation: sweep left -> right so the user learns the orientation
void barSweep() {
  for (int i = 0; i < NUM_LEDS; i++) { barShow(i); delay(60); }
  for (int i = NUM_LEDS - 1; i >= 0; i--) { barShow(i); delay(60); }
  barShow(-1);
}

// ------------------------------------------------------ interrupt routines --
volatile unsigned long tRight = 0, tLeft = 0;
volatile bool gotRight = false, gotLeft = false;
volatile bool armed = false;

void isrRight() { if (armed && !gotRight) { tRight = micros(); gotRight = true; } }
void isrLeft()  { if (armed && !gotLeft)  { tLeft  = micros(); gotLeft  = true; } }

void rearm() {
  noInterrupts();
  gotRight = gotLeft = false;
  armed = true;
  interrupts();
}

// ------------------------------------------------------------------- state --
const float MAX_DT_US = MIC_DISTANCE_M / SPEED_OF_SOUND * 1e6;  
float smoothedAngle = 0;
unsigned long lastEventMs = 0;
unsigned long ledOnSinceMs = 0;
bool ledOn = false;

// dt = tLeft - tRight  (positive -> right mic heard it first -> source on the right)
float delayToAngle(long dt) {
  float s = (float)dt / MAX_DT_US;           // = sin(theta)
  s = constrain(s, -1.0, 1.0);
  return asin(s) * 180.0 / PI;               // -90 (left) .. +90 (right)
}

int angleToLed(float angle) {
  // linear in angle: -90 -> 0 , +90 -> NUM_LEDS-1
  float pos = (angle + 90.0) / 180.0 * (NUM_LEDS - 1);
  return constrain((int)lround(pos), 0, NUM_LEDS - 1);
}

// ------------------------------------------------------------------- setup --
void setup() {
  Serial.begin(115200);
  pinMode(PIN_MIC_RIGHT, INPUT);
  pinMode(PIN_MIC_LEFT,  INPUT);
  barBegin();
  barSweep();

  attachInterrupt(digitalPinToInterrupt(PIN_MIC_RIGHT), isrRight, TRIGGER_EDGE);
  attachInterrupt(digitalPinToInterrupt(PIN_MIC_LEFT),  isrLeft,  TRIGGER_EDGE);

  Serial.print(F("Sound Locator ready. Max ITD (us): "));
  Serial.println(MAX_DT_US);
  Serial.println(F("dt_us\tangle_deg\tled"));
  rearm();
}

// -------------------------------------------------------------------- loop --
void loop() {
  // atomic copy of the ISR variables
  noInterrupts();
  bool r = gotRight, l = gotLeft;
  unsigned long tr = tRight, tl = tLeft;
  interrupts();

  if (r || l) {
    bool haveEvent = false;
    float angle = 0;
    long dt = 0;

    if (r && l) {                               // both mics heard the sound
      dt = (long)(tl - tr) - DT_OFFSET_US;
      if (labs(dt) <= (long)(MAX_DT_US * 1.3)) {  // reject physically impossible delays
        angle = delayToAngle(dt);
        haveEvent = true;
      }
      armed = false;
    } else {                                    // only one so far: wait a bit
      unsigned long tFirst = r ? tr : tl;
      if (micros() - tFirst > WAIT_SECOND_US) {
        if (ONE_MIC_MEANS_SIDE) {               // other ear did not hear it at all
          angle = r ? 90.0 : -90.0;
          dt = r ? (long)MAX_DT_US : -(long)MAX_DT_US;
          haveEvent = true;
        }
        armed = false;
      }
    }

    if (!armed) {
      if (haveEvent) {
        unsigned long now = millis();
        if (now - lastEventMs < SMOOTH_WINDOW_MS)
          smoothedAngle = SMOOTHING * smoothedAngle + (1.0 - SMOOTHING) * angle;
        else
          smoothedAngle = angle;
        lastEventMs = now;

        int led = angleToLed(smoothedAngle);
        barShow(led);
        ledOn = true;
        ledOnSinceMs = now;

        Serial.print(dt);            Serial.print('\t');
        Serial.print(smoothedAngle, 1); Serial.print('\t');
        Serial.println(led);
      }
      delay(REARM_MS);               // ignore echoes and module ringing
      rearm();
    }
  }

  // switch the LED off after HOLD_MS of silence
  if (ledOn && millis() - ledOnSinceMs > HOLD_MS) {
    barShow(-1);
    ledOn = false;
  }
}
