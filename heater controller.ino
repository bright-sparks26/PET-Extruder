/*
  Selvi Filament Machine – heater controller (replaces REX-C100)
  DharaLabs · v1.0 · Oct 2026

  Hardware
    Arduino Nano (ATmega328P, CH340), powered from +12 V into VIN
    MAX6675 + K-type thermocouple   SO=D12  CS=D10  SCK=D13  VCC=5V  GND
    TM1637 4-digit display          CLK=D2  DIO=D3              VCC=5V  GND
    10k potentiometer (setpoint)    ends 5V/GND, wiper=A0
    SSR-40DD input                  IN+ = D9, IN- = GND  (SSR output switches heater low side)

  Libraries (Arduino IDE > Library Manager)
    "MAX6675 library" by Adafruit
    "TM1637" by Avishay Orpaz  (TM1637Display.h)

  Behaviour
    - Pot sets 150–250 °C in 1 °C steps. Display shows setpoint for 2 s while you turn it,
      otherwise the live block temperature.
    - PID with time-proportioning output: a 1 s window, SSR on for (output %) of it.
    - Safety cut-outs (heater forced OFF, display shows code, latches until reset):
        Err1  thermocouple open / not connected
        Err2  over-temperature (> MAX_SAFE_C)
        Err3  thermal runaway: heater at 100 % for RUNAWAY_S seconds with < 5 °C rise
              (probe fell off the block, heater wire broken, etc.)
*/

#include <max6675.h>
#include <TM1637Display.h>

// ---------- pins ----------
const uint8_t PIN_TC_SO  = 12;
const uint8_t PIN_TC_CS  = 10;
const uint8_t PIN_TC_SCK = 13;
const uint8_t PIN_DISP_CLK = 2;
const uint8_t PIN_DISP_DIO = 3;
const uint8_t PIN_SSR = 9;
const uint8_t PIN_POT = A0;

// ---------- limits ----------
const int   SET_MIN_C   = 150;
const int   SET_MAX_C   = 250;
const float MAX_SAFE_C  = 265.0;   // hard cut-out
const unsigned long RUNAWAY_S = 60;

// ---------- PID (output in %, error in °C) ----------
// Starting values for a 40 W cartridge in a V6 block. Tune: see notes at bottom.
float Kp = 6.0;     // % per °C
float Ki = 0.15;    // % per °C·s
float Kd = 25.0;    // % per (°C/s)

const unsigned long WINDOW_MS = 1000;  // SSR time-proportioning window
const unsigned long SAMPLE_MS = 250;   // MAX6675 needs >= 220 ms between reads

MAX6675 tc(PIN_TC_SCK, PIN_TC_CS, PIN_TC_SO);
TM1637Display disp(PIN_DISP_CLK, PIN_DISP_DIO);

float tempC = 0, lastTemp = 0, integral = 0, output = 0;
int   setC = 215, lastPotSet = -1;
unsigned long lastSample = 0, windowStart = 0, showSetUntil = 0;
unsigned long fullSince = 0; float tempAtFull = 0;
uint8_t fault = 0;

const uint8_t SEG_ERR[] = {
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,  // E
  SEG_E | SEG_G,                          // r
  SEG_E | SEG_G,                          // r
  0
};

int readSetpoint() {
  // average 8 reads to steady the pot
  long sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += analogRead(PIN_POT);
  return map(sum / 8, 0, 1023, SET_MIN_C, SET_MAX_C);
}

void trip(uint8_t code) {
  fault = code;
  digitalWrite(PIN_SSR, LOW);
}

void setup() {
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);
  Serial.begin(115200);
  disp.setBrightness(5);
  delay(500);                       // MAX6675 power-up
  setC = readSetpoint();
  lastPotSet = setC;
  tempC = lastTemp = tc.readCelsius();
  windowStart = millis();
  Serial.println(F("ms,set,temp,out%"));
}

void loop() {
  unsigned long now = millis();

  if (fault) {                      // latched safe state
    digitalWrite(PIN_SSR, LOW);
    uint8_t segs[4] = { SEG_ERR[0], SEG_ERR[1], SEG_ERR[2], disp.encodeDigit(fault) };
    disp.setSegments(segs);
    delay(200);
    return;
  }

  // ---- setpoint from pot (only react to real changes, ±2 °C deadband) ----
  int s = readSetpoint();
  if (abs(s - lastPotSet) >= 2) { lastPotSet = s; setC = s; showSetUntil = now + 2000; }

  // ---- sample + PID ----
  if (now - lastSample >= SAMPLE_MS) {
    float dt = (now - lastSample) / 1000.0;
    lastSample = now;

    float t = tc.readCelsius();
    if (isnan(t)) { trip(1); return; }          // open thermocouple
    tempC = t;
    if (tempC > MAX_SAFE_C) { trip(2); return; }

    float err = setC - tempC;
    float dTemp = (tempC - lastTemp) / dt;      // derivative on measurement (no kick on setpoint change)
    lastTemp = tempC;

    // conditional integration (anti-windup): only integrate when not saturated
    float trial = Kp * err + Ki * integral - Kd * dTemp;
    if ((trial < 100 || err < 0) && (trial > 0 || err > 0)) integral += err * dt;
    output = constrain(Kp * err + Ki * integral - Kd * dTemp, 0, 100);

    // ---- thermal runaway watch ----
    if (output >= 99.5) {
      if (fullSince == 0) { fullSince = now; tempAtFull = tempC; }
      else if (now - fullSince > RUNAWAY_S * 1000UL && tempC - tempAtFull < 5.0) { trip(3); return; }
    } else fullSince = 0;

    Serial.print(now); Serial.print(','); Serial.print(setC); Serial.print(',');
    Serial.print(tempC, 1); Serial.print(','); Serial.println(output, 0);

    disp.showNumberDec(now < showSetUntil ? setC : (int)(tempC + 0.5), false);
  }

  // ---- time-proportioning SSR drive ----
  if (now - windowStart >= WINDOW_MS) windowStart += WINDOW_MS;
  digitalWrite(PIN_SSR, (now - windowStart) < (unsigned long)(output * WINDOW_MS / 100.0) ? HIGH : LOW);
}

/* ---------------- Tuning notes ----------------
  Open Tools > Serial Plotter at 115200 to watch set / temp / out%.
  1. Set Ki = 0, Kd = 0. Raise Kp until temp oscillates steadily around the setpoint, then halve it.
  2. Add Kd until overshoot on a 150 -> 215 step is under ~3 °C.
  3. Add Ki slowly until the steady-state offset disappears within ~1 min.
  Target: ±2 °C at 215 °C with the puller running and the fan on.
  If the reading moves the wrong way when heating, swap the thermocouple wires at the MAX6675 (+/-).
*/
