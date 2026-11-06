/*
  arduino_nano_rl_pid.ino  —  Arduino NANO firmware for the RL PID auto-tuner rig.

  The Nano is the fast "reflex": every 1 s it reads the FARTHEST thermocouple, runs PID_v1 and drives the heater.
  The laptop (run_on_rig/live_tuner.py) is the slow "brain": it reads this data and sends new Kp, Ki, Kd.

  WIRING (Arduino Nano)
    A0  <- current sensor output   (default: ACS712-5A module, 185 mV/A, 2.5 V at 0 A)
    A1  <- voltage sensor output   (default: 0–25 V "voltage sensor" module = 30k/7.5k divider, ratio 5)
           connect the voltage sensor across the heater SUPPLY (before the MOSFET), so it reads the inlet voltage
    D4  <- MAX6675 SO   (farthest thermocouple from the heater = the controlled temperature)
    D5  -> MAX6675 CS
    D6  -> MAX6675 SCK
    D9  -> heater driver (logic-level MOSFET gate / SSR input). Heater power = duty × V²/R
    5V/GND -> MAX6675, current sensor, voltage sensor
    USB -> laptop

  LIBRARIES (Arduino IDE > Library Manager):  "PID" by Brett Beauregard,  "MAX6675 library" by Adafruit
  BOARD: Tools > Board > Arduino Nano   (old clones: Tools > Processor > ATmega328P (Old Bootloader))

  SERIAL (9600 baud)
    out every 1 s:  T:<C>,SP:<C>,E:<C>,U:<%>,V:<volts>,I:<amps>,P:<watts>,G:<Kp>,<Ki>,<Kd>
    in:             K:<Kp>,<Ki>,<Kd>   new gains (clamped here too; SetTunings keeps the integral -> no bump)
                    S:<C>              new setpoint
                    M:<%>              manual PWM for step tests,  M:-1 back to automatic
  SAFETY: the PID never stops; if the laptop disconnects the last gains stay active; heater OFF above T_CUTOFF.
*/
#include <PID_v1.h>
#include <max6675.h>

// ---------------- pins ----------------
const int PIN_CURRENT = A0;
const int PIN_VOLT    = A1;
const int TC_SO = 4, TC_CS = 5, TC_SCK = 6;
const int PIN_HEATER  = 9;

// ---------------- sensor calibration (CHECK THESE FOR YOUR MODULES) ----------------
const float ADC_REF = 5.0;            // Nano ADC reference (measure your 5 V pin for better accuracy)
const float VOLT_RATIO = 5.0;         // voltage-sensor module 30k/7.5k -> 5.  Own divider: (R1+R2)/R2
const float I_SENS_V_PER_A = 0.185;   // ACS712-05B 0.185, -20A 0.100, -30A 0.066 V/A
float I_ZERO_V = 2.5;                 // auto-calibrated at start-up with the heater OFF

// ---------------- safety and gain bounds (same as the simulator) ----------------
const float T_CUTOFF = 110.0;
const double KP_MIN = 1.0, KP_MAX = 300.0, KI_MIN = 0.002, KI_MAX = 3.0, KD_MIN = 0.0, KD_MAX = 3000.0;

MAX6675 tc(TC_SCK, TC_CS, TC_SO);
double T = 0, SP = 45.0, U = 0;
double Kp = 20.0, Ki = 0.05, Kd = 100.0;      // safe start; the agent improves them
PID pid(&T, &U, &SP, Kp, Ki, Kd, DIRECT);

const unsigned long WINDOW_MS = 1000;          // 1 s PWM window = PID sample time
unsigned long windowStart = 0, lastTick = 0;
double manualU = -1;
float sumV = 0, sumI = 0; long nSamp = 0;      // averaged over the 1 s window (heater switches inside it)
String rx;

double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
float adcVolts(int pin) { return analogRead(pin) * (ADC_REF / 1023.0); }

void handleLine(const String& s) {
  if (s.startsWith("K:")) {
    int a = s.indexOf(','), b = s.indexOf(',', a + 1);
    if (a < 0 || b < 0) return;
    Kp = clampd(s.substring(2, a).toFloat(), KP_MIN, KP_MAX);
    Ki = clampd(s.substring(a + 1, b).toFloat(), KI_MIN, KI_MAX);
    Kd = clampd(s.substring(b + 1).toFloat(), KD_MIN, KD_MAX);
    pid.SetTunings(Kp, Ki, Kd);
  } else if (s.startsWith("S:")) {
    SP = clampd(s.substring(2).toFloat(), 0, T_CUTOFF - 5);
  } else if (s.startsWith("M:")) {
    manualU = s.substring(2).toFloat();
    pid.SetMode(manualU < 0 ? AUTOMATIC : MANUAL);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);
  delay(500);                                  // MAX6675 settle
  float z = 0;                                 // current-sensor zero with the heater OFF
  for (int i = 0; i < 200; i++) { z += adcVolts(PIN_CURRENT); delay(2); }
  I_ZERO_V = z / 200.0;
  pid.SetOutputLimits(0, 100);
  pid.SetSampleTime(1000);
  pid.SetMode(AUTOMATIC);
  windowStart = lastTick = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { rx.trim(); handleLine(rx); rx = ""; } else if (rx.length() < 64) rx += c;
  }

  // continuous sampling of the analog sensors, averaged once per second
  sumV += adcVolts(PIN_VOLT) * VOLT_RATIO;
  sumI += (adcVolts(PIN_CURRENT) - I_ZERO_V) / I_SENS_V_PER_A;
  nSamp++;

  unsigned long now = millis();
  if (now - lastTick >= 1000) {
    lastTick = now;
    double t = tc.readCelsius();
    if (!isnan(t)) T = t;                      // keep the last good reading on a glitch
    if (manualU >= 0) U = clampd(manualU, 0, 100); else pid.Compute();
    if (T > T_CUTOFF) U = 0;
    float V = nSamp ? sumV / nSamp : 0, I = nSamp ? sumI / nSamp : 0;
    if (I < 0) I = 0;
    sumV = sumI = 0; nSamp = 0;
    Serial.print("T:"); Serial.print(T, 2);
    Serial.print(",SP:"); Serial.print(SP, 2);
    Serial.print(",E:"); Serial.print(SP - T, 2);
    Serial.print(",U:"); Serial.print(U, 1);
    Serial.print(",V:"); Serial.print(V, 2);
    Serial.print(",I:"); Serial.print(I, 3);
    Serial.print(",P:"); Serial.print(V * I, 3);
    Serial.print(",G:"); Serial.print(Kp, 3); Serial.print(","); Serial.print(Ki, 5); Serial.print(","); Serial.println(Kd, 2);
  }

  if (now - windowStart > WINDOW_MS) windowStart += WINDOW_MS;
  digitalWrite(PIN_HEATER, (now - windowStart) < (U / 100.0) * WINDOW_MS ? HIGH : LOW);
}
