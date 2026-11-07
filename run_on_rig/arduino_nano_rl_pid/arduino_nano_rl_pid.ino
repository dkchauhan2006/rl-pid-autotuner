/*
  arduino_nano_rl_pid.ino  —  firmware for OUR rig (exact parts below). Fast PID "reflex" loop;
  the laptop (run_on_rig/live_tuner.py) is the slow "brain" that sends new Kp, Ki, Kd.

  ─── PARTS ────────────────────────────────────────────────────────────────────────────────
   Board        Nano R3, CH340 USB chip (generic Arduino Nano compatible)
   Temperature  B3950 100K NTC thermistor, glass head, 1 m  (placed at the FARTHEST point from the heater)
   Current      ACS712 30A hall-effect current sensor module (66 mV/A, 2.5 V at 0 A)
   Voltage      Voltage Detection Sensor Module 25V (30k/7.5k divider -> ratio 5)
   Switch       STP55NF06 (P55NF06) N-channel MOSFET, 60 V 50 A, TO-220
   Heater       24 V 40 W ceramic cartridge heater (3D-printer type)

  ─── ASSUMED SPECS (typical values for these parts; we did not have datasheets for our exact units) ───
   Heater   24 V 40 W ceramic cartridge (3D-printer type): R = V²/P = 24²/40 ≈ 14.4 Ω (hot), ≈ 1.67 A at 24 V,
            rated max 24 V. Our step tests were logged with a heater that measured ≈ 3.7 Ω (V/I), so
            live_tuner.py measures R on the rig (A2 / A1) and rescales the voltage it shows the agent.
            With a 14.4 Ω heater keep the supply between ~3.5 V and ~10.8 V to stay inside the power range
            the agent was trained on (1–7 W); above that, re-run the step test and retrain.
   NTC      B3950 100K: R25 = 100 kΩ, Beta = 3950 K. Divider resistor = 100 kΩ: confirmed on our rig from the raw A0 reading (533 counts =
            23 °C room temperature with 100 kΩ; with 4.7 kΩ the same reading would mean 111 °C).
   ACS712   30A version: 66 mV/A, output 2.5 V at 0 A (VCC/2). One ADC step = 4.9 mV ≈ 74 mA, so the reading
            is averaged over every 1 s window (~thousands of samples) to resolve the ~0.5–1.7 A heater current.
   Voltage  25V detection module: 30 kΩ / 7.5 kΩ divider -> ratio 5, max 25 V input (5 V at the Nano pin).
   MOSFET   STP55NF06: Vds 60 V, Id 50 A, Vgs(th) 2–4 V, Rds(on) ≈ 18 mΩ at Vgs = 10 V. NOT logic-level: with
            the Nano's 5 V gate drive Rds(on) is higher, which is fine for ≤ 2 A (it gets warm, add a small heat
            sink). 1 s time-proportioned switching (not fast PWM) keeps switching losses negligible.
   ADC      Nano 10-bit ADC, reference = USB 5 V (measure it and put the real value in ADC_REF).

  ─── PINS (from our wiring) ──────────────────────────────────────────────────────────────
   A0  <- NTC thermistor divider:   5V ── 100kΩ fixed ──┬── A0
                                                        └── NTC 100k ── GND
   A1  <- ACS712-30A  OUT      (VCC 5V, GND; the heater current goes through its screw terminals)
   A2  <- Voltage sensor  S    (its input terminals across the heater SUPPLY, before the MOSFET; "-" to GND)
   D5  -> MOSFET gate through 220 Ω, plus 10 kΩ gate -> GND pull-down
          MOSFET drain -> heater (-), heater (+) -> supply (+), source -> supply GND = Nano GND (common ground!)

  ─── LIBRARIES ── Arduino IDE > Library Manager:  "PID" by Brett Beauregard (PID_v1).  Nothing else.
  ─── BOARD ────── Tools > Board > Arduino Nano;  Processor > ATmega328P (Old Bootloader) for CH340 clones.
                   Windows: install the CH340 driver if no COM port appears.

  ─── SERIAL 9600 ─────────────────────────────────────────────────────────────────────────
   out (1 s):  T:<C>,SP:<C>,E:<C>,U:<%>,V:<volts>,I:<amps>,P:<watts>,A:<raw A0 0-1023>,G:<Kp>,<Ki>,<Kd>
   in:         K:<Kp>,<Ki>,<Kd>   new gains (clamped; SetTunings keeps the integral -> bumpless)
               S:<C>              new setpoint
               M:<%>              manual PWM for step tests,  M:-1 back to automatic
  SAFETY: PID never stops; last gains stay if the laptop disconnects; heater OFF above T_CUTOFF or on a
          broken / shorted thermistor reading.
*/
#include <PID_v1.h>

// ---------------- pins ----------------
const int PIN_TEMP    = A0;   // NTC thermistor divider
const int PIN_CURRENT = A1;   // ACS712-30A
const int PIN_VOLT    = A2;   // 25V voltage sensor module
const int PIN_HEATER  = 5;    // STP55NF06 gate (D5)

// ---------------- calibration (check with a multimeter) ----------------
const float ADC_REF        = 5.0;      // measure the Nano 5V pin and put the real value here
const float VOLT_RATIO     = 5.0;      // 25V module: 30k + 7.5k  -> 5
const float I_SENS_V_PER_A = 0.066;    // ACS712-30A: 66 mV/A  (5A: 0.185, 20A: 0.100)
float       I_ZERO_V       = 2.5;      // auto-measured at start-up with the heater OFF
// NTC B3950 100K
const float NTC_R0   = 100000.0;       // resistance at 25 °C
const float NTC_BETA = 3950.0;
const float NTC_T0_K = 298.15;
const float R_FIXED  = 100000.0;       // fixed resistor of the NTC divider: 100 kΩ (matches room temperature)
const bool  NTC_TO_GND = true;         // true: 5V-R_FIXED-A0-NTC-GND,  false: 5V-NTC-A0-R_FIXED-GND

// ---------------- safety / gain bounds (same as the simulator) ----------------
const float  T_CUTOFF = 110.0;
const double KP_MIN = 1.0, KP_MAX = 300.0, KI_MIN = 0.002, KI_MAX = 3.0, KD_MIN = 0.0, KD_MAX = 3000.0;

double T = 0, setpointC = 45.0, U = 0;   // (not "SP": SP is the AVR stack-pointer macro)
double Kp = 20.0, Ki = 0.05, Kd = 100.0;      // safe start; the agent improves them
PID pid(&T, &U, &setpointC, Kp, Ki, Kd, DIRECT);

const unsigned long WINDOW_MS = 1000;          // heater PWM window = PID sample time = 1 s
unsigned long windowStart = 0, lastTick = 0;
double manualU = -1;
float sumV = 0, sumI = 0, sumAdcT = 0; long nSamp = 0;
bool sensorFault = false;
String rx;

double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
float adcVolts(int pin) { return analogRead(pin) * (ADC_REF / 1023.0); }

// B-parameter equation:  1/T = 1/T0 + ln(R/R0)/B
float ntcCelsius(float adc) {
  if (adc < 2 || adc > 1021) { sensorFault = true; return NAN; }   // open or shorted thermistor
  float ratio = adc / 1023.0;
  float r = NTC_TO_GND ? R_FIXED * ratio / (1.0 - ratio) : R_FIXED * (1.0 - ratio) / ratio;
  float invT = 1.0 / NTC_T0_K + log(r / NTC_R0) / NTC_BETA;
  sensorFault = false;
  return 1.0 / invT - 273.15;
}

void handleLine(const String& s) {
  if (s.startsWith("K:")) {
    int a = s.indexOf(','), b = s.indexOf(',', a + 1);
    if (a < 0 || b < 0) return;
    Kp = clampd(s.substring(2, a).toFloat(), KP_MIN, KP_MAX);
    Ki = clampd(s.substring(a + 1, b).toFloat(), KI_MIN, KI_MAX);
    Kd = clampd(s.substring(b + 1).toFloat(), KD_MIN, KD_MAX);
    pid.SetTunings(Kp, Ki, Kd);
  } else if (s.startsWith("S:")) {
    setpointC = clampd(s.substring(2).toFloat(), 0, T_CUTOFF - 5);
  } else if (s.startsWith("M:")) {
    manualU = s.substring(2).toFloat();
    pid.SetMode(manualU < 0 ? AUTOMATIC : MANUAL);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);
  delay(300);
  float z = 0;                                  // ACS712 zero-current offset, heater OFF
  for (int i = 0; i < 400; i++) { z += adcVolts(PIN_CURRENT); delay(1); }
  I_ZERO_V = z / 400.0;
  float a = 0;                                  // first temperature reading
  for (int i = 0; i < 64; i++) a += analogRead(PIN_TEMP);
  T = ntcCelsius(a / 64.0);
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

  // oversample all analog inputs continuously; averaged once per second (smooths ACS712 noise and PWM ripple)
  sumAdcT += analogRead(PIN_TEMP);
  sumV    += adcVolts(PIN_VOLT) * VOLT_RATIO;
  sumI    += (adcVolts(PIN_CURRENT) - I_ZERO_V) / I_SENS_V_PER_A;
  nSamp++;

  unsigned long now = millis();
  if (now - lastTick >= 1000) {
    lastTick = now;
    float adcT = sumAdcT / nSamp;              // raw A0 average, printed as A: for sensor checks
    float t = ntcCelsius(adcT);
    if (!isnan(t)) T = t;
    float V = sumV / nSamp, I = sumI / nSamp;
    if (I < 0) I = 0;
    sumAdcT = sumV = sumI = 0; nSamp = 0;

    if (manualU >= 0) U = clampd(manualU, 0, 100); else pid.Compute();
    if (T > T_CUTOFF || sensorFault) U = 0;      // hard safety

    Serial.print("T:"); Serial.print(T, 2);
    Serial.print(",SP:"); Serial.print(setpointC, 2);
    Serial.print(",E:"); Serial.print(setpointC - T, 2);
    Serial.print(",U:"); Serial.print(U, 1);
    Serial.print(",V:"); Serial.print(V, 2);
    Serial.print(",I:"); Serial.print(I, 3);
    Serial.print(",P:"); Serial.print(V * I, 3);
    Serial.print(",A:"); Serial.print(adcT, 1);
    Serial.print(",G:"); Serial.print(Kp, 3); Serial.print(","); Serial.print(Ki, 5); Serial.print(","); Serial.println(Kd, 2);
  }

  if (now - windowStart > WINDOW_MS) windowStart += WINDOW_MS;
  digitalWrite(PIN_HEATER, (now - windowStart) < (U / 100.0) * WINDOW_MS ? HIGH : LOW);
}
