/*
  pid_rig.ino  —  fast local PID loop for the heater rig (the "reflex"), tuned live by the RL agent (the "brain").

  Libraries (Arduino Library Manager): "PID" by Brett Beauregard (PID_v1), "MAX6675 library" by Adafruit.

  Every 1 s:   read the FARTHEST thermocouple -> PID_v1 -> heater PWM (time-proportioned SSR window)
               print  T:<degC>,SP:<degC>,E:<degC>,U:<%>,V:<volts>,G:<Kp>,<Ki>,<Kd>
  Serial in:   K:<Kp>,<Ki>,<Kd>   new gains from the agent (bounded, applied with SetTunings -> bumpless)
               S:<degC>           new setpoint
               M:<%>              manual/open-loop output (step tests); M:-1 returns to automatic
  Safety:      PID never stops. If the laptop disconnects, the last gains stay active.
               Over-temperature cutoff and gain bounds are enforced HERE, not only in Python.
*/
#include <PID_v1.h>
#include <max6675.h>

// ---------- pins (change to your wiring) ----------
const int TC_SO = 4, TC_CS = 5, TC_SCK = 6;   // MAX6675 of the farthest sensor (ch1)
const int SSR_PIN = 9;                          // heater driver (SSR / MOSFET)
const int VSENSE_PIN = A0;                      // supply voltage through a divider (optional)
const float VSENSE_RATIO = 11.0;                // e.g. 100k/10k divider -> (100+10)/10

// ---------- safety ----------
const float T_CUTOFF = 110.0;                   // degC: heater forced OFF above this
const double KP_MIN = 1.0, KP_MAX = 300.0;      // same bounds as thermal_pid_env.py
const double KI_MIN = 0.002, KI_MAX = 3.0;
const double KD_MIN = 0.0, KD_MAX = 3000.0;

MAX6675 tc(TC_SCK, TC_CS, TC_SO);
double T = 0, setpointC = 45.0, U = 0;   // (not "SP": SP is the AVR stack-pointer macro)
double Kp = 20.0, Ki = 0.05, Kd = 100.0;        // safe defaults until the agent sends gains
PID pid(&T, &U, &setpointC, Kp, Ki, Kd, DIRECT);

const unsigned long WINDOW_MS = 1000;           // PWM window for the SSR (1 s, same as PID sample time)
unsigned long windowStart = 0, lastPrint = 0;
double manualU = -1;
String rx;

double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

void handleLine(const String& s) {
  if (s.startsWith("K:")) {
    int a = s.indexOf(','), b = s.indexOf(',', a + 1);
    if (a < 0 || b < 0) return;
    Kp = clampd(s.substring(2, a).toFloat(), KP_MIN, KP_MAX);
    Ki = clampd(s.substring(a + 1, b).toFloat(), KI_MIN, KI_MAX);
    Kd = clampd(s.substring(b + 1).toFloat(), KD_MIN, KD_MAX);
    pid.SetTunings(Kp, Ki, Kd);                 // PID_v1 keeps ITerm -> no bump
  } else if (s.startsWith("S:")) {
    setpointC = clampd(s.substring(2).toFloat(), 0, T_CUTOFF - 5);
  } else if (s.startsWith("M:")) {
    manualU = s.substring(2).toFloat();
    pid.SetMode(manualU < 0 ? AUTOMATIC : MANUAL);
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SSR_PIN, OUTPUT);
  pid.SetOutputLimits(0, 100);
  pid.SetSampleTime(1000);
  pid.SetMode(AUTOMATIC);
  delay(500);                                   // MAX6675 needs time after power-up
  windowStart = millis();
}

void loop() {
  while (Serial.available()) {                  // non-blocking serial read
    char c = Serial.read();
    if (c == '\n') { rx.trim(); handleLine(rx); rx = ""; } else if (rx.length() < 64) rx += c;
  }

  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    double t = tc.readCelsius();
    if (!isnan(t)) T = t;                       // keep last good value on a glitch
    if (manualU >= 0) U = clampd(manualU, 0, 100); else pid.Compute();
    if (T > T_CUTOFF) U = 0;                    // hard over-temperature cutoff
    float V = analogRead(VSENSE_PIN) * (5.0 / 1023.0) * VSENSE_RATIO;
    Serial.print("T:"); Serial.print(T, 2); Serial.print(",SP:"); Serial.print(setpointC, 2);
    Serial.print(",E:"); Serial.print(setpointC - T, 2); Serial.print(",U:"); Serial.print(U, 1);
    Serial.print(",V:"); Serial.print(V, 2);
    Serial.print(",G:"); Serial.print(Kp, 3); Serial.print(","); Serial.print(Ki, 5); Serial.print(","); Serial.println(Kd, 2);
  }

  // time-proportioning window: heater ON for U% of each WINDOW_MS
  if (now - windowStart > WINDOW_MS) windowStart += WINDOW_MS;
  digitalWrite(SSR_PIN, (now - windowStart) < (U / 100.0) * WINDOW_MS ? HIGH : LOW);
}
