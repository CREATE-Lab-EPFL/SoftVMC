// ============================================================
// Arduino Code: 2x Load Cells (HX711)
// Output format: Force1(g),Force2(g)
//
// NOTE on rate: HX711 default is 10 SPS (100ms/sample).
// For ~80 SPS, pull the RATE pin HIGH (board-dependent solder jumper).
// At 10 SPS, each get_units() can block up to 100ms, so two sensors
// in series cap the loop at ~5 Hz. Removing delay() helps; 80 SPS is better.
// ============================================================

#include "HX711.h"

// --- HX711 pin definitions ---
const int LOADCELL_DOUT_PIN_1 = 3;
const int LOADCELL_SCK_PIN_1  = 2;

const int LOADCELL_DOUT_PIN_2 = 5;
const int LOADCELL_SCK_PIN_2  = 4;

HX711 scale1;
HX711 scale2;

float calibration_factor_1 = 1861.078002;
float calibration_factor_2 = 2030.073927;

// Timeout [ms] per sensor read — prevents infinite hang on sensor fault
static const unsigned long READ_TIMEOUT_MS = 150;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing 2x Load Cells...");

  scale1.begin(LOADCELL_DOUT_PIN_1, LOADCELL_SCK_PIN_1);
  scale2.begin(LOADCELL_DOUT_PIN_2, LOADCELL_SCK_PIN_2);

  delay(10);

  if (scale1.is_ready()) Serial.println("Load Cell 1 ready.");
  else Serial.println("Load Cell 1 not found!");

  if (scale2.is_ready()) Serial.println("Load Cell 2 ready.");
  else Serial.println("Load Cell 2 not found!");

  scale1.set_scale(calibration_factor_1);
  scale2.set_scale(calibration_factor_2);

  Serial.println("Taring Load Cells...");
  scale1.tare(10);
  scale2.tare(10);
  Serial.println("Tare done.");

  Serial.println("Force1(g),Force2(g)");
}

void loop() {
  // wait_ready_timeout() avoids blocking forever if a sensor glitches.
  // No delay() — HX711 sample-ready wait is the natural pacing.
  bool r1 = scale1.wait_ready_timeout(READ_TIMEOUT_MS);
  bool r2 = scale2.wait_ready_timeout(READ_TIMEOUT_MS);

  float force1 = r1 ? scale1.get_units(1) : 0.0f;
  float force2 = r2 ? scale2.get_units(1) : 0.0f;

  Serial.print(force1, 4);
  Serial.print(",");
  Serial.println(force2, 4);
}
