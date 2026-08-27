#include <Wire.h>

#define FDC1004_ADDR     0x50

#define REG_MEAS1_CONFIG 0x08

#define REG_MEAS1_MSB    0x00
#define REG_MEAS1_LSB    0x01

#define REG_CONFIG       0x0C

#define LED_PIN 6

// Only MEAS1 enabled, 100 S/s, repeat mode
#define WORKING_CONFIG_MEAS1 0b0000010110000000

// CIN1
const uint8_t CIN1_SEL = 0b000;

// Offset - you can modify this but 3 is a good default
const int8_t CAPDAC_OFFSET = 3;

const float CAP_DEADBAND   = 0.7;
const float CAP_HYSTERESIS = 3.0;

#define BATCH_SIZE    10
#define STABLE_WINDOW 20
const float STABLE_MAX_SPREAD  = 8.0;
const float BASELINE_PROXIMITY = 20.0;
const float BASELINE_ALPHA     = 0.08;

struct Taxel {
  int8_t  capdac;

  float   batch[BATCH_SIZE];

  float   stableBuf[STABLE_WINDOW];
  uint8_t stableIdx;
  uint8_t stableCount;

  float   baseline;
  bool    baselineReady;

  bool    pressed;
};

Taxel taxel; 

uint8_t batchCount = 0;

void write16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(FDC1004_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

uint16_t read16(uint8_t reg) {
  Wire.beginTransmission(FDC1004_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)FDC1004_ADDR, (uint8_t)2);
  uint16_t v = ((uint16_t)Wire.read() << 8);
  v |= Wire.read();
  return v;
}

uint16_t singleMeasConfig() {
  // rate=100S/s, repeat=1,enable only MEAS1
  return (0b01 << 10) | (1 << 8) | (1 << 7);
}

float medianOf(const float* buf, uint8_t n) {
  float sorted[STABLE_WINDOW];
  memcpy(sorted, buf, sizeof(float) * n);
  for (uint8_t i = 1; i < n; i++) {
    float key = sorted[i];
    int8_t j = i - 1;
    while (j >= 0 && sorted[j] > key) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = key;
  }
  return sorted[n / 2];
}

void updateBaseline(Taxel& t, float filtered) {
  if (fabs(filtered - t.baseline) > BASELINE_PROXIMITY) return;
  if (t.pressed) return;

  t.stableBuf[t.stableIdx] = filtered;
  t.stableIdx = (t.stableIdx + 1) % STABLE_WINDOW;
  if (t.stableCount < STABLE_WINDOW) t.stableCount++;
  if (t.stableCount < STABLE_WINDOW) return;

  float vmin = t.stableBuf[0], vmax = t.stableBuf[0];
  for (uint8_t i = 1; i < STABLE_WINDOW; i++) {
    if (t.stableBuf[i] < vmin) vmin = t.stableBuf[i];
    if (t.stableBuf[i] > vmax) vmax = t.stableBuf[i];
  }
  if ((vmax - vmin) >= STABLE_MAX_SPREAD) return;

  float target = medianOf(t.stableBuf, STABLE_WINDOW);
  t.baseline = (1.0f - BASELINE_ALPHA) * t.baseline + BASELINE_ALPHA * target;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(100000);
  delay(200);

  // CAPDAC auto-scan
  Serial.println("# CAPDAC scan taxel1 — do not press sensor");

  int8_t  best   = 0;
  int16_t bestRaw = 32767;

  for (int8_t c = 0; c <= 31; c++) {
    write16(REG_MEAS1_CONFIG, (CIN1_SEL << 13) | (0b100 << 10) | ((c & 0x1F) << 5));
    write16(REG_CONFIG, singleMeasConfig());
    delay(30);
    int16_t raw = (int16_t)read16(REG_MEAS1_MSB);
    Serial.print("# taxel1 capdac="); Serial.print(c);
    Serial.print(" raw=");            Serial.println(raw);
    if (raw >= 0 && raw < bestRaw) { bestRaw = raw; best = c; }
  }

  int8_t chosen = best + CAPDAC_OFFSET;
  if (chosen > 31) chosen = 31; // stay in the 5-bit CAPDAC field
  if (chosen < 0)  chosen = 0;
  taxel.capdac = chosen;

  Serial.print("# Selected capdac[1]=");
  Serial.println(taxel.capdac);

  // Start continuous measurement
  write16(REG_MEAS1_CONFIG, (CIN1_SEL << 13) | (0b100 << 10) | ((taxel.capdac & 0x1F) << 5));
  write16(REG_CONFIG, WORKING_CONFIG_MEAS1);
  delay(30);

  pinMode(LED_PIN, OUTPUT);

  Serial.println("READY");
  Serial.println("raw1,filtered1,zeroed1,pressed1");
}

void loop() {
  delay(12);

  int16_t msb  = (int16_t)read16(REG_MEAS1_MSB);
  uint8_t frac = read16(REG_MEAS1_LSB) >> 8;
  float capRaw = (float)msb / 2048.0f + (float)frac / 524288.0f;
  taxel.batch[batchCount] = capRaw;

  batchCount++;
  if (batchCount < BATCH_SIZE) return;
  batchCount = 0;

  Taxel& t = taxel;

  float filtered = medianOf(t.batch, BATCH_SIZE);

  if (!t.baselineReady) {
    t.baseline = filtered;
    t.baselineReady = true;
    Serial.print("# Baseline[1] init: ");
    Serial.println(t.baseline, 3);
  }

  float zeroed = max(0.0f, filtered - t.baseline);

  updateBaseline(t, filtered);

  float offThreshold = CAP_DEADBAND - CAP_HYSTERESIS;
  if (!t.pressed && zeroed >= CAP_DEADBAND)   t.pressed = true;
  if ( t.pressed && zeroed <  offThreshold)   t.pressed = false;

  // CSV output: raw,filtered,zeroed, pressed
  Serial.print(capRaw, 4);   Serial.print(",");
  Serial.print(filtered, 4); Serial.print(",");
  Serial.print(zeroed, 4);   Serial.print(",");
  Serial.println(t.pressed);

  if (zeroed > 0.07){
    digitalWrite(LED_PIN, HIGH);
  }
  else{
    digitalWrite(LED_PIN, LOW);
  }
}
