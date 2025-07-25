// Enhanced MQ-135 Gas Sensor Reading on NodeMCU ESP8266
// Includes outlier filtering, smarter calibration, and formatted gas analysis

#include <EEPROM.h>

// Configuration
const int MQ_PIN = A0;
const float RL_VALUE = 10.0;
const int CALIBRATION_SAMPLES = 100;
const int CALIBRATION_INTERVAL = 200;
const int READING_SAMPLES = 20;
const int PREHEAT_TIME = 20000;
const unsigned long READ_INTERVAL = 2000;
const bool FORCE_CALIBRATION_ON_BOOT = false;

// EEPROM
const int EEPROM_SIZE = 512;
const int R0_EEPROM_ADDR = 0;
const float R0_MAGIC_NUMBER = 12345.67;

// Sensor state
float R0 = 1.0;
bool isCalibrated = false;
bool isPreheated = false;
unsigned long startTime = 0;
unsigned long lastReadTime = 0;

// Gas definitions
struct GasThreshold { float warning, danger; };
struct GasData {
  const char* name;
  float curveA, curveB;
  GasThreshold threshold;
  const char* unit;
};

enum Gas { NH3, NOx, CO2, ALCOHOL, BENZENE, GAS_COUNT };

GasData gasData[GAS_COUNT] = {
  {"NH3",      102.2,   -2.473,  {25.0,   50.0},   "ppm"},
  {"NOx",      163.4,   -2.518,  {5.0,    10.0},   "ppm"},
  {"CO2",      110.47,  -2.862,  {1000.0, 5000.0}, "ppm"},
  {"Alcohol",  77.255,  -2.715,  {100.0,  500.0},  "ppm"},
  {"Benzene",  32.764,  -2.380,  {1.0,    5.0},    "ppm"}
};

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  startTime = millis();

  Serial.println("\n=== Enhanced MQ-135 Gas Sensor ===");

  if (loadR0FromEEPROM()) {
    Serial.printf("✓ Loaded R0 = %.3f k\u03A9\n", R0);
    isCalibrated = true;
  } else {
    Serial.println("⚠ No valid R0 found. Use 'calibrate' to set.");
  }

  if (!isCalibrated && FORCE_CALIBRATION_ON_BOOT) {
    R0 = calibrateR0();
    if (R0 > 0) {
      saveR0ToEEPROM();
      isCalibrated = true;
    }
  }

  Serial.println("Preheating sensor...");
}

void loop() {
  handleSerialCommands();

  if (!isPreheated) {
    unsigned long elapsed = millis() - startTime;
    if (elapsed >= PREHEAT_TIME) {
      isPreheated = true;
      Serial.println("✓ Preheat complete.");
    } else {
      static unsigned long lastUpdate = 0;
      if (elapsed - lastUpdate >= 5000) {
        Serial.printf("Preheating... %d s remaining\n", (PREHEAT_TIME - elapsed) / 1000);
        lastUpdate = elapsed;
      }
    }
  }

  if (isPreheated && isCalibrated && millis() - lastReadTime >= READ_INTERVAL) {
    takeReading();
    lastReadTime = millis();
  }

  delay(50);
}

void takeReading() {
  float Rs = readRs();
  if (Rs <= 0) {
    Serial.println("⚠ Invalid Rs");
    return;
  }

  float ratio = Rs / R0;
  Serial.printf("\n[%.1fs] Rs = %.2f k\u03A9  (Rs/R0 = %.3f)\n", millis() / 1000.0, Rs, ratio);

  for (int g = 0; g < GAS_COUNT; g++) {
    float A = gasData[g].curveA;
    float B = gasData[g].curveB;
    float ppm = pow(ratio / A, 1.0 / B);
    if (ppm < 0) ppm = 0;

    Serial.printf("%-8s: %7.1f %s", gasData[g].name, ppm, gasData[g].unit);
    if (ppm >= gasData[g].threshold.danger) Serial.print(" ⚠ DANGER");
    else if (ppm >= gasData[g].threshold.warning) Serial.print(" ⚠ WARNING");
    else Serial.print(" ✓ OK");

    if (g == CO2) Serial.printf(" (%.2f%%)", ppm / 10000.0 * 100);
    Serial.println();
  }
  Serial.println("--------------------------");
}

float readRs() {
  float volts[READING_SAMPLES];
  int valid = 0;
  for (int i = 0; i < READING_SAMPLES; i++) {
    int raw = analogRead(MQ_PIN);
    if (raw < 0 || raw > 1023) continue;
    volts[valid++] = raw * 5.0 / 1023.0;
    delay(5);
  }
  if (valid < 5) return -1;

  float sum = 0, mean = 0, stdev = 0;
  for (int i = 0; i < valid; i++) sum += volts[i];
  mean = sum / valid;
  for (int i = 0; i < valid; i++) stdev += pow(volts[i] - mean, 2);
  stdev = sqrt(stdev / valid);

  float filtered = 0;
  int count = 0;
  for (int i = 0; i < valid; i++) {
    if (fabs(volts[i] - mean) < stdev * 1.5) {
      filtered += volts[i];
      count++;
    }
  }
  if (count < 3) return -1;
  float avgV = filtered / count;
  if (avgV <= 0.05 || avgV >= 4.95) return -1;
  return RL_VALUE * (5.0 - avgV) / avgV;
}

float calibrateR0() {
  float total = 0;
  int count = 0;
  Serial.println("Calibrating in clean air...");
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float rs = readRs();
    if (rs > 0) {
      total += rs;
      count++;
    }
    delay(CALIBRATION_INTERVAL);
  }
  if (count < 10) return -1;
  return (total / count) / 3.6;
}

void saveR0ToEEPROM() {
  EEPROM.put(R0_EEPROM_ADDR, R0_MAGIC_NUMBER);
  EEPROM.put(R0_EEPROM_ADDR + 4, R0);
  EEPROM.commit();
}

bool loadR0FromEEPROM() {
  float magic;
  EEPROM.get(R0_EEPROM_ADDR, magic);
  if (abs(magic - R0_MAGIC_NUMBER) < 0.01) {
    EEPROM.get(R0_EEPROM_ADDR + 4, R0);
    return R0 > 0.1 && R0 < 100.0;
  }
  return false;
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); cmd.toLowerCase();
  if (cmd == "calibrate") {
    R0 = calibrateR0();
    if (R0 > 0) {
      saveR0ToEEPROM();
      isCalibrated = true;
      Serial.printf("✓ New R0 = %.3f\n", R0);
    } else Serial.println("✗ Calibration failed");
  } else if (cmd == "reset") {
    for (int i = 0; i < 8; i++) EEPROM.write(R0_EEPROM_ADDR + i, 0);
    EEPROM.commit();
    isCalibrated = false;
    Serial.println("✓ EEPROM reset. Use 'calibrate' again.");
  } else if (cmd == "info") {
    Serial.printf("R0: %.3f k\u03A9\n", R0);
    Serial.printf("Preheated: %s\n", isPreheated ? "Yes" : "No");
    Serial.printf("Calibrated: %s\n", isCalibrated ? "Yes" : "No");
  } else if (cmd.length() > 0) {
    Serial.println("Unknown command. Use: calibrate, reset, info");
  }
}

