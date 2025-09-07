// Enhanced MQ-135 Gas Sensor Reading on NodeMCU ESP8266
// This version incorporates hardware-specific ADC voltage, robust constants,
// safer EEPROM handling, and improved readability for better accuracy and maintenance.

#include <EEPROM.h>
#include <math.h> // For fabs, pow, sqrt (Arduino compatibility)

// --- Hardware & ADC Configuration ---
const int MQ_PIN = A0;
// IMPORTANT: NodeMCU ESP8266's ADC has a 1.0V reference voltage internally.
// The analogRead function maps this 0-1.0V range to 0-1023.
const float ADC_REF_VOLTAGE = 1.0;
const int ADC_MAX_VALUE = 1023;

// --- Sensor & Load Configuration ---
const float RL_VALUE = 10.0; // Load resistance in Kilo-Ohms

// --- Calibration & Reading Parameters ---
const int CALIBRATION_SAMPLES = 100;    // Number of samples for calibration
const int CALIBRATION_INTERVAL = 200; // Milliseconds between calibration samples
const int READING_SAMPLES = 20;       // Number of- samples for a single reading
const int PREHEAT_TIME_MS = 30000;    // 30 seconds for sensor preheating
const unsigned long READ_INTERVAL_MS = 2000; // 2 seconds between readings
const bool FORCE_CALIBRATION_ON_BOOT = false; // Set to true to force calibration on every startup
const float CLEAN_AIR_FACTOR = 3.6f;      // Clean air factor for CO2 from datasheet (for R0 calculation)

// --- Outlier Filtering ---
const float FILTER_STDDEV_MULTIPLIER = 1.5; // How many standard deviations from the mean to keep
const int MIN_VALID_SAMPLES_FOR_READ = 5;  // Minimum samples needed before filtering
const int MIN_FILTERED_SAMPLES = 3;      // Minimum samples required after filtering

// --- EEPROM Configuration ---
const int EEPROM_SIZE = 512;
const int R0_EEPROM_ADDR = 0;
const float R0_MAGIC_NUMBER = 12345.67f; // Use 'f' suffix for float literals

// --- Sensor State ---
float R0 = 1.0;
bool isCalibrated = false;
bool isPreheated = false;
unsigned long startTime = 0;
unsigned long lastReadTime = 0;

// --- Gas Curve Data & Thresholds ---
// This data is derived from the MQ-135 datasheet.
struct GasThreshold { float warning, danger; };
struct GasData {
  const char* name;
  float curveA, curveB; // Parameters for calculating PPM: ppm = A * (Rs/R0)^B (log-log scale)
  GasThreshold threshold;
  const char* unit;
};

enum Gas { NH3, NOx, CO2, ALCOHOL, BENZENE, GAS_COUNT };

GasData gasData[GAS_COUNT] = {
  // Gas Name, Curve A, Curve B, {Warning, Danger}, Unit
  {"NH3",     102.2f,  -2.473f, {25.0f,   50.0f},   "ppm"},
  {"NOx",     163.4f,  -2.518f, {5.0f,    10.0f},   "ppm"},
  {"CO2",     110.47f, -2.862f, {1000.0f, 5000.0f}, "ppm"},
  {"Alcohol", 77.255f, -2.715f, {100.0f,  500.0f},  "ppm"},
  {"Benzene", 32.764f, -2.380f, {1.0f,    5.0f},    "ppm"}
};

// --- Function Prototypes ---
void setup();
void loop();
void takeReading();
float readRs();
float calibrateR0();
void saveR0ToEEPROM();
bool loadR0FromEEPROM();
void handleSerialCommands();

// --- Setup ---
void setup() {
  Serial.begin(115200);
  // Wait for Serial to connect, with a 5-second timeout for headless operation
  unsigned long serial_timeout = millis();
  while (!Serial && (millis() - serial_timeout < 5000)) {
      delay(100);
  }

  EEPROM.begin(EEPROM_SIZE);
  startTime = millis();

  Serial.println("\n=== Enhanced MQ-135 Gas Sensor ===");
  Serial.printf("ADC Reference Voltage: %.1fV\n", ADC_REF_VOLTAGE);

  if (loadR0FromEEPROM()) {
    Serial.printf("✓ Loaded R0 from EEPROM = %.3f kΩ\n", R0);
    isCalibrated = true;
  } else {
    Serial.println("⚠ No valid R0 found in EEPROM. Use 'calibrate' command in clean air.");
  }

  if (!isCalibrated && FORCE_CALIBRATION_ON_BOOT) {
    R0 = calibrateR0();
    if (R0 > 0) {
      saveR0ToEEPROM();
      isCalibrated = true;
    }
  }

  Serial.println("Sensor preheating started...");
}

// --- Main Loop ---
void loop() {
  handleSerialCommands();

  if (!isPreheated) {
    unsigned long elapsed = millis() - startTime;
    if (elapsed >= PREHEAT_TIME_MS) {
      isPreheated = true;
      Serial.println("✓ Preheat complete. Sensor is now active.");
    } else {
      static unsigned long lastUpdate = 0;
      if (millis() - lastUpdate >= 5000) { // Update every 5 seconds
        Serial.printf("Preheating... %lu s remaining\n", (PREHEAT_TIME_MS - elapsed) / 1000);
        lastUpdate = millis();
      }
    }
  }

  if (isPreheated && isCalibrated && (millis() - lastReadTime >= READ_INTERVAL_MS)) {
    takeReading();
    lastReadTime = millis();
  }

  delay(50); // Small delay to prevent busy-looping
}

// --- Core Functions ---

/**
 * @brief Takes a sensor reading, calculates gas concentrations, and prints them.
 */
void takeReading() {
  float Rs = readRs();
  if (Rs < 0) {
    Serial.println("⚠ Invalid Rs reading, skipping analysis.");
    return;
  }

  float ratio = Rs / R0;
  Serial.printf("\n[%.1fs] Sensor Rs = %.2f kΩ (Rs/R0 = %.3f)\n", millis() / 1000.0, Rs, ratio);
  Serial.println("--------------------------");

  for (int g = 0; g < GAS_COUNT; g++) {
    float A = gasData[g].curveA;
    float B = gasData[g].curveB;
    // The formula ppm = A * (Rs/R0)^B is a common representation of the sensor's behavior in log-log scale.
    // Ensure your 'A' and 'B' curve parameters match this formula structure.
    float ppm = A * pow(ratio, B);
    if (ppm < 0) ppm = 0; // Prevent negative PPM values

    Serial.printf("%-8s: %7.1f %-4s", gasData[g].name, ppm, gasData[g].unit);
    
    if (ppm >= gasData[g].threshold.danger) {
        Serial.print(" [DANGER]");
    } else if (ppm >= gasData[g].threshold.warning) {
        Serial.print(" [WARNING]");
    } else {
        Serial.print(" [OK]");
    }

    if (g == CO2) {
        Serial.printf(" (%.3f%%)", ppm / 10000.0);
    }
    Serial.println();
  }
  Serial.println("--------------------------");
}

/**
 * @brief Reads the sensor resistance (Rs) with outlier filtering.
 * @return The calculated sensor resistance in kOhms, or -1.0 on failure.
 */
float readRs() {
  float volts[READING_SAMPLES];
  int validSampleCount = 0;

  for (int i = 0; i < READING_SAMPLES; i++) {
    int rawValue = analogRead(MQ_PIN);
    if (rawValue >= 0 && rawValue <= ADC_MAX_VALUE) {
        volts[validSampleCount++] = rawValue * (ADC_REF_VOLTAGE / ADC_MAX_VALUE);
    }
    delay(5);
  }

  if (validSampleCount < MIN_VALID_SAMPLES_FOR_READ) return -1.0;

  // Calculate mean and standard deviation for outlier filtering
  float sum = 0, mean = 0, stdev = 0;
  for (int i = 0; i < validSampleCount; i++) sum += volts[i];
  mean = sum / validSampleCount;

  for (int i = 0; i < validSampleCount; i++) stdev += pow(volts[i] - mean, 2);
  stdev = sqrt(stdev / validSampleCount);

  // Filter out values beyond the standard deviation multiplier
  float filteredSum = 0;
  int filteredCount = 0;
  for (int i = 0; i < validSampleCount; i++) {
    if (fabs(volts[i] - mean) < stdev * FILTER_STDDEV_MULTIPLIER) { // Use fabs for float absolute value
      filteredSum += volts[i];
      filteredCount++;
    }
  }

  if (filteredCount < MIN_FILTERED_SAMPLES) return -1.0; // Ensure enough valid points after filtering

  float avgVoltage = filteredSum / filteredCount;
  
  if (avgVoltage < 0.01) return -1.0; // Prevent division by near-zero voltage

  // VRL = Vout = avgVoltage. We need to calculate Rs.
  // Vout = Vc * RL / (RL + Rs) -> (RL + Rs) = Vc * RL / Vout -> Rs = (Vc * RL / Vout) - RL
  return RL_VALUE * (ADC_REF_VOLTAGE - avgVoltage) / avgVoltage;
}

/**
 * @brief Calibrates the sensor to find R0 in clean air.
 * @return The calculated R0 value, or -1.0 on failure.
 */
float calibrateR0() {
  float totalRs = 0;
  int count = 0;
  Serial.println("Calibrating in clean air... Please wait.");
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float rs = readRs();
    if (rs > 0) {
      totalRs += rs;
      count++;
    }
    // Simple progress indicator
    if (i % 10 == 0) Serial.print(".");
    delay(CALIBRATION_INTERVAL);
  }
  Serial.println("\nCalibration sampling complete.");

  if (count < CALIBRATION_SAMPLES / 2) {
      Serial.println("✗ Calibration failed: Not enough valid readings.");
      return -1.0;
  }
  
  // R0 is Rs in clean air divided by a constant from the datasheet.
  float calculatedR0 = (totalRs / count) / CLEAN_AIR_FACTOR;
  Serial.printf("✓ Calculated R0 = %.3f kΩ\n", calculatedR0);
  return calculatedR0;
}

// --- EEPROM Utility Functions ---

/**
 * @brief Saves the R0 value and a magic number to EEPROM for persistence.
 */
void saveR0ToEEPROM() {
  Serial.printf("Saving R0 = %.3f kΩ to EEPROM...\n", R0);
  EEPROM.put(R0_EEPROM_ADDR, R0_MAGIC_NUMBER);
  EEPROM.put(R0_EEPROM_ADDR + sizeof(float), R0);
  if (EEPROM.commit()) {
    Serial.println("✓ EEPROM save successful.");
  } else {
    Serial.println("✗ EEPROM save failed.");
  }
}

/**
 * @brief Loads R0 from EEPROM if the magic number is valid.
 * @return True if loading was successful, false otherwise.
 */
bool loadR0FromEEPROM() {
  float magic;
  EEPROM.get(R0_EEPROM_ADDR, magic);
  if (fabs(magic - R0_MAGIC_NUMBER) < 0.01) { // Use fabs for float absolute value
    EEPROM.get(R0_EEPROM_ADDR + sizeof(float), R0);
    return R0 > 0.1 && R0 < 100.0; // Sanity check for a reasonable R0 value
  }
  return false;
}

// --- Serial Command Handler ---
void handleSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "calibrate") {
    float newR0 = calibrateR0();
    if (newR0 > 0) {
      R0 = newR0;
      saveR0ToEEPROM();
      isCalibrated = true;
    }
  } else if (cmd == "reset") {
    // Overwrite the magic number and R0 value in EEPROM
    for (unsigned int i = 0; i < sizeof(float) * 2; i++) {
        EEPROM.write(R0_EEPROM_ADDR + i, 0);
    }
    EEPROM.commit();
    isCalibrated = false;
    R0 = 1.0; // Reset runtime value
    Serial.println("✓ EEPROM has been reset. Please 'calibrate' again in clean air.");
  } else if (cmd == "info") {
    Serial.println("\n--- Sensor Info ---");
    Serial.printf("R0 Value: %.3f kΩ\n", R0);
    Serial.printf("Preheated: %s\n", isPreheated ? "Yes" : "No");
    Serial.printf("Calibrated: %s\n", isCalibrated ? "Yes" : "No");
    Serial.println("-------------------");
  } else if (cmd.length() > 0) {
    Serial.println("Unknown command. Available commands: 'calibrate', 'reset', 'info'");
  }
}



