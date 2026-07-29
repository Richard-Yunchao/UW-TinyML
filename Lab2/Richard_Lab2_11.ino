#include <Arduino_HS300x.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <math.h>
#include <string.h>

// ============================================================
// Task 11: Smart Indoor Event Monitor
// Rule-based event detection using:
//   - humidity and temperature
//   - magnetometer
//   - APDS9960 RGB / clear channels
// ============================================================

// -------------------------
// Timing
// -------------------------
const unsigned long SAMPLE_INTERVAL_MS = 3000;
const unsigned long BASELINE_CALIBRATION_MS = 10000;
const unsigned long EVENT_COOLDOWN_MS = 3000;

// -------------------------
// Thresholds
// -------------------------
const float HUMIDITY_JUMP_THRESHOLD = 3.0f;
const float TEMPERATURE_RISE_THRESHOLD = 1.0f;
const float MAG_SHIFT_THRESHOLD = 35.0f;
const float LIGHT_CHANGE_RATIO_THRESHOLD = 0.30f;
const float COLOR_CHANGE_RATIO_THRESHOLD = 0.25f;

// Slowly adapt the baseline only during normal conditions.
const float BASELINE_ALPHA = 0.02f;

// -------------------------
// Latest valid sensor readings
// -------------------------
float latestMagX = 0.0f;
float latestMagY = 0.0f;
float latestMagZ = 0.0f;
bool haveMagReading = false;

int latestR = 0;
int latestG = 0;
int latestB = 0;
int latestClear = 0;
bool haveColorReading = false;

// -------------------------
// Baseline values
// -------------------------
bool baselineReady = false;
unsigned long calibrationStart = 0;
unsigned long calibrationSamples = 0;

float baseHumidity = 0.0f;
float baseTemperature = 0.0f;
float baseMagX = 0.0f;
float baseMagY = 0.0f;
float baseMagZ = 0.0f;
float baseR = 0.0f;
float baseG = 0.0f;
float baseB = 0.0f;
float baseClear = 0.0f;

unsigned long lastSampleTime = 0;
unsigned long lastEventTime = 0;
const char* lastEventLabel = "BASELINE_NORMAL";

// -------------------------
// Helpers
// -------------------------
float vectorMagnitude(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

float vectorDistance(
    float x1, float y1, float z1,
    float x2, float y2, float z2) {
  float dx = x1 - x2;
  float dy = y1 - y2;
  float dz = z1 - z2;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

float relativeDifference(float current, float baseline) {
  float denominator = fabs(baseline);

  if (denominator < 1.0f) {
    denominator = 1.0f;
  }

  return fabs(current - baseline) / denominator;
}

void pollSensors() {
  // Retain the latest valid magnetometer sample.
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(latestMagX, latestMagY, latestMagZ);
    haveMagReading = true;
  }

  // Retain the latest valid color/light sample.
  // colorAvailable() is not guaranteed to be true during every loop.
  if (APDS.colorAvailable()) {
    APDS.readColor(latestR, latestG, latestB, latestClear);
    haveColorReading = true;
  }
}

void addCalibrationSample(
    float humidity,
    float temperature,
    float magX,
    float magY,
    float magZ,
    int r,
    int g,
    int b,
    int clearValue) {
  calibrationSamples++;
  float n = (float)calibrationSamples;

  if (calibrationSamples == 1) {
    baseHumidity = humidity;
    baseTemperature = temperature;
    baseMagX = magX;
    baseMagY = magY;
    baseMagZ = magZ;
    baseR = r;
    baseG = g;
    baseB = b;
    baseClear = clearValue;
    return;
  }

  baseHumidity += (humidity - baseHumidity) / n;
  baseTemperature += (temperature - baseTemperature) / n;
  baseMagX += (magX - baseMagX) / n;
  baseMagY += (magY - baseMagY) / n;
  baseMagZ += (magZ - baseMagZ) / n;
  baseR += ((float)r - baseR) / n;
  baseG += ((float)g - baseG) / n;
  baseB += ((float)b - baseB) / n;
  baseClear += ((float)clearValue - baseClear) / n;
}

void updateBaseline(
    float humidity,
    float temperature,
    float magX,
    float magY,
    float magZ,
    int r,
    int g,
    int b,
    int clearValue) {
  baseHumidity =
      (1.0f - BASELINE_ALPHA) * baseHumidity +
      BASELINE_ALPHA * humidity;

  baseTemperature =
      (1.0f - BASELINE_ALPHA) * baseTemperature +
      BASELINE_ALPHA * temperature;

  baseMagX =
      (1.0f - BASELINE_ALPHA) * baseMagX +
      BASELINE_ALPHA * magX;
  baseMagY =
      (1.0f - BASELINE_ALPHA) * baseMagY +
      BASELINE_ALPHA * magY;
  baseMagZ =
      (1.0f - BASELINE_ALPHA) * baseMagZ +
      BASELINE_ALPHA * magZ;

  baseR =
      (1.0f - BASELINE_ALPHA) * baseR +
      BASELINE_ALPHA * r;
  baseG =
      (1.0f - BASELINE_ALPHA) * baseG +
      BASELINE_ALPHA * g;
  baseB =
      (1.0f - BASELINE_ALPHA) * baseB +
      BASELINE_ALPHA * b;
  baseClear =
      (1.0f - BASELINE_ALPHA) * baseClear +
      BASELINE_ALPHA * clearValue;
}

void printThreeLines(
    float humidity,
    float temperature,
    float magMagnitude,
    int r,
    int g,
    int b,
    int clearValue,
    bool humidJump,
    bool tempRise,
    bool magShift,
    bool lightOrColorChange,
    const char* finalLabel) {

  Serial.print("raw,rh=");
  Serial.print(humidity, 2);
  Serial.print(",temp=");
  Serial.print(temperature, 2);
  Serial.print(",mag=");
  Serial.print(magMagnitude, 2);
  Serial.print(",r=");
  Serial.print(r);
  Serial.print(",g=");
  Serial.print(g);
  Serial.print(",b=");
  Serial.print(b);
  Serial.print(",clear=");
  Serial.println(clearValue);

  Serial.print("flag,humid_jump=");
  Serial.print(humidJump ? 1 : 0);
  Serial.print(",temp_rise=");
  Serial.print(tempRise ? 1 : 0);
  Serial.print(",mag_shift=");
  Serial.print(magShift ? 1 : 0);
  Serial.print(",light_or_color_change=");
  Serial.println(lightOrColorChange ? 1 : 0);

  Serial.print("event,");
  Serial.println(finalLabel);
}

// -------------------------
// Setup
// -------------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize HS300x sensor.");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  calibrationStart = millis();
}

// -------------------------
// Main loop
// -------------------------
void loop() {
  // Poll sensors continuously so short-lived availability flags are not missed.
  pollSensors();

  if (millis() - lastSampleTime < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleTime = millis();

  // Do not generate fake zeros before the first valid magnetic and color samples.
  if (!haveMagReading || !haveColorReading) {
    return;
  }

  float humidity = HS300x.readHumidity();
  float temperature = HS300x.readTemperature();

  float magMagnitude =
      vectorMagnitude(latestMagX, latestMagY, latestMagZ);

  // -------------------------
  // Initial baseline calibration
  // -------------------------
  if (!baselineReady) {
    addCalibrationSample(
        humidity,
        temperature,
        latestMagX,
        latestMagY,
        latestMagZ,
        latestR,
        latestG,
        latestB,
        latestClear);

    if (millis() - calibrationStart >= BASELINE_CALIBRATION_MS) {
      baselineReady = true;
    }

    printThreeLines(
        humidity,
        temperature,
        magMagnitude,
        latestR,
        latestG,
        latestB,
        latestClear,
        false,
        false,
        false,
        false,
        "BASELINE_NORMAL");

    return;
  }

  // -------------------------
  // Relative changes from baseline
  // -------------------------
  float humidityDelta = humidity - baseHumidity;
  float temperatureDelta = temperature - baseTemperature;

  float magneticShift =
      vectorDistance(
          latestMagX,
          latestMagY,
          latestMagZ,
          baseMagX,
          baseMagY,
          baseMagZ);

  float clearChangeRatio =
      relativeDifference(latestClear, baseClear);

  float rChangeRatio =
      relativeDifference(latestR, baseR);
  float gChangeRatio =
      relativeDifference(latestG, baseG);
  float bChangeRatio =
      relativeDifference(latestB, baseB);

  float largestColorChangeRatio = rChangeRatio;
  if (gChangeRatio > largestColorChangeRatio) {
    largestColorChangeRatio = gChangeRatio;
  }
  if (bChangeRatio > largestColorChangeRatio) {
    largestColorChangeRatio = bChangeRatio;
  }

  // -------------------------
  // Binary event indicators
  // -------------------------
  bool humidJump =
      humidityDelta >= HUMIDITY_JUMP_THRESHOLD;

  bool tempRise =
      temperatureDelta >= TEMPERATURE_RISE_THRESHOLD;

  bool magShift =
      magneticShift >= MAG_SHIFT_THRESHOLD;

  bool lightOrColorChange =
      clearChangeRatio >= LIGHT_CHANGE_RATIO_THRESHOLD ||
      largestColorChangeRatio >= COLOR_CHANGE_RATIO_THRESHOLD;

  // -------------------------
  // Event priority
  // -------------------------
  const char* candidateLabel = "BASELINE_NORMAL";

  if (magShift) {
    candidateLabel = "MAGNETIC_DISTURBANCE_EVENT";
  } else if (humidJump || tempRise) {
    candidateLabel = "BREATH_OR_WARM_AIR_EVENT";
  } else if (lightOrColorChange) {
    candidateLabel = "LIGHT_OR_COLOR_CHANGE_EVENT";
  }

  // -------------------------
  // Cooldown / debounce
  // -------------------------
  const char* finalLabel = candidateLabel;

  bool sameEvent =
      strcmp(candidateLabel, lastEventLabel) == 0;

  bool cooldownActive =
      millis() - lastEventTime < EVENT_COOLDOWN_MS;

  if (strcmp(candidateLabel, "BASELINE_NORMAL") != 0) {
    if (sameEvent && cooldownActive) {
      finalLabel = "BASELINE_NORMAL";
    } else {
      lastEventLabel = candidateLabel;
      lastEventTime = millis();
    }
  }

  // Adapt baseline only when there is no detected event.
  if (strcmp(candidateLabel, "BASELINE_NORMAL") == 0) {
    updateBaseline(
        humidity,
        temperature,
        latestMagX,
        latestMagY,
        latestMagZ,
        latestR,
        latestG,
        latestB,
        latestClear);
  }

  printThreeLines(
      humidity,
      temperature,
      magMagnitude,
      latestR,
      latestG,
      latestB,
      latestClear,
      humidJump,
      tempRise,
      magShift,
      lightOrColorChange,
      finalLabel);
}
