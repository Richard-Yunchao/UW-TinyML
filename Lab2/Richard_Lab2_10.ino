#include <PDM.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

// ============================================================
// Smart Workspace Situation Classifier
// ============================================================

// Tunable thresholds
const int MIC_THRESHOLD = 100;
const int LIGHT_THRESHOLD = 120;
const float MOTION_THRESHOLD = 0.20f;
const int PROX_THRESHOLD = 80;

const unsigned long UPDATE_INTERVAL_MS = 3000;

// ---------------------------
// Microphone
// ---------------------------
short sampleBuffer[256];
volatile int samplesRead = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();

  if (bytesAvailable > (int)sizeof(sampleBuffer)) {
    bytesAvailable = sizeof(sampleBuffer);
  }

  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / sizeof(short);
}

int readMicrophoneLevel() {
  noInterrupts();
  int count = samplesRead;

  if (count <= 0) {
    interrupts();
    return 0;
  }

  long sum = 0;
  for (int i = 0; i < count; i++) {
    sum += abs(sampleBuffer[i]);
  }

  samplesRead = 0;
  interrupts();

  return (int)(sum / count);
}

// ---------------------------
// IMU motion metric
// ---------------------------
bool havePreviousAcceleration = false;
float previousX = 0.0f;
float previousY = 0.0f;
float previousZ = 0.0f;

float readMotionMetric() {
  float x, y, z;

  if (!IMU.accelerationAvailable()) {
    return 0.0f;
  }

  IMU.readAcceleration(x, y, z);

  if (!havePreviousAcceleration) {
    previousX = x;
    previousY = y;
    previousZ = z;
    havePreviousAcceleration = true;
    return 0.0f;
  }

  float dx = x - previousX;
  float dy = y - previousY;
  float dz = z - previousZ;

  previousX = x;
  previousY = y;
  previousZ = z;

  return sqrt(dx * dx + dy * dy + dz * dz);
}

// ---------------------------
// Retained APDS9960 readings
// ---------------------------
int lastClear = 0;
int lastProx = 255;
bool haveClearReading = false;
bool haveProxReading = false;

void updateAPDSReadings() {
  if (APDS.colorAvailable()) {
    int r, g, b, clearValue;
    APDS.readColor(r, g, b, clearValue);
    lastClear = clearValue;
    haveClearReading = true;
  }

  if (APDS.proximityAvailable()) {
    lastProx = APDS.readProximity();
    haveProxReading = true;
  }
}

// ---------------------------
// Rule-based classification
// ---------------------------
const char* classifySituation(
    bool sound,
    bool dark,
    bool moving,
    bool near) {

  if (!sound && !dark && !moving && !near) {
    return "QUIET_BRIGHT_STEADY_FAR";
  }

  if (sound && !dark && !moving && !near) {
    return "NOISY_BRIGHT_STEADY_FAR";
  }

  if (!sound && dark && !moving && near) {
    return "QUIET_DARK_STEADY_NEAR";
  }

  if (sound && !dark && moving && near) {
    return "NOISY_BRIGHT_MOVING_NEAR";
  }

  // Fallback mapping for other temporary combinations.
  if (near && sound && moving) {
    return "NOISY_BRIGHT_MOVING_NEAR";
  }

  if (near && dark && !sound) {
    return "QUIET_DARK_STEADY_NEAR";
  }

  if (!near && sound) {
    return "NOISY_BRIGHT_STEADY_FAR";
  }

  return "QUIET_BRIGHT_STEADY_FAR";
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  PDM.onReceive(onPDMdata);

  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM microphone.");
    while (1);
  }
}

void loop() {
  static unsigned long lastUpdate = 0;

  // Poll APDS continuously and retain the most recent valid samples.
  updateAPDSReadings();

  if (millis() - lastUpdate < UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = millis();

  // Avoid printing fake clear=0 or prox=255 values before real
  // APDS9960 samples have arrived.
  if (!haveClearReading || !haveProxReading) {
    return;
  }

  int mic = readMicrophoneLevel();
  int clearValue = lastClear;
  int prox = lastProx;
  float motion = readMotionMetric();

  bool sound = mic >= MIC_THRESHOLD;
  bool dark = clearValue < LIGHT_THRESHOLD;
  bool moving = motion >= MOTION_THRESHOLD;
  bool near = prox <= PROX_THRESHOLD;

  const char* finalLabel =
      classifySituation(sound, dark, moving, near);

  Serial.print("raw,mic=");
  Serial.print(mic);
  Serial.print(",clear=");
  Serial.print(clearValue);
  Serial.print(",motion=");
  Serial.print(motion, 3);
  Serial.print(",prox=");
  Serial.println(prox);

  Serial.print("flag,sound=");
  Serial.print(sound ? 1 : 0);
  Serial.print(",dark=");
  Serial.print(dark ? 1 : 0);
  Serial.print(",moving=");
  Serial.print(moving ? 1 : 0);
  Serial.print(",near=");
  Serial.println(near ? 1 : 0);

  Serial.print("state,");
  Serial.println(finalLabel);
}
