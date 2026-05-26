#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/*
  ESP32 mecanum controller for Raspberry Pi UART control.
  Paired with pi_mecanum_ir_stepper_control.py.
  The mecanum control logic is intentionally unchanged from the working UART build;
  IR-stop and post-move stepper behavior live on the Raspberry Pi side.

  This sketch preserves the tuned mecanum defaults from the GUI tuning flow:
  - Wheel PID: Kp=50, Ki=6, Kd=0
  - RPM signs: -1 / -1 / -1 / -1
  - Min PWM: 160
  - Position + heading tuning:
      Pos Kp=0.05, Pos Ki=0.0018, Pos Kd=0
      Pos cruise=42, slowdown start=10 cm, final window=1 cm, final min=5 rpm
      No-reverse band=0.8 cm, done tol=0.45 cm, done max wheel rpm=0.9
      Dist overshoot=0, dist scale=2%, dist comp max=8 cm
      Move heading PID: 1.95 / 0.01 / 0.08
      Hold heading PID: 0.65 / 0.08 / 1.20
      Turn overshoot=4.5 deg

  UART protocol:
  - PING seq=N
  - STATUS seq=N
  - ZERO_IMU seq=N
  - CAL_IMU seq=N
  - INIT_IMU seq=N        // calibrate IMU bias, then zero yaw
  - RESET_ENC seq=N
  - MOVE angle=0 dist=120 speed=42 heading=-10.5 seq=N
  - TURN heading=90 speed=12 seq=N
  - STOP seq=N
*/

struct EncoderPins {
  uint8_t a;
  uint8_t b;
  const char* name;
};

struct MotorPins {
  uint8_t in1;
  uint8_t in2;
  const char* name;
  uint8_t encIndex;
  uint8_t ch1;
  uint8_t ch2;
};

struct PIDState {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
  float out;
};

struct CascadePidState {
  float kp;
  float ki;
  float kd;
  float integral;
  float prevError;
  float out;
};

enum CommandMode : uint8_t {
  CMD_IDLE = 0,
  CMD_SEQUENCE = 1,
  CMD_TWIST = 2,
};

enum MovePhase : uint8_t {
  MOVE_IDLE = 0,
  MOVE_TURNING = 1,
  MOVE_DRIVING = 2,
};

EncoderPins encoders[4] = {
    {36, 39, "ENC1 BackLeft"},
    {35, 34, "ENC2 FrontRight"},
    {33, 32, "ENC3 FrontLeft"},
    {25, 26, "ENC4 BackRight"},
};

// Motor mapping preserved from the finalized tuning sketch.
// Index order is BR, FR, BL, FL.
MotorPins motors[4] = {
    {14, 27, "MOTA BackRight", 3, 0, 1},
    {19, 13, "MOTB FrontRight", 1, 2, 3},
    {18, 17, "MOTD BackLeft",   0, 6, 7},
    {4, 16,  "MOTC FrontLeft",  2, 4, 5},
};

volatile int32_t encoderCount[4] = {0, 0, 0, 0};
volatile uint8_t encoderPrevState[4] = {0, 0, 0, 0};

int32_t lastEncoderCount[4] = {0, 0, 0, 0};
float measuredRpmRaw[4] = {0, 0, 0, 0};
float measuredRpm[4] = {0, 0, 0, 0};
float targetRpm[4] = {0, 0, 0, 0};
int16_t pwmCmd[4] = {0, 0, 0, 0};
int8_t rpmSign[4] = {-1, -1, -1, -1};

PIDState pid[4] = {
  {50.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {50.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {50.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  {50.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f},
};

static SemaphoreHandle_t stateMutex = nullptr;
static SemaphoreHandle_t serialMutex = nullptr;
static TaskHandle_t controlTaskHandle = nullptr;
static TaskHandle_t uartRxTaskHandle = nullptr;
static TaskHandle_t telemetryTaskHandle = nullptr;

#define STATE_LOCK()   do { if (stateMutex) xSemaphoreTake(stateMutex, portMAX_DELAY); } while (0)
#define STATE_UNLOCK() do { if (stateMutex) xSemaphoreGive(stateMutex); } while (0)
#define SERIAL_LOCK()   do { if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY); } while (0)
#define SERIAL_UNLOCK() do { if (serialMutex) xSemaphoreGive(serialMutex); } while (0)

const uint32_t UART_BAUD = 115200;
const size_t UART_LINE_MAX = 160;
const uint32_t CONTROL_INTERVAL_MS = 20;
const uint32_t TELEMETRY_INTERVAL_MS = 100;
const uint16_t PWM_FREQ = 1000;
const uint8_t PWM_RES_BITS = 8;
const int PWM_MAX = 255;
const float TARGET_RPM_MAX = 60.0f;
const float COUNTS_PER_WHEEL_REV = 4346.8f;
const float wheelDiameterCm = 9.7f;

const uint8_t MPU6050_ADDR = 0x68;
const float MPU6050_GYRO_Z_SCALE = 65.5f;
const float MPU6050_ACC_SCALE = 16384.0f;
const uint16_t IMU_BIAS_CAL_SAMPLES = 500;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
void pwmAttachPinCompat(uint8_t pin, uint8_t channelHint) {
  (void)channelHint;
  ledcAttach(pin, PWM_FREQ, PWM_RES_BITS);
}

void pwmWriteCompat(uint8_t pin, uint8_t channelHint, uint32_t duty) {
  (void)channelHint;
  ledcWrite(pin, duty);
}
#else
void pwmAttachPinCompat(uint8_t pin, uint8_t channel) {
  ledcSetup(channel, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(pin, channel);
}

void pwmWriteCompat(uint8_t pin, uint8_t channel, uint32_t duty) {
  (void)pin;
  ledcWrite(channel, duty);
}
#endif

bool imuOk = false;
float imuYawDeg = 0.0f;
float imuGyroDps = 0.0f;
float imuGyroDpsFilt = 0.0f;
float headingGyroLpfAlpha = 0.18f;
float imuBiasDps = 0.0f;
float imuAccXg = 0.0f;
float imuAccYg = 0.0f;
float imuAccXYg = 0.0f;
float imuAccAlpha = 0.20f;
float rpmFilterAlpha = 0.02f;

CascadePidState posPid = {0.05f, 0.0018f, 0.0f, 0.0f, 0.0f, 0.0f};
CascadePidState headingPid = {1.95f, 0.01f, 0.08f, 0.0f, 0.0f, 0.0f};
CascadePidState headingHoldPid = {0.65f, 0.08f, 1.20f, 0.0f, 0.0f, 0.0f};

CommandMode commandMode = CMD_IDLE;
MovePhase movePhase = MOVE_IDLE;
bool controllerEnabled = false;
bool holdHeadingAtStop = true;
bool yawHoldInPlaceMode = false;
bool positionDone = false;
uint8_t positionDoneTicks = 0;
bool headingHoldActive = false;
bool headingErrSettledLatch = false;
bool headingRateSettledLatch = false;

float positionCurrentCounts = 0.0f;
float positionErrorCounts = 0.0f;
float positionCmdRpm = 0.0f;
float positionTolCm = 1.0f;
float posCruiseRpm = 42.0f;
float posSlowdownStartCm = 10.0f;
float posFinalWindowCm = 1.0f;
float posFinalMinRpm = 5.0f;
float posCmdSlewRpmPerSec = 220.0f;
float posNoReverseBandCm = 0.8f;
float posIntegralBandCm = 40.0f;
float posDoneMaxWheelRpm = 0.9f;
float posDoneTolCm = 0.45f;
float distOvershootCm = 0.0f;
float distScaleCompPct = 2.0f;
float distCompMaxCm = 8.0f;
float distLongMoveExtraCm = 4.0f;
float distLongMoveStartCm = 80.0f;
float posCmdPrevRpm = 0.0f;

float headingTargetDeg = 0.0f;
float headingErrorDeg = 0.0f;
float headingCorrRpm = 0.0f;
float headingCorrMaxRpm = 20.0f;
int8_t headingCorrSign = -1;
float headingMoveDeadbandDeg = 0.12f;
float headingMoveKiErrBandDeg = 4.0f;
float headingEnableBaseRpm = 0.8f;
float headingHoldMaxRpm = 10.0f;
float headingHoldDeadbandDeg = 2.0f;
float headingHoldExitRateDps = 2.5f;
float yawHoldInPlaceKiScale = 0.50f;
float yawHoldInPlaceKiErrDeg = 8.0f;
float headingCorrSlewRpmPerSec = 70.0f;
float headingHoldStableMs = 0.0f;
float headingCorrCmdPrev = 0.0f;
float turnOvershootDeg = 4.5f;
float headingHoldSettleHoldMs = 320.0f;

float yawHoldWheelKpScale = 0.78f;
float yawHoldWheelKdScale = 0.06f;
int16_t yawHoldWheelPwmMax = 200;
float yawHoldTargetDeadbandRpm = 1.0f;
uint8_t yawHoldBreakawayPwm = 8;
float yawHoldBreakawayErrDeg = 9.0f;
float yawHoldWheelPwmSlewPerSec = 550.0f;
int16_t yawHoldWheelCmdPrev[4] = {0, 0, 0, 0};

float yawHoldCruiseRpm = 50.0f;
float yawHoldFinalCaptureRpm = 6.5f;
float yawHoldFinalMinRpm = 3.2f;
float yawHoldSlowdownStartDeg = 30.0f;
float yawHoldFinalCaptureDeg = 8.0f;

uint8_t minPwm = 160;

uint32_t activeSeq = 0;
float activeMoveAngleDeg = 0.0f;
float activeMoveDistanceCm = 0.0f;
float activeMoveRequestedYawDeg = 0.0f;
float activeMoveAppliedYawDeg = 0.0f;
float activeMoveAppliedDistBiasCm = 0.0f;
float activeMoveCruiseRpm = 42.0f;
float activeTurnMaxRpm = 10.0f;
float activeMoveMix[4] = {1.0f, 1.0f, 1.0f, 1.0f};
float activeMoveDirForward = 1.0f;
float activeMoveDirStrafe = 0.0f;
float activeMoveStartForwardCounts = 0.0f;
float activeMoveStartStrafeCounts = 0.0f;
float activeMoveTargetCounts = 0.0f;
bool activeMoveTurnOnly = false;
uint32_t activeMoveTurnTimeoutMs = 12000;
uint32_t activeMovePhaseStartMs = 0;
float activeMoveTurnNearMs = 0.0f;
float activeMoveTurnCloseMs = 0.0f;

float bodyForwardCounts = 0.0f;
float bodyStrafeCounts = 0.0f;

float twistForwardRpm = 0.0f;
float twistStrafeRpm = 0.0f;
float twistTurnRpm = 0.0f;
uint32_t twistTimeoutMs = 0;
uint32_t twistStartMs = 0;

uint8_t debugYawPhase = 0;
bool debugErrSettled = false;
bool debugRateSettled = false;
float debugRawHeadingErrDeg = 0.0f;
float debugUsedHeadingErrDeg = 0.0f;

bool pendingDoneEvent = false;
uint32_t pendingDoneSeq = 0;
uint8_t pendingDoneCode = 0;

bool isInputOnlyPin(uint8_t pin) {
  return pin == 34 || pin == 35 || pin == 36 || pin == 39;
}

float wrapAngleDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

float shortestAngleErrorDeg(float targetDeg, float currentDeg) {
  float d = (targetDeg - currentDeg) * DEG_TO_RAD;
  return atan2f(sinf(d), cosf(d)) * RAD_TO_DEG;
}

float applyTurnOvershootComp(float requestedYawDeg, float currentYawDeg, float maxBiasDeg, float* appliedBiasDegOut = nullptr) {
  float requested = wrapAngleDeg(requestedYawDeg);
  float absBias = fabsf(maxBiasDeg);
  float err = shortestAngleErrorDeg(requested, currentYawDeg);
  float applied = 0.0f;
  if (absBias > 0.001f && fabsf(err) > 0.8f) {
    applied = (err >= 0.0f ? absBias : -absBias);
  }
  if (appliedBiasDegOut) {
    *appliedBiasDegOut = applied;
  }
  return wrapAngleDeg(requested + applied);
}

uint32_t computePoseTurnTimeoutMs(float targetYawDeg, float currentYawDeg, float maxTurnRpm) {
  float absErrDeg = fabsf(shortestAngleErrorDeg(targetYawDeg, currentYawDeg));
  float rpmForEstimate = max(4.0f, maxTurnRpm);
  float estRateDps = max(6.0f, 0.95f * rpmForEstimate);
  float estTurnMs = (absErrDeg / estRateDps) * 1000.0f;
  float timeoutMs = 1500.0f + 1.60f * estTurnMs;
  float minFloorMs = absErrDeg >= 60.0f ? 12000.0f : 5000.0f;
  return (uint32_t)constrain(timeoutMs, minFloorMs, 30000.0f);
}

float countsToCm(float counts) {
  float countsPerCm = COUNTS_PER_WHEEL_REV / (PI * wheelDiameterCm);
  return counts / countsPerCm;
}

float cmToCounts(float cm) {
  float countsPerCm = COUNTS_PER_WHEEL_REV / (PI * wheelDiameterCm);
  return cm * countsPerCm;
}

float computeDistanceCompCm(float requestedAbsCm) {
  float req = max(0.0f, requestedAbsCm);
  if (req <= 0.5f) {
    return 0.0f;
  }

  float comp = max(0.0f, distOvershootCm);
  if (req >= 20.0f && distScaleCompPct > 0.001f) {
    comp += req * (distScaleCompPct * 0.01f);
  }
  if (req >= max(20.0f, distLongMoveStartCm)) {
    comp += max(0.0f, distLongMoveExtraCm);
  }
  return constrain(comp, 0.0f, max(0.0f, distCompMaxCm));
}

void resetCascadePid(CascadePidState& s) {
  s.integral = 0.0f;
  s.prevError = 0.0f;
  s.out = 0.0f;
}

void resetPidState(uint8_t i) {
  pid[i].integral = 0.0f;
  pid[i].prevError = 0.0f;
  pid[i].out = 0.0f;
}

void resetAllPidStates() {
  for (uint8_t i = 0; i < 4; i++) {
    resetPidState(i);
    yawHoldWheelCmdPrev[i] = 0;
  }
}

void resetHighLevelControllers() {
  resetCascadePid(posPid);
  resetCascadePid(headingPid);
  resetCascadePid(headingHoldPid);
  positionCmdRpm = 0.0f;
  posCmdPrevRpm = 0.0f;
  headingCorrRpm = 0.0f;
  positionDone = false;
  positionDoneTicks = 0;
  headingHoldActive = false;
  headingErrSettledLatch = false;
  headingRateSettledLatch = false;
  headingHoldStableMs = 0.0f;
  headingCorrCmdPrev = 0.0f;
  debugYawPhase = 0;
  debugErrSettled = false;
  debugRateSettled = false;
  debugRawHeadingErrDeg = 0.0f;
  debugUsedHeadingErrDeg = 0.0f;
  activeMoveTurnNearMs = 0.0f;
  activeMoveTurnCloseMs = 0.0f;
  resetAllPidStates();
}

bool imuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool imuReadRegs(uint8_t startReg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  uint8_t read = Wire.requestFrom((int)MPU6050_ADDR, (int)len, (int)true);
  if (read != len) {
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool imuReadGyroZRaw(int16_t& gzRaw) {
  uint8_t data[2];
  if (!imuReadRegs(0x47, data, 2)) {
    return false;
  }
  gzRaw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  return true;
}

bool imuReadAccelXYRaw(int16_t& axRaw, int16_t& ayRaw) {
  uint8_t data[4];
  if (!imuReadRegs(0x3B, data, 4)) {
    return false;
  }
  axRaw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  ayRaw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  return true;
}

void imuZeroYaw() {
  imuYawDeg = 0.0f;
}

bool imuCalibrateBias(uint16_t samples) {
  if (!imuOk || samples < 20) {
    return false;
  }

  float sum = 0.0f;
  uint16_t okCount = 0;
  for (uint16_t i = 0; i < samples; i++) {
    int16_t gzRaw = 0;
    if (imuReadGyroZRaw(gzRaw)) {
      sum += ((float)gzRaw) / MPU6050_GYRO_Z_SCALE;
      okCount++;
    }
    delay(2);
  }

  if (okCount < samples / 2) {
    return false;
  }
  imuBiasDps = sum / (float)okCount;
  return true;
}

bool imuInit() {
  Wire.begin();
  delay(10);

  bool ok = true;
  ok &= imuWriteReg(0x6B, 0x00);
  ok &= imuWriteReg(0x1B, 0x08);
  ok &= imuWriteReg(0x1A, 0x03);
  delay(50);

  imuOk = ok;
  if (imuOk) {
    imuBiasDps = 0.0f;
    imuGyroDps = 0.0f;
    imuYawDeg = 0.0f;
    imuCalibrateBias(400);
  }
  return imuOk;
}

void imuUpdate(float dt) {
  if (!imuOk || dt <= 0.0001f) {
    imuGyroDps = 0.0f;
    imuGyroDpsFilt = 0.0f;
    imuAccXg = 0.0f;
    imuAccYg = 0.0f;
    imuAccXYg = 0.0f;
    return;
  }

  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  if (imuReadAccelXYRaw(axRaw, ayRaw)) {
    float ax = ((float)axRaw) / MPU6050_ACC_SCALE;
    float ay = ((float)ayRaw) / MPU6050_ACC_SCALE;
    float a = constrain(imuAccAlpha, 0.02f, 1.0f);
    imuAccXg += a * (ax - imuAccXg);
    imuAccYg += a * (ay - imuAccYg);
    imuAccXYg = sqrtf(imuAccXg * imuAccXg + imuAccYg * imuAccYg);
  }

  int16_t gzRaw = 0;
  if (!imuReadGyroZRaw(gzRaw)) {
    imuGyroDps = 0.0f;
    return;
  }

  float gzDps = ((float)gzRaw) / MPU6050_GYRO_Z_SCALE;
  imuGyroDps = gzDps - imuBiasDps;
  float a = constrain(headingGyroLpfAlpha, 0.02f, 1.0f);
  imuGyroDpsFilt += a * (imuGyroDps - imuGyroDpsFilt);
  imuYawDeg = wrapAngleDeg(imuYawDeg + imuGyroDps * dt);
}

void setupEncoderPin(uint8_t pin) {
  if (isInputOnlyPin(pin)) {
    pinMode(pin, INPUT);
  } else {
    pinMode(pin, INPUT_PULLUP);
  }
}

void IRAM_ATTR updateEncoder(uint8_t idx) {
  uint8_t state = (digitalRead(encoders[idx].a) << 1) | digitalRead(encoders[idx].b);
  uint8_t prev = encoderPrevState[idx];
  uint8_t trans = (prev << 2) | state;

  switch (trans) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      encoderCount[idx]++;
      break;
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
      encoderCount[idx]--;
      break;
    default:
      break;
  }

  encoderPrevState[idx] = state;
}

void IRAM_ATTR isrEnc0A() { updateEncoder(0); }
void IRAM_ATTR isrEnc0B() { updateEncoder(0); }
void IRAM_ATTR isrEnc1A() { updateEncoder(1); }
void IRAM_ATTR isrEnc1B() { updateEncoder(1); }
void IRAM_ATTR isrEnc2A() { updateEncoder(2); }
void IRAM_ATTR isrEnc2B() { updateEncoder(2); }
void IRAM_ATTR isrEnc3A() { updateEncoder(3); }
void IRAM_ATTR isrEnc3B() { updateEncoder(3); }

int32_t readEncoderCount(uint8_t idx) {
  noInterrupts();
  int32_t c = encoderCount[idx];
  interrupts();
  return c;
}

int32_t readMotorSignedCount(uint8_t wheel) {
  int32_t raw = readEncoderCount(motors[wheel].encIndex);
  return rpmSign[wheel] * (-raw);
}

void resetEncoders() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    encoderCount[i] = 0;
  }
  interrupts();

  for (uint8_t i = 0; i < 4; i++) {
    lastEncoderCount[i] = 0;
    measuredRpmRaw[i] = 0.0f;
    measuredRpm[i] = 0.0f;
  }

  bodyForwardCounts = 0.0f;
  bodyStrafeCounts = 0.0f;
}

void setMotorPwmSigned(uint8_t wheel, int cmd) {
  cmd = constrain(cmd, -PWM_MAX, PWM_MAX);
  pwmCmd[wheel] = cmd;

  const MotorPins& m = motors[wheel];
  if (cmd > 0) {
    pwmWriteCompat(m.in1, m.ch1, (uint32_t)cmd);
    pwmWriteCompat(m.in2, m.ch2, 0);
  } else if (cmd < 0) {
    pwmWriteCompat(m.in1, m.ch1, 0);
    pwmWriteCompat(m.in2, m.ch2, (uint32_t)(-cmd));
  } else {
    pwmWriteCompat(m.in1, m.ch1, 0);
    pwmWriteCompat(m.in2, m.ch2, 0);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    setMotorPwmSigned(i, 0);
  }
}

void updateMeasuredRpm(float dt) {
  if (dt <= 0.0001f) {
    return;
  }

  const float rpmScale = (60.0f / COUNTS_PER_WHEEL_REV) / dt;
  float alpha = constrain(rpmFilterAlpha, 0.02f, 1.0f);
  for (uint8_t i = 0; i < 4; i++) {
    const uint8_t encIdx = motors[i].encIndex;
    int32_t countNow = readEncoderCount(encIdx);
    int32_t delta = countNow - lastEncoderCount[encIdx];
    lastEncoderCount[encIdx] = countNow;
    measuredRpmRaw[i] = ((float)rpmSign[i]) * (-((float)delta) * rpmScale);
    measuredRpm[i] += alpha * (measuredRpmRaw[i] - measuredRpm[i]);
  }
}

void updateBodyCounts() {
  float br = (float)readMotorSignedCount(0);
  float fr = (float)readMotorSignedCount(1);
  float bl = (float)readMotorSignedCount(2);
  float fl = (float)readMotorSignedCount(3);
  bodyForwardCounts = 0.25f * (br + fr + bl + fl);
  bodyStrafeCounts = 0.25f * (-br + fr + bl - fl);
}

float computeMoveProgressCounts() {
  float dForward = bodyForwardCounts - activeMoveStartForwardCounts;
  float dStrafe = bodyStrafeCounts - activeMoveStartStrafeCounts;
  return dForward * activeMoveDirForward + dStrafe * activeMoveDirStrafe;
}

float computeMoveCrossTrackCounts() {
  float dForward = bodyForwardCounts - activeMoveStartForwardCounts;
  float dStrafe = bodyStrafeCounts - activeMoveStartStrafeCounts;
  return -dForward * activeMoveDirStrafe + dStrafe * activeMoveDirForward;
}

void computeMoveMix(float angleDeg) {
  float rad = angleDeg * DEG_TO_RAD;
  activeMoveDirForward = cosf(rad);
  activeMoveDirStrafe = sinf(rad);

  float mix[4] = {
    activeMoveDirForward - activeMoveDirStrafe,
    activeMoveDirForward + activeMoveDirStrafe,
    activeMoveDirForward + activeMoveDirStrafe,
    activeMoveDirForward - activeMoveDirStrafe,
  };

  float maxAbs = 1.0f;
  for (uint8_t i = 0; i < 4; i++) {
    maxAbs = max(maxAbs, fabsf(mix[i]));
  }
  for (uint8_t i = 0; i < 4; i++) {
    activeMoveMix[i] = mix[i] / maxAbs;
  }
}

const char* resultCodeText(uint8_t code) {
  switch (code) {
    case 1: return "completed";
    case 2: return "stopped";
    case 3: return "turn_timeout";
    case 4: return "imu_missing";
    case 5: return "twist_timeout";
    default: return "idle";
  }
}

const char* commandModeText() {
  if (commandMode == CMD_TWIST) {
    return "twist";
  }
  if (commandMode == CMD_SEQUENCE) {
    if (movePhase == MOVE_TURNING) return "turning";
    if (movePhase == MOVE_DRIVING) return "driving";
  }
  return "idle";
}

void queueDoneEvent(uint32_t seq, uint8_t code) {
  pendingDoneEvent = true;
  pendingDoneSeq = seq;
  pendingDoneCode = code;
}

void clearTargets() {
  for (uint8_t i = 0; i < 4; i++) {
    targetRpm[i] = 0.0f;
  }
}

void cancelActiveCommand(uint8_t resultCode) {
  if (commandMode != CMD_IDLE && activeSeq != 0) {
    queueDoneEvent(activeSeq, resultCode);
  }
  commandMode = CMD_IDLE;
  movePhase = MOVE_IDLE;
  controllerEnabled = false;
  yawHoldInPlaceMode = false;
  activeMoveTurnOnly = false;
  twistTimeoutMs = 0;
  clearTargets();
  headingCorrCmdPrev = 0.0f;
  resetHighLevelControllers();
  stopAllMotors();
  activeSeq = 0;
}

void finishSequence(uint8_t resultCode) {
  uint32_t seq = activeSeq;
  commandMode = CMD_IDLE;
  movePhase = MOVE_IDLE;
  controllerEnabled = false;
  yawHoldInPlaceMode = false;
  clearTargets();
  headingCorrCmdPrev = 0.0f;
  stopAllMotors();
  if (seq != 0) {
    queueDoneEvent(seq, resultCode);
  }
  activeSeq = 0;
}

void sendJsonLine(const String& line) {
  SERIAL_LOCK();
  Serial.println(line);
  SERIAL_UNLOCK();
}

String buildTelemetryJson() {
  String s;
  float progressCm = countsToCm(positionCurrentCounts);
  float remainingCm = countsToCm(positionErrorCounts);
  float crossTrackCm = countsToCm(computeMoveCrossTrackCounts());
  s.reserve(420);
  s += "{\"type\":\"telemetry\"";
  s += ",\"mode\":\"" + String(commandModeText()) + "\"";
  s += ",\"seq\":" + String(activeSeq);
  s += ",\"imu\":{\"ok\":" + String(imuOk ? "true" : "false");
  s += ",\"yawDeg\":" + String(imuYawDeg, 3);
  s += ",\"gyroDps\":" + String(imuGyroDps, 3);
  s += ",\"accXg\":" + String(imuAccXg, 4);
  s += ",\"accYg\":" + String(imuAccYg, 4);
  s += ",\"accXYg\":" + String(imuAccXYg, 4);
  s += "}";
  s += ",\"pose\":{\"forwardCm\":" + String(countsToCm(bodyForwardCounts), 2);
  s += ",\"strafeCm\":" + String(countsToCm(bodyStrafeCounts), 2);
  s += ",\"progressCm\":" + String(progressCm, 2);
  s += ",\"remainingCm\":" + String(remainingCm, 2);
  s += ",\"crossTrackCm\":" + String(crossTrackCm, 2);
  s += "}";
  s += ",\"move\":{\"phase\":" + String((int)movePhase);
  s += ",\"angleDeg\":" + String(activeMoveAngleDeg, 2);
  s += ",\"distanceCm\":" + String(activeMoveDistanceCm, 2);
  s += ",\"headingTargetDeg\":" + String(headingTargetDeg, 2);
  s += ",\"headingErrorDeg\":" + String(headingErrorDeg, 2);
  s += ",\"done\":" + String(positionDone ? "true" : "false");
  s += "}";
  s += ",\"rpm\":[" + String(measuredRpm[0], 2) + "," + String(measuredRpm[1], 2) + "," + String(measuredRpm[2], 2) + "," + String(measuredRpm[3], 2) + "]";
  s += ",\"targetRpm\":[" + String(targetRpm[0], 2) + "," + String(targetRpm[1], 2) + "," + String(targetRpm[2], 2) + "," + String(targetRpm[3], 2) + "]";
  s += ",\"pwm\":[" + String(pwmCmd[0]) + "," + String(pwmCmd[1]) + "," + String(pwmCmd[2]) + "," + String(pwmCmd[3]) + "]";
  s += "}";
  return s;
}

String buildDoneJson(uint32_t seq, uint8_t code) {
  String s;
  s.reserve(260);
  s += "{\"type\":\"done\",\"seq\":" + String(seq);
  s += ",\"result\":\"" + String(resultCodeText(code)) + "\"";
  s += ",\"headingDeg\":" + String(imuYawDeg, 2);
  s += ",\"forwardCm\":" + String(countsToCm(bodyForwardCounts), 2);
  s += ",\"strafeCm\":" + String(countsToCm(bodyStrafeCounts), 2);
  s += ",\"progressCm\":" + String(countsToCm(positionCurrentCounts), 2);
  s += "}";
  return s;
}

String buildAckJson(uint32_t seq, const char* cmd, bool ok, const String& message) {
  String s;
  s.reserve(220);
  s += "{\"type\":\"ack\",\"seq\":" + String(seq);
  s += ",\"cmd\":\"" + String(cmd) + "\"";
  s += ",\"ok\":" + String(ok ? "true" : "false");
  s += ",\"message\":\"" + message + "\"}";
  return s;
}

void normalizeMoveRequest(float& angleDeg, float& distCm) {
  if (distCm < 0.0f) {
    distCm = -distCm;
    angleDeg = wrapAngleDeg(angleDeg + 180.0f);
  }
  angleDeg = wrapAngleDeg(angleDeg);
}

void startMoveSequence(uint32_t seq, float angleDeg, float distCm, float cruiseRpm, float headingDeg, uint32_t timeoutMs) {
  normalizeMoveRequest(angleDeg, distCm);
  cancelActiveCommand(2);

  activeSeq = seq;
  activeMoveAngleDeg = angleDeg;
  activeMoveDistanceCm = distCm;
  activeMoveCruiseRpm = constrain(cruiseRpm, posFinalMinRpm, TARGET_RPM_MAX);
  activeTurnMaxRpm = constrain(cruiseRpm, headingHoldMaxRpm, TARGET_RPM_MAX);
  activeMoveRequestedYawDeg = wrapAngleDeg(headingDeg);
  activeMoveAppliedYawDeg = 0.0f;
  headingTargetDeg = applyTurnOvershootComp(activeMoveRequestedYawDeg, imuYawDeg, turnOvershootDeg, &activeMoveAppliedYawDeg);

  updateBodyCounts();
  activeMoveStartForwardCounts = bodyForwardCounts;
  activeMoveStartStrafeCounts = bodyStrafeCounts;

  activeMoveAppliedDistBiasCm = (distCm > 0.0f) ? computeDistanceCompCm(distCm) : 0.0f;
  activeMoveTargetCounts = cmToCounts(distCm + activeMoveAppliedDistBiasCm);
  activeMoveTurnOnly = distCm <= 0.5f;
  activeMoveTurnTimeoutMs = timeoutMs > 0 ? constrain(timeoutMs, 2000UL, 30000UL) : computePoseTurnTimeoutMs(headingTargetDeg, imuYawDeg, activeTurnMaxRpm);
  activeMovePhaseStartMs = millis();
  movePhase = MOVE_TURNING;
  commandMode = CMD_SEQUENCE;
  controllerEnabled = true;
  holdHeadingAtStop = true;
  yawHoldInPlaceMode = true;
  computeMoveMix(activeMoveAngleDeg);
  resetHighLevelControllers();
  positionDone = true;
  positionDoneTicks = 5;
}

void startTwist(uint32_t seq, float forwardRpm, float strafeRpm, float turnRpm, uint32_t timeoutMs) {
  cancelActiveCommand(2);
  activeSeq = seq;
  commandMode = CMD_TWIST;
  movePhase = MOVE_IDLE;
  controllerEnabled = false;
  yawHoldInPlaceMode = false;
  twistForwardRpm = constrain(forwardRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  twistStrafeRpm = constrain(strafeRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  twistTurnRpm = constrain(turnRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  twistTimeoutMs = constrain(timeoutMs, 100UL, 60000UL);
  twistStartMs = millis();
  resetHighLevelControllers();
}

void updateTwistTargets() {
  clearTargets();
  targetRpm[0] = constrain(twistForwardRpm - twistStrafeRpm - twistTurnRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[1] = constrain(twistForwardRpm + twistStrafeRpm - twistTurnRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[2] = constrain(twistForwardRpm + twistStrafeRpm + twistTurnRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[3] = constrain(twistForwardRpm - twistStrafeRpm + twistTurnRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);

  if (twistTimeoutMs > 0 && (millis() - twistStartMs) >= twistTimeoutMs) {
    uint32_t seq = activeSeq;
    commandMode = CMD_IDLE;
    clearTargets();
    stopAllMotors();
    if (seq != 0) {
      queueDoneEvent(seq, 5);
    }
    activeSeq = 0;
  }
}

void updateHighLevelTargets(float dt) {
  clearTargets();
  debugYawPhase = 0;
  debugErrSettled = false;
  debugRateSettled = false;
  debugRawHeadingErrDeg = 0.0f;
  debugUsedHeadingErrDeg = 0.0f;

  if (!controllerEnabled || commandMode != CMD_SEQUENCE) {
    return;
  }

  updateBodyCounts();
  if (movePhase == MOVE_TURNING) {
    yawHoldInPlaceMode = true;
    holdHeadingAtStop = true;
  }

  if (yawHoldInPlaceMode) {
    positionCurrentCounts = 0.0f;
    positionErrorCounts = 0.0f;
    positionDone = true;
    positionDoneTicks = 5;
    positionCmdRpm = 0.0f;
    posCmdPrevRpm = 0.0f;
    posPid.out = 0.0f;
    resetCascadePid(posPid);
  } else {
    positionCurrentCounts = computeMoveProgressCounts();
    positionErrorCounts = activeMoveTargetCounts - positionCurrentCounts;

    float positionErrorCm = countsToCm(positionErrorCounts);
    float absPositionErrCm = fabsf(positionErrorCm);
    float avgAbsWheelRpm = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
      avgAbsWheelRpm += fabsf(measuredRpm[i]);
    }
    avgAbsWheelRpm *= 0.25f;

    float doneTolCm = constrain(posDoneTolCm, 0.2f, max(2.0f, positionTolCm));
    bool nearPosTol = absPositionErrCm <= min(positionTolCm, doneTolCm);
    bool wheelSettled = avgAbsWheelRpm <= max(0.4f, posDoneMaxWheelRpm);
    if (nearPosTol && wheelSettled) {
      if (positionDoneTicks < 255) positionDoneTicks++;
    } else {
      positionDoneTicks = 0;
    }
    positionDone = (positionDoneTicks >= 8);

    float iBandCm = max(positionTolCm + 0.2f, posIntegralBandCm);
    if (!positionDone && absPositionErrCm <= iBandCm) {
      posPid.integral += positionErrorCounts * dt;
      posPid.integral = constrain(posPid.integral, -30000.0f, 30000.0f);
    } else {
      posPid.integral *= 0.85f;
      if (absPositionErrCm > (iBandCm + 5.0f)) {
        posPid.integral = 0.0f;
      }
    }

    float posDeriv = (positionErrorCounts - posPid.prevError) / dt;
    posPid.prevError = positionErrorCounts;

    float base = posPid.kp * positionErrorCounts + posPid.ki * posPid.integral + posPid.kd * posDeriv;

    float cruiseRpm = constrain(activeMoveCruiseRpm, 0.0f, TARGET_RPM_MAX);
    float finalMinRpm = constrain(posFinalMinRpm, 0.0f, cruiseRpm);
    float finalWinCm = max(positionTolCm + 0.1f, posFinalWindowCm);
    float slowStartCm = max(finalWinCm + 0.2f, posSlowdownStartCm);
    float profileMaxRpm = cruiseRpm;
    if (absPositionErrCm <= slowStartCm) {
      float t = (absPositionErrCm - finalWinCm) / max(0.2f, (slowStartCm - finalWinCm));
      t = constrain(t, 0.0f, 1.0f);
      profileMaxRpm = finalMinRpm + t * (cruiseRpm - finalMinRpm);
    }
    profileMaxRpm = constrain(profileMaxRpm, finalMinRpm, cruiseRpm);
    base = constrain(base, -profileMaxRpm, profileMaxRpm);

    if (!positionDone && absPositionErrCm > (positionTolCm + 0.3f) && fabsf(base) < finalMinRpm) {
      base = (positionErrorCm >= 0.0f ? 1.0f : -1.0f) * finalMinRpm;
    }

    float noReverseBand = max(0.25f, min(posNoReverseBandCm, max(0.6f, positionTolCm * 0.9f)));
    if (absPositionErrCm <= noReverseBand && (base * positionErrorCm) < 0.0f) {
      base = 0.0f;
      posPid.integral = 0.0f;
    }

    if (positionDone) {
      base = 0.0f;
    }

    float posSlew = max(0.0f, posCmdSlewRpmPerSec) * dt;
    float cmd = posCmdPrevRpm + constrain(base - posCmdPrevRpm, -posSlew, posSlew);
    if (positionDone) {
      cmd = 0.0f;
    }

    posCmdPrevRpm = cmd;
    posPid.out = cmd;
    positionCmdRpm = cmd;
  }

  float corr = 0.0f;
  headingErrorDeg = 0.0f;
  if (imuOk) {
    float rawHeadingError = shortestAngleErrorDeg(headingTargetDeg, imuYawDeg);
    debugRawHeadingErrDeg = rawHeadingError;
    bool movingPhase = !positionDone;
    bool holdPhase = positionDone && holdHeadingAtStop;
    float corrTarget = 0.0f;

    if (movingPhase) {
      debugYawPhase = 1;
      float errMove = rawHeadingError;
      float moveDb = constrain(headingMoveDeadbandDeg, 0.0f, 10.0f);
      if (fabsf(errMove) <= moveDb) {
        errMove = 0.0f;
      } else if (errMove > 0.0f) {
        errMove -= moveDb;
      } else {
        errMove += moveDb;
      }
      debugUsedHeadingErrDeg = errMove;

      float uMove = headingPid.kp * errMove - headingPid.kd * imuGyroDpsFilt;
      float moveKiBand = max(moveDb + 0.2f, headingMoveKiErrBandDeg);
      if (fabsf(headingPid.ki) > 0.0001f && fabsf(errMove) < moveKiBand) {
        headingPid.integral += errMove * dt;
        headingPid.integral = constrain(headingPid.integral, -40.0f, 40.0f);
        uMove += headingPid.ki * headingPid.integral;
      } else {
        headingPid.integral = 0.0f;
      }

      uMove *= (float)headingCorrSign;
      corrTarget = constrain(uMove, -headingCorrMaxRpm, headingCorrMaxRpm);
      headingPid.prevError = errMove;
      headingPid.out = corrTarget;
      headingErrorDeg = errMove;

      resetCascadePid(headingHoldPid);
      headingHoldActive = false;
      headingHoldStableMs = 0.0f;
    } else if (holdPhase) {
      float errAbs = fabsf(rawHeadingError);
      float rateAbs = fabsf(imuGyroDpsFilt);
      float errEnter = max(headingHoldDeadbandDeg, 0.05f);
      float rateEnter = max(headingHoldExitRateDps, 0.05f);
      float errExit = errEnter + 0.6f;
      float rateExit = rateEnter + 0.8f;

      if (!yawHoldInPlaceMode) {
        errEnter = min(errEnter, 0.55f);
        errExit = errEnter + 0.25f;
        rateEnter = min(rateEnter, 1.8f);
        rateExit = rateEnter + 0.5f;
      }

      if (headingErrSettledLatch) {
        headingErrSettledLatch = errAbs <= errExit;
      } else {
        headingErrSettledLatch = errAbs <= errEnter;
      }

      if (headingRateSettledLatch) {
        headingRateSettledLatch = rateAbs <= rateExit;
      } else {
        headingRateSettledLatch = rateAbs <= rateEnter;
      }

      bool errSettled = headingErrSettledLatch;
      bool rateSettled = headingRateSettledLatch;
      debugErrSettled = errSettled;
      debugRateSettled = rateSettled;

      if (errSettled && rateSettled) {
        debugYawPhase = 3;
        headingHoldStableMs += dt * 1000.0f;
        if (headingHoldStableMs >= headingHoldSettleHoldMs) {
          corrTarget = 0.0f;
          debugUsedHeadingErrDeg = 0.0f;
          headingHoldActive = false;
          resetCascadePid(headingHoldPid);
        }
      } else {
        debugYawPhase = 2;
        headingHoldStableMs = 0.0f;
        headingHoldActive = true;

        float errHold = rawHeadingError;
        if (fabsf(errHold) <= headingHoldDeadbandDeg) {
          errHold = 0.0f;
        } else if (errHold > 0.0f) {
          errHold -= headingHoldDeadbandDeg;
        } else {
          errHold += headingHoldDeadbandDeg;
        }
        debugUsedHeadingErrDeg = errHold;

        float uHold = headingHoldPid.kp * errHold - headingHoldPid.kd * imuGyroDpsFilt;
        float holdKiEff = headingHoldPid.ki;
        float holdKiErrBand = 6.0f;
        if (yawHoldInPlaceMode) {
          holdKiEff *= yawHoldInPlaceKiScale;
          holdKiErrBand = max(4.0f, yawHoldInPlaceKiErrDeg);
        }
        if (fabsf(holdKiEff) > 0.0001f && fabsf(errHold) < holdKiErrBand) {
          headingHoldPid.integral += errHold * dt;
          float iLim = yawHoldInPlaceMode ? 24.0f : 20.0f;
          headingHoldPid.integral = constrain(headingHoldPid.integral, -iLim, iLim);
          uHold += holdKiEff * headingHoldPid.integral;
        } else {
          headingHoldPid.integral = 0.0f;
        }

        uHold *= (float)headingCorrSign;

        float holdMaxEff = yawHoldInPlaceMode ? activeTurnMaxRpm : min(headingHoldMaxRpm, 3.2f);
        corrTarget = constrain(uHold, -holdMaxEff, holdMaxEff);

        if (yawHoldInPlaceMode) {
          float absRawErr = fabsf(rawHeadingError);
          float db = max(headingHoldDeadbandDeg, 0.1f);
          float slowStart = max(yawHoldSlowdownStartDeg, db + 0.5f);
          float finalCapDeg = constrain(yawHoldFinalCaptureDeg, db + 0.3f, slowStart - 0.1f);
          float minCmd = 0.0f;
          float tunedCruiseRpm = min(yawHoldCruiseRpm, activeTurnMaxRpm);

          if (absRawErr > db) {
            if (absRawErr >= slowStart) {
              minCmd = tunedCruiseRpm;
            } else if (absRawErr >= finalCapDeg) {
              float t = (absRawErr - finalCapDeg) / (slowStart - finalCapDeg);
              t = constrain(t, 0.0f, 1.0f);
              minCmd = yawHoldFinalCaptureRpm + t * (tunedCruiseRpm - yawHoldFinalCaptureRpm);
            } else {
              float t = (absRawErr - db) / (finalCapDeg - db);
              t = constrain(t, 0.0f, 1.0f);
              minCmd = yawHoldFinalMinRpm + t * (yawHoldFinalCaptureRpm - yawHoldFinalMinRpm);
            }
          }

          if (absRawErr <= finalCapDeg) {
            holdMaxEff = min(holdMaxEff, max(yawHoldFinalCaptureRpm, yawHoldFinalMinRpm + 0.6f));
          } else if (absRawErr <= (finalCapDeg + 4.0f)) {
            holdMaxEff = min(holdMaxEff, max(yawHoldFinalCaptureRpm + 0.8f, 4.5f));
          }
          corrTarget = constrain(corrTarget, -holdMaxEff, holdMaxEff);

          if (absRawErr > (db + 1.0f)) {
            int desiredSign = 0;
            float signedErr = rawHeadingError * ((float)headingCorrSign);
            if (signedErr > 0.0f) desiredSign = 1;
            else if (signedErr < 0.0f) desiredSign = -1;
            if (desiredSign != 0 && (corrTarget * ((float)desiredSign)) < 0.0f) {
              corrTarget = 0.0f;
            }
          }

          minCmd = constrain(minCmd, 0.0f, holdMaxEff);
          bool enforceMin = absRawErr > (db + 0.3f) && headingHoldStableMs < headingHoldSettleHoldMs;
          if (enforceMin && minCmd > 0.0f && fabsf(corrTarget) < minCmd) {
            int sign = 0;
            if (corrTarget > 0.0f) sign = 1;
            else if (corrTarget < 0.0f) sign = -1;
            else sign = (rawHeadingError >= 0.0f) ? 1 : -1;
            corrTarget = ((float)sign) * minCmd;
          }
        }

        headingHoldPid.prevError = errHold;
        headingHoldPid.out = corrTarget;
      }

      headingErrorDeg = rawHeadingError;
      resetCascadePid(headingPid);
    } else {
      resetCascadePid(headingPid);
      resetCascadePid(headingHoldPid);
      headingHoldActive = false;
      headingHoldStableMs = 0.0f;
      corrTarget = 0.0f;
    }

    float slew = max(0.0f, headingCorrSlewRpmPerSec) * dt;
    corr = headingCorrCmdPrev + constrain(corrTarget - headingCorrCmdPrev, -slew, slew);
    if (holdPhase && !headingHoldActive) {
      corr = 0.0f;
    }
    headingCorrCmdPrev = corr;
  }
  headingCorrRpm = corr;

  if (movePhase == MOVE_TURNING) {
    float absHeadingErr = fabsf(headingErrorDeg);
    float absGyro = fabsf(imuGyroDpsFilt);
    bool nearHeadingAndSlow = (absHeadingErr <= 3.0f) && (absGyro <= 3.0f);
    bool closeHeadingBand = (absHeadingErr <= 5.0f);
    if (nearHeadingAndSlow) {
      activeMoveTurnNearMs += dt * 1000.0f;
    } else {
      activeMoveTurnNearMs = 0.0f;
    }
    if (closeHeadingBand) {
      activeMoveTurnCloseMs += dt * 1000.0f;
    } else {
      activeMoveTurnCloseMs = 0.0f;
    }

    bool strictSettled = debugErrSettled && debugRateSettled && (headingHoldStableMs >= headingHoldSettleHoldMs);
    bool nearSlowSettled = (activeMoveTurnNearMs >= 300.0f);
    bool closeBandSettled = (activeMoveTurnCloseMs >= 1200.0f);
    bool headingSettledNow = activeMoveTurnOnly ? (strictSettled || (nearSlowSettled && (absHeadingErr <= 2.2f))) : (strictSettled || nearSlowSettled || closeBandSettled);

    if (headingSettledNow) {
      if (activeMoveTurnOnly) {
        finishSequence(1);
        return;
      }

      movePhase = MOVE_DRIVING;
      activeMovePhaseStartMs = millis();
      activeMoveTurnNearMs = 0.0f;
      activeMoveTurnCloseMs = 0.0f;
      yawHoldInPlaceMode = false;
      headingCorrCmdPrev = 0.0f;
      resetCascadePid(headingHoldPid);
      positionDone = false;
      positionDoneTicks = 0;
      positionCmdRpm = 0.0f;
      posCmdPrevRpm = 0.0f;
      resetCascadePid(posPid);
    }

    if (movePhase == MOVE_TURNING && activeMovePhaseStartMs != 0) {
      uint32_t elapsed = millis() - activeMovePhaseStartMs;
      if (elapsed > activeMoveTurnTimeoutMs) {
        finishSequence(3);
        return;
      }
    }
  } else if (movePhase == MOVE_DRIVING) {
    if (positionDone) {
      finishSequence(1);
      return;
    }
  }

  targetRpm[0] = constrain(positionCmdRpm * activeMoveMix[0] - headingCorrRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[1] = constrain(positionCmdRpm * activeMoveMix[1] - headingCorrRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[2] = constrain(positionCmdRpm * activeMoveMix[2] + headingCorrRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
  targetRpm[3] = constrain(positionCmdRpm * activeMoveMix[3] + headingCorrRpm, -TARGET_RPM_MAX, TARGET_RPM_MAX);
}

void runWheelPid(float dt) {
  bool yawHoldWheelMode = controllerEnabled && (fabsf(positionCmdRpm) < headingEnableBaseRpm) && (headingHoldActive || yawHoldInPlaceMode);

  for (uint8_t i = 0; i < 4; i++) {
    float target = targetRpm[i];
    if (yawHoldWheelMode && fabsf(target) < yawHoldTargetDeadbandRpm) {
      target = 0.0f;
    }

    float err = target - measuredRpm[i];
    if (fabsf(target) < 0.01f) {
      resetPidState(i);
      yawHoldWheelCmdPrev[i] = 0;
      setMotorPwmSigned(i, 0);
      continue;
    }

    pid[i].integral += err * dt;
    pid[i].integral = constrain(pid[i].integral, -300.0f, 300.0f);

    float deriv = (err - pid[i].prevError) / dt;
    pid[i].prevError = err;

    float kpEff = pid[i].kp;
    float kiEff = pid[i].ki;
    float kdEff = pid[i].kd;
    if (yawHoldWheelMode) {
      kpEff *= yawHoldWheelKpScale;
      kiEff = 0.0f;
      kdEff *= yawHoldWheelKdScale;
      pid[i].integral = 0.0f;
    }

    float u = kpEff * err + kiEff * pid[i].integral + kdEff * deriv;
    pid[i].out = u;

    int cmd = (int)lroundf(u);
    int maxPwm = yawHoldWheelMode ? constrain((int)yawHoldWheelPwmMax, 0, PWM_MAX) : PWM_MAX;
    if (yawHoldWheelMode && yawHoldInPlaceMode) {
      float absErr = fabsf(debugRawHeadingErrDeg);
      if (absErr < 6.0f) {
        maxPwm = min(maxPwm, 90);
      } else if (absErr < 12.0f) {
        maxPwm = min(maxPwm, 130);
      }
    }
    cmd = constrain(cmd, -maxPwm, maxPwm);

    bool satHigh = (cmd >= maxPwm - 1) && (err > 0.0f);
    bool satLow = (cmd <= -maxPwm + 1) && (err < 0.0f);
    if (kiEff > 0.0f && (satHigh || satLow)) {
      pid[i].integral -= err * dt;
      pid[i].integral = constrain(pid[i].integral, -300.0f, 300.0f);
    }

    bool translationalMotion = !(controllerEnabled && fabsf(positionCmdRpm) < headingEnableBaseRpm);
    bool wantsMotion = fabsf(target) > 0.8f;
    bool wheelNearlyStopped = fabsf(measuredRpm[i]) < 1.0f;
    int breakawayPwm = translationalMotion ? (int)minPwm : 0;
    if (wantsMotion && wheelNearlyStopped && breakawayPwm > 0 && abs(cmd) < breakawayPwm) {
      int sign = 0;
      if (cmd > 0) sign = 1;
      else if (cmd < 0) sign = -1;
      else sign = (err >= 0.0f) ? 1 : -1;
      cmd = sign * breakawayPwm;
    }

    if (yawHoldWheelMode && yawHoldInPlaceMode) {
      bool farFromTarget = fabsf(debugRawHeadingErrDeg) >= yawHoldBreakawayErrDeg;
      bool stalled = fabsf(measuredRpm[i]) < 0.8f;
      bool nonSettled = !debugErrSettled;
      bool withinSettleDwell = headingHoldStableMs < headingHoldSettleHoldMs;
      if (withinSettleDwell && farFromTarget && stalled && nonSettled && abs(cmd) < yawHoldBreakawayPwm) {
        int sign = 0;
        if (cmd > 0) sign = 1;
        else if (cmd < 0) sign = -1;
        else sign = (target >= 0.0f) ? 1 : -1;
        cmd = sign * (int)yawHoldBreakawayPwm;
      }
    }

    if (yawHoldWheelMode && yawHoldInPlaceMode) {
      int slewStep = max(4, (int)lroundf(max(0.0f, yawHoldWheelPwmSlewPerSec) * dt));
      int prev = yawHoldWheelCmdPrev[i];
      cmd = constrain(cmd, prev - slewStep, prev + slewStep);
      yawHoldWheelCmdPrev[i] = (int16_t)cmd;
    } else {
      yawHoldWheelCmdPrev[i] = (int16_t)cmd;
    }

    setMotorPwmSigned(i, cmd);
  }
}

bool extractArg(const String& line, const char* key, String& valueOut) {
  String pattern = String(key) + "=";
  int start = line.indexOf(pattern);
  if (start < 0) {
    return false;
  }
  start += pattern.length();
  int end = line.indexOf(' ', start);
  if (end < 0) {
    end = line.length();
  }
  valueOut = line.substring(start, end);
  return true;
}

float getFloatArg(const String& line, const char* key, float defaultValue) {
  String value;
  if (!extractArg(line, key, value)) {
    return defaultValue;
  }
  return value.toFloat();
}

uint32_t getUIntArg(const String& line, const char* key, uint32_t defaultValue) {
  String value;
  if (!extractArg(line, key, value)) {
    return defaultValue;
  }
  return (uint32_t)value.toInt();
}

String firstToken(const String& line) {
  int sp = line.indexOf(' ');
  if (sp < 0) {
    return line;
  }
  return line.substring(0, sp);
}

void handleCommandLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String cmd = firstToken(line);
  cmd.toUpperCase();
  uint32_t seq = getUIntArg(line, "seq", 0);

  if (cmd == "PING") {
    sendJsonLine("{\"type\":\"pong\",\"seq\":" + String(seq) + ",\"ok\":true}");
    return;
  }

  if (cmd == "STATUS") {
    sendJsonLine(buildAckJson(seq, "STATUS", true, "telemetry_follows"));
    STATE_LOCK();
    String s = buildTelemetryJson();
    STATE_UNLOCK();
    sendJsonLine(s);
    return;
  }

  if (cmd == "STOP") {
    STATE_LOCK();
    cancelActiveCommand(2);
    STATE_UNLOCK();
    sendJsonLine(buildAckJson(seq, "STOP", true, "stopped"));
    return;
  }

  if (cmd == "ZERO_IMU") {
    STATE_LOCK();
    imuZeroYaw();
    headingTargetDeg = 0.0f;
    resetHighLevelControllers();
    STATE_UNLOCK();
    sendJsonLine(buildAckJson(seq, "ZERO_IMU", true, "yaw_zeroed"));
    return;
  }

  if (cmd == "CAL_IMU") {
    bool ok = false;
    STATE_LOCK();
    cancelActiveCommand(2);
    ok = imuCalibrateBias(IMU_BIAS_CAL_SAMPLES);
    resetHighLevelControllers();
    STATE_UNLOCK();
    sendJsonLine(buildAckJson(seq, "CAL_IMU", ok, ok ? "imu_bias_calibrated" : "imu_calibration_failed"));
    return;
  }

  if (cmd == "INIT_IMU") {
    bool ok = false;
    STATE_LOCK();
    cancelActiveCommand(2);
    ok = imuCalibrateBias(IMU_BIAS_CAL_SAMPLES);
    if (ok) {
      imuZeroYaw();
      headingTargetDeg = 0.0f;
    }
    resetHighLevelControllers();
    STATE_UNLOCK();
    sendJsonLine(buildAckJson(seq, "INIT_IMU", ok, ok ? "imu_calibrated_and_zeroed" : "imu_calibration_failed"));
    return;
  }

  if (cmd == "RESET_ENC") {
    STATE_LOCK();
    resetEncoders();
    STATE_UNLOCK();
    sendJsonLine(buildAckJson(seq, "RESET_ENC", true, "encoders_reset"));
    return;
  }

  if (cmd == "MOVE") {
    float angleDeg = getFloatArg(line, "angle", 0.0f);
    float distCm = getFloatArg(line, "dist", 0.0f);
    float speedRpm = getFloatArg(line, "speed", posCruiseRpm);
    float headingDeg = getFloatArg(line, "heading", imuYawDeg);
    uint32_t timeoutMs = getUIntArg(line, "timeout", 0);

    STATE_LOCK();
    if (!imuOk) {
      STATE_UNLOCK();
      sendJsonLine(buildAckJson(seq, "MOVE", false, "imu_missing"));
      return;
    }
    startMoveSequence(seq, angleDeg, distCm, speedRpm, headingDeg, timeoutMs);
    STATE_UNLOCK();
    sendJsonLine(buildAckJson(seq, "MOVE", true, "accepted"));
    return;
  }

  if (cmd == "TURN") {
    float headingDeg = getFloatArg(line, "heading", imuYawDeg);
    float speedRpm = getFloatArg(line, "speed", headingHoldMaxRpm);
    uint32_t timeoutMs = getUIntArg(line, "timeout", 0);

    STATE_LOCK();
    if (!imuOk) {
      STATE_UNLOCK();
      sendJsonLine(buildAckJson(seq, "TURN", false, "imu_missing"));
      return;
    }
    startMoveSequence(seq, 0.0f, 0.0f, speedRpm, headingDeg, timeoutMs);
    STATE_UNLOCK();
    sendJsonLine(buildAckJson(seq, "TURN", true, "accepted"));
    return;
  }

  if (cmd == "TWIST") {
    float forwardRpm = getFloatArg(line, "forward", 0.0f);
    float strafeRpm = getFloatArg(line, "strafe", 0.0f);
    float turnRpm = getFloatArg(line, "turn", 0.0f);
    uint32_t timeoutMs = getUIntArg(line, "timeout", 1000);

    STATE_LOCK();
    startTwist(seq, forwardRpm, strafeRpm, turnRpm, timeoutMs);
    STATE_UNLOCK();
    sendJsonLine(buildAckJson(seq, "TWIST", true, "accepted"));
    return;
  }

  sendJsonLine(buildAckJson(seq, "UNKNOWN", false, "unknown_command"));
}

void controlTaskFn(void* arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastControlMs = millis();

  for (;;) {
    STATE_LOCK();
    uint32_t now = millis();
    float dt = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    if (dt < 0.001f) {
      dt = CONTROL_INTERVAL_MS / 1000.0f;
    }

    updateMeasuredRpm(dt);
    imuUpdate(dt);

    if (commandMode == CMD_TWIST) {
      updateTwistTargets();
    } else {
      updateHighLevelTargets(dt);
    }
    runWheelPid(dt);
    STATE_UNLOCK();

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
  }
}

void uartRxTaskFn(void* arg) {
  (void)arg;
  char lineBuf[UART_LINE_MAX];
  size_t idx = 0;

  for (;;) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        lineBuf[idx] = '\0';
        handleCommandLine(String(lineBuf));
        idx = 0;
        continue;
      }
      if (idx + 1 < UART_LINE_MAX) {
        lineBuf[idx++] = c;
      } else {
        idx = 0;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void telemetryTaskFn(void* arg) {
  (void)arg;
  uint32_t lastTelemetryMs = 0;

  for (;;) {
    uint32_t doneSeq = 0;
    uint8_t doneCode = 0;
    bool sendDone = false;
    String telemetry;

    STATE_LOCK();
    uint32_t now = millis();
    if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
      updateBodyCounts();
      telemetry = buildTelemetryJson();
      lastTelemetryMs = now;
    }
    if (pendingDoneEvent) {
      sendDone = true;
      doneSeq = pendingDoneSeq;
      doneCode = pendingDoneCode;
      pendingDoneEvent = false;
    }
    STATE_UNLOCK();

    if (telemetry.length() > 0) {
      sendJsonLine(telemetry);
    }
    if (sendDone) {
      sendJsonLine(buildDoneJson(doneSeq, doneCode));
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  stateMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  Serial.begin(UART_BAUD);
  Serial.setTimeout(5);
  delay(300);

  imuInit();

  for (uint8_t i = 0; i < 4; i++) {
    setupEncoderPin(encoders[i].a);
    setupEncoderPin(encoders[i].b);
    encoderPrevState[i] = (digitalRead(encoders[i].a) << 1) | digitalRead(encoders[i].b);
  }

  attachInterrupt(digitalPinToInterrupt(encoders[0].a), isrEnc0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[0].b), isrEnc0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[1].a), isrEnc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[1].b), isrEnc1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[2].a), isrEnc2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[2].b), isrEnc2B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[3].a), isrEnc3A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoders[3].b), isrEnc3B, CHANGE);

  for (uint8_t i = 0; i < 4; i++) {
    pwmAttachPinCompat(motors[i].in1, motors[i].ch1);
    pwmAttachPinCompat(motors[i].in2, motors[i].ch2);
  }

  stopAllMotors();
  resetEncoders();
  resetHighLevelControllers();

  xTaskCreatePinnedToCore(controlTaskFn, "control", 8192, nullptr, 5, &controlTaskHandle, 1);
  xTaskCreatePinnedToCore(uartRxTaskFn, "uart_rx", 6144, nullptr, 3, &uartRxTaskHandle, 0);
  xTaskCreatePinnedToCore(telemetryTaskFn, "telemetry", 6144, nullptr, 2, &telemetryTaskHandle, 0);

  sendJsonLine("{\"type\":\"ready\",\"baud\":115200,\"imu\":" + String(imuOk ? "true" : "false") + "}");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
