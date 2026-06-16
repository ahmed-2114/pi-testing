// Clean mecanum cascaded position controller for the ESP32 Mecanum base.
//
// Clean control rebuild plan:
//   1) Tune one wheel velocity PI clearly.
//   2) Run all four wheel velocity PI loops together.
//   3) Add mecanum body velocity mixing: forward, strafe, rotate.
//   4) Add encoder odometry.
//   5) Add outer position and heading control as cascaded loops.
//
// Research references used for this rebuild:
//   - https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/tje2.70006
//   - https://docs.simplefoc.com/angle_cascade_control
//   - https://forum.arduino.cc/t/pid-control-loops/599383
//   - https://robotics.stackexchange.com/questions/90020/how-to-fuse-encoder-ticks-imu-for-odometry
//   - https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/mecanum-drive-odometry.html
//   - https://docs.ros.org/en/rolling/p/robot_localization/configuring_robot_localization.html
//
// Stage 5 of the clean control rebuild:
//   - body-frame forward / strafe / rotate commands
//   - mecanum inverse kinematics -> four wheel RPM targets
//   - all four wheel velocity loops run at the same time
//   - encoder odometry estimates body-frame forward and strafe displacement
//   - outer body-position P loops generate velocity commands
//   - heading uses position error plus gyro-rate damping for clean settling
//   - per-wheel tuned inner velocity PI loops
//   - wheel target scaling preserves direction while respecting RPM limit
//   - PWM saturation with conditional-integration anti-windup
//
// Pi bridge note:
//   This sketch keeps the exact new_pid controller and web GUI. The UART JSON
//   bridge commands are transported by micro-ROS topics instead of raw UART so
//   micro_ros_agent can own the ESP serial link.
//
// This sketch intentionally has no minPWM, feedforward, derivative term,
// slip detector, full EKF, or field-frame pose correction yet.
// Heading control uses IMU yaw. Encoder yaw is kept only as a diagnostic/fallback
// because mecanum wheel slip makes encoder-derived yaw unreliable during turns.

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>

struct EncoderPins {
  uint8_t a;
  uint8_t b;
  const char* name;
};

struct MotorPins {
  uint8_t in1;
  uint8_t in2;
  const char* name;
  uint8_t encoderIndex;
  uint8_t ch1;
  uint8_t ch2;
};

struct WheelPiState {
  float integralError;
  float errorRpm;
  float pTerm;
  float iTerm;
  float rawOutput;
  float output;
};

struct BodyPiState {
  float integralError;
  float error;
  float pTerm;
  float iTerm;
  float rawOutputRpm;
  float outputRpm;
};

struct TelemetrySnapshot {
  bool enabled;
  bool timedRunActive;
  bool positionModeActive;
  bool positionTargetReached;
  uint32_t positionMoveId;
  float kp[4];
  float ki[4];
  float positionKp;
  float positionKi;
  float yawKp;
  float yawKi;
  float positionMaxRpm;
  float positionToleranceCm;
  float yawToleranceDeg;
  bool imuOk;
  bool headingUsesImu;
  float imuYawDeg;
  float imuYawRawDeg;
  float encoderYawDeg;
  float imuGyroDps;
  float imuGyroDpsFilt;
  float imuBiasDps;
  float imuCalStdDps;
  bool imuCalValid;
  float loopDtMs;
  uint32_t nowMs;
  uint32_t timedRunId;
  uint32_t timedRunDurationMs;
  uint32_t timedRunElapsedMs;
  float commandForwardRpm;
  float commandStrafeRpm;
  float commandRotateRpm;
  float targetScale;
  int8_t strafeSign;
  int8_t rotateSign;
  int8_t odomForwardSign;
  int8_t odomStrafeSign;
  int8_t odomYawSign;
  float odomForwardScale;
  float odomStrafeScale;
  float odomForwardCm;
  float odomStrafeCm;
  float odomYawDeg;
  float targetForwardCm;
  float targetStrafeCm;
  float targetYawDeg;
  float positionErrorForwardCm;
  float positionErrorStrafeCm;
  float positionErrorYawDeg;
  float positionPTerm[3];
  float positionITerm[3];
  float positionOutputRpm[3];
  float odomWheelCm[4];
  float rawTargetRpm[4];
  float targetRpm[4];
  float appliedTargetRpm[4];
  float rawRpm[4];
  float measuredRpm[4];
  float errorRpm[4];
  float pTerm[4];
  float iTerm[4];
  float rawOutput[4];
  float output[4];
  float integralError[4];
  int16_t pwm[4];
  int16_t hardwarePwm[4];
  int8_t encoderSign[4];
  int8_t motorSign[4];
  int32_t rawCounts[4];
  int32_t signedCounts[4];
};

const char* WHEEL_NAMES[4] = {"BR", "FR", "BL", "FL"};

// Logical motor order is fixed here as BR, FR, BL, FL.
// Calibrated mapping from the clean single-wheel tests:
//   BR -> MOTA / ENC1
//   FR -> MOTB / ENC2
//   BL -> MOTD / ENC4
//   FL -> MOTC / ENC3
EncoderPins encoders[4] = {
    {36, 39, "ENC1 BackRight"},
    {35, 34, "ENC2 FrontRight"},
    {33, 32, "ENC3 FrontLeft"},
    {25, 26, "ENC4 BackLeft"},
};

MotorPins motors[4] = {
    {14, 27, "MOTA BackRight", 0, 0, 1},
    {19, 13, "MOTB FrontRight", 1, 2, 3},
    {18, 17, "MOTD BackLeft", 3, 6, 7},
    {4, 16, "MOTC FrontLeft", 2, 4, 5},
};

// Calibrated order BR, FR, BL, FL.
int8_t encoderSign[4] = {-1, 1, -1, 1};
int8_t motorSign[4] = {1, 1, 1, 1};

volatile int32_t encoderCount[4] = {0, 0, 0, 0};
volatile uint8_t encoderPrevState[4] = {0, 0, 0, 0};

int32_t lastEncoderCount[4] = {0, 0, 0, 0};
int32_t lastOdomSignedCount[4] = {0, 0, 0, 0};
float rawRpm[4] = {0, 0, 0, 0};
float measuredRpm[4] = {0, 0, 0, 0};
float targetRpm[4] = {0, 0, 0, 0};
int16_t pwmCmd[4] = {0, 0, 0, 0};
int16_t hardwarePwmCmd[4] = {0, 0, 0, 0};
WheelPiState piState[4] = {};
BodyPiState positionPi[3] = {};

const char* AP_SSID = "ESP32-MECANUM-POS";
const char* AP_PASS = "position123";

const float COUNTS_PER_WHEEL_REV = 4346.8f;
const float WHEEL_DIAMETER_CM = 9.7f;
const float WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * PI;
const uint32_t CONTROL_INTERVAL_MS = 20;
const float NOMINAL_DT_S = CONTROL_INTERVAL_MS / 1000.0f;
const float RPM_TARGET_LIMIT = 45.0f;
const float RPM_FILTER_ALPHA = 0.20f;
const uint32_t UART_BAUD = 115200;
const size_t UART_LINE_MAX = 180;
const uint32_t PI_TELEMETRY_INTERVAL_MS = 100;
const uint8_t DEFAULT_LIMIT_STATUS_PIN = 23;
const uint32_t TIMED_RUN_MIN_MS = 250;
const uint32_t TIMED_RUN_MAX_MS = 120000;
const float CMD_VEL_LINEAR_MPS_TO_RPM = 60.0f / (WHEEL_CIRCUMFERENCE_CM / 100.0f);
const float CMD_VEL_ANGULAR_RADPS_TO_RPM = 22.0f;
const uint32_t CMD_VEL_TIMEOUT_MS = 300;
const uint32_t MICRO_ROS_SPIN_MS = 5;
const size_t MICRO_ROS_TELEMETRY_BUFFER = 640;
const uint32_t MICRO_ROS_RECONNECT_PERIOD_MS = 1000;
const uint32_t MICRO_ROS_AGENT_PING_TIMEOUT_MS = 100;
const uint8_t MICRO_ROS_AGENT_PING_ATTEMPTS = 1;

const uint16_t PWM_FREQ = 1000;
const uint8_t PWM_RES_BITS = 8;
const int PWM_MAX = 255;

const uint8_t MPU6050_ADDR = 0x68;
const float MPU6050_GYRO_Z_SCALE = 65.5f;  // LSB/(deg/s) at +-500 dps.
const float MPU6050_GYRO_FILTER_ALPHA = 0.20f;
const float MPU6050_CAL_MAX_STD_DPS = 0.35f;
const float ENCODER_YAW_CM_PER_DEG = 0.30f;
const float YAW_DAMP_RPM_PER_DPS = 0.20f;
const float YAW_CONTROL_DEADBAND_DEG = 2.0f;
const float YAW_SETTLE_RATE_DPS = 3.0f;
const float YAW_NEAR_TARGET_DEG = 15.0f;
const float YAW_NEAR_TARGET_MAX_RPM = 10.0f;
const uint8_t POSITION_SETTLE_TICKS_REQUIRED = 8;

float velocityKp[4] = {15.0f, 15.0f, 15.0f, 15.0f};
float velocityKi[4] = {200.0f, 200.0f, 200.0f, 200.0f};
float commandForwardRpm = 0.0f;
float commandStrafeRpm = 0.0f;
float commandRotateRpm = 0.0f;
float rawTargetRpm[4] = {0, 0, 0, 0};
float targetScale = 1.0f;
int8_t strafeSign = -1;
int8_t rotateSign = -1;
int8_t odomForwardSign = 1;
int8_t odomStrafeSign = -1;
int8_t odomYawSign = -1;
float odomForwardScale = 0.9854f;
float odomStrafeScale = 0.9375f;
float odomForwardCm = 0.0f;
float odomStrafeCm = 0.0f;
float odomYawWheelCm = 0.0f;
float odomWheelCm[4] = {0, 0, 0, 0};
bool positionModeActive = false;
bool positionTargetReached = false;
uint8_t positionSettleTicks = 0;
uint32_t positionMoveId = 0;
float targetForwardCm = 0.0f;
float targetStrafeCm = 0.0f;
float targetYawDeg = 0.0f;
float positionErrorForwardCm = 0.0f;
float positionErrorStrafeCm = 0.0f;
float positionErrorYawDeg = 0.0f;
float positionKp = 100.0f;
float positionKi = 0.0f;
float yawKp = 10.0f;
float yawKi = 0.0f;
float positionMaxRpm = 40.0f;
float positionToleranceCm = 2.0f;
float yawToleranceDeg = 2.0f;
bool imuOk = false;
float imuYawDeg = 0.0f;
float imuYawRawDeg = 0.0f;
float imuGyroDps = 0.0f;
float imuGyroDpsFilt = 0.0f;
float imuBiasDps = 0.0f;
float imuCalStdDps = 0.0f;
bool imuCalValid = false;
bool controllerEnabled = false;
float lastLoopDtMs = CONTROL_INTERVAL_MS;
bool timedRunActive = false;
uint32_t timedRunStartMs = 0;
uint32_t timedRunDurationMs = 3000;
uint32_t timedRunId = 0;

WebServer server(80);
SemaphoreHandle_t stateMutex = nullptr;
SemaphoreHandle_t serialMutex = nullptr;
TaskHandle_t controlTaskHandle = nullptr;
TaskHandle_t webTaskHandle = nullptr;
TaskHandle_t uartTaskHandle = nullptr;
TaskHandle_t piTelemetryTaskHandle = nullptr;
TaskHandle_t microRosTaskHandle = nullptr;

#define STATE_LOCK()   do { if (stateMutex) xSemaphoreTake(stateMutex, portMAX_DELAY); } while (0)
#define STATE_UNLOCK() do { if (stateMutex) xSemaphoreGive(stateMutex); } while (0)
#define SERIAL_LOCK()   do { if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY); } while (0)
#define SERIAL_UNLOCK() do { if (serialMutex) xSemaphoreGive(serialMutex); } while (0)

uint32_t piActiveSeq = 0;
bool piMoveActive = false;
float piMoveStartForwardCm = 0.0f;
float piMoveStartStrafeCm = 0.0f;
float piOdomForwardCm = 0.0f;
float piOdomStrafeCm = 0.0f;
float piLastMoveAngleDeg = 0.0f;
float piLastMoveDistanceCm = 0.0f;
bool piPendingDone = false;
uint32_t piPendingDoneSeq = 0;
uint8_t piPendingDoneCode = 0;

rcl_allocator_t microrosAllocator;
rclc_support_t microrosSupport;
rcl_node_t microrosNode = rcl_get_zero_initialized_node();
rclc_executor_t microrosExecutor = rclc_executor_get_zero_initialized_executor();
rcl_timer_t telemetryTimer = rcl_get_zero_initialized_timer();
rcl_subscription_t moveGoalSub = rcl_get_zero_initialized_subscription();
rcl_publisher_t telemetryJsonPub = rcl_get_zero_initialized_publisher();
std_msgs__msg__String moveGoalMsg;
std_msgs__msg__String telemetryJsonMsg;
char moveGoalBuffer[UART_LINE_MAX];
char telemetryJsonBuffer[MICRO_ROS_TELEMETRY_BUFFER];
bool microrosEntitiesCreated = false;
uint32_t microrosLastConnectAttemptMs = 0;

#define RCSOFTCHECK(fn) do { rcl_ret_t temp_rc = (fn); (void)temp_rc; } while (0)

float wrapAngleDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

float shortestAngleErrorDeg(float target, float measurement) {
  return wrapAngleDeg(target - measurement);
}

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

bool isInputOnlyPin(uint8_t pin) {
  return pin == 34 || pin == 35 || pin == 36 || pin == 39;
}

void setupEncoderPin(uint8_t pin) {
  pinMode(pin, isInputOnlyPin(pin) ? INPUT : INPUT_PULLUP);
}

void IRAM_ATTR updateEncoder(uint8_t idx) {
  uint8_t state = (digitalRead(encoders[idx].a) << 1) | digitalRead(encoders[idx].b);
  uint8_t prev = encoderPrevState[idx];
  uint8_t transition = (prev << 2) | state;

  switch (transition) {
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
  int32_t count = encoderCount[idx];
  interrupts();
  return count;
}

void setMotorPwmSigned(uint8_t wheel, int command) {
  command = constrain(command, -PWM_MAX, PWM_MAX);
  pwmCmd[wheel] = (int16_t)command;

  int hardwareCommand = constrain(command * (int)motorSign[wheel], -PWM_MAX, PWM_MAX);
  hardwarePwmCmd[wheel] = (int16_t)hardwareCommand;

  const MotorPins& motor = motors[wheel];
  if (hardwareCommand > 0) {
    pwmWriteCompat(motor.in1, motor.ch1, (uint32_t)hardwareCommand);
    pwmWriteCompat(motor.in2, motor.ch2, 0);
  } else if (hardwareCommand < 0) {
    pwmWriteCompat(motor.in1, motor.ch1, 0);
    pwmWriteCompat(motor.in2, motor.ch2, (uint32_t)(-hardwareCommand));
  } else {
    pwmWriteCompat(motor.in1, motor.ch1, 0);
    pwmWriteCompat(motor.in2, motor.ch2, 0);
  }
}

void stopAllMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    setMotorPwmSigned(i, 0);
  }
}

void resetPiState(uint8_t wheel) {
  piState[wheel].integralError = 0.0f;
  piState[wheel].errorRpm = 0.0f;
  piState[wheel].pTerm = 0.0f;
  piState[wheel].iTerm = 0.0f;
  piState[wheel].rawOutput = 0.0f;
  piState[wheel].output = 0.0f;
}

void resetAllPiStates() {
  for (uint8_t i = 0; i < 4; i++) {
    resetPiState(i);
  }
}

void resetBodyPiState(uint8_t axis) {
  positionPi[axis].integralError = 0.0f;
  positionPi[axis].error = 0.0f;
  positionPi[axis].pTerm = 0.0f;
  positionPi[axis].iTerm = 0.0f;
  positionPi[axis].rawOutputRpm = 0.0f;
  positionPi[axis].outputRpm = 0.0f;
}

void resetAllBodyPiStates() {
  for (uint8_t axis = 0; axis < 3; axis++) {
    resetBodyPiState(axis);
  }
  positionSettleTicks = 0;
}

void resetEncoderMeasurements() {
  noInterrupts();
  for (uint8_t i = 0; i < 4; i++) {
    encoderCount[i] = 0;
  }
  interrupts();

  for (uint8_t i = 0; i < 4; i++) {
    lastEncoderCount[i] = 0;
    rawRpm[i] = 0.0f;
    measuredRpm[i] = 0.0f;
  }
}

void resetRpmMeasurementsToCurrentCounts() {
  for (uint8_t encoder = 0; encoder < 4; encoder++) {
    lastEncoderCount[encoder] = readEncoderCount(encoder);
  }

  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    rawRpm[wheel] = 0.0f;
    measuredRpm[wheel] = 0.0f;
  }
}

void resetOdometryToCurrentCounts() {
  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    int32_t rawCount = readEncoderCount(motors[wheel].encoderIndex);
    lastOdomSignedCount[wheel] = ((int32_t)encoderSign[wheel]) * rawCount;
    odomWheelCm[wheel] = 0.0f;
  }

  odomForwardCm = 0.0f;
  odomStrafeCm = 0.0f;
  odomYawWheelCm = 0.0f;
}

void updateMeasuredRpm(float dt) {
  if (dt <= 0.0001f) {
    return;
  }

  const float rpmScale = (60.0f / COUNTS_PER_WHEEL_REV) / dt;
  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    uint8_t encoderIndex = motors[wheel].encoderIndex;
    int32_t countNow = readEncoderCount(encoderIndex);
    int32_t delta = countNow - lastEncoderCount[encoderIndex];
    lastEncoderCount[encoderIndex] = countNow;

    rawRpm[wheel] = ((float)encoderSign[wheel]) * ((float)delta) * rpmScale;
    measuredRpm[wheel] += RPM_FILTER_ALPHA * (rawRpm[wheel] - measuredRpm[wheel]);
  }
}

void updateOdometry() {
  float deltaWheelCm[4] = {0, 0, 0, 0};

  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    int32_t rawCount = readEncoderCount(motors[wheel].encoderIndex);
    int32_t signedCount = ((int32_t)encoderSign[wheel]) * rawCount;
    int32_t deltaCount = signedCount - lastOdomSignedCount[wheel];
    lastOdomSignedCount[wheel] = signedCount;

    deltaWheelCm[wheel] = ((float)deltaCount / COUNTS_PER_WHEEL_REV) * WHEEL_CIRCUMFERENCE_CM;
    odomWheelCm[wheel] += deltaWheelCm[wheel];
  }

  // Logical order is BR, FR, BL, FL. This is body-frame dead reckoning.
  odomForwardCm += (deltaWheelCm[0] + deltaWheelCm[1] + deltaWheelCm[2] + deltaWheelCm[3]) * 0.25f;
  odomStrafeCm += (-deltaWheelCm[0] + deltaWheelCm[1] + deltaWheelCm[2] - deltaWheelCm[3]) * 0.25f;
  odomYawWheelCm += (-deltaWheelCm[0] - deltaWheelCm[1] + deltaWheelCm[2] + deltaWheelCm[3]) * 0.25f;
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
  uint8_t data[2] = {0, 0};
  if (!imuReadRegs(0x47, data, 2)) {
    return false;
  }
  gzRaw = (int16_t)((data[0] << 8) | data[1]);
  return true;
}

bool imuCalibrateBias(uint16_t samples) {
  if (!imuOk || samples < 20) {
    return false;
  }

  float sum = 0.0f;
  float sumSq = 0.0f;
  uint16_t okCount = 0;
  for (uint16_t i = 0; i < samples; i++) {
    int16_t gzRaw = 0;
    if (imuReadGyroZRaw(gzRaw)) {
      float dps = ((float)gzRaw) / MPU6050_GYRO_Z_SCALE;
      sum += dps;
      sumSq += dps * dps;
      okCount++;
    }
    delay(2);
  }

  if (okCount < samples / 2) {
    imuCalValid = false;
    return false;
  }

  float mean = sum / (float)okCount;
  float variance = (sumSq / (float)okCount) - (mean * mean);
  imuCalStdDps = sqrtf(max(0.0f, variance));
  if (imuCalStdDps > MPU6050_CAL_MAX_STD_DPS) {
    imuCalValid = false;
    return false;
  }

  imuBiasDps = mean;
  imuGyroDps = 0.0f;
  imuGyroDpsFilt = 0.0f;
  imuCalValid = true;
  return true;
}

bool imuInit() {
  Wire.begin();
  delay(10);

  bool ok = true;
  ok &= imuWriteReg(0x6B, 0x00);  // Wake up MPU6050.
  ok &= imuWriteReg(0x1B, 0x08);  // Gyro FS_SEL=1 => +-500 dps.
  ok &= imuWriteReg(0x1A, 0x03);  // DLPF for less noise.
  delay(50);

  imuOk = ok;
  imuBiasDps = 0.0f;
  imuYawDeg = 0.0f;
  imuYawRawDeg = 0.0f;
  imuGyroDps = 0.0f;
  imuGyroDpsFilt = 0.0f;
  imuCalStdDps = 0.0f;
  imuCalValid = false;

  if (imuOk) {
    imuCalibrateBias(400);
  }
  return imuOk;
}

void imuUpdate(float dt) {
  if (!imuOk || dt <= 0.0001f) {
    imuGyroDps = 0.0f;
    imuGyroDpsFilt = 0.0f;
    return;
  }

  int16_t gzRaw = 0;
  if (!imuReadGyroZRaw(gzRaw)) {
    imuGyroDps = 0.0f;
    return;
  }

  imuGyroDps = ((float)gzRaw) / MPU6050_GYRO_Z_SCALE - imuBiasDps;
  imuGyroDpsFilt += MPU6050_GYRO_FILTER_ALPHA * (imuGyroDps - imuGyroDpsFilt);
  imuYawRawDeg = wrapAngleDeg(imuYawRawDeg + imuGyroDps * dt);
  imuYawDeg = wrapAngleDeg(imuYawDeg + imuGyroDpsFilt * dt);
}

void updateMecanumTargets() {
  float vx = constrain(commandForwardRpm, -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT);
  float vy = constrain(commandStrafeRpm, -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT) * (float)strafeSign;
  float omega = constrain(commandRotateRpm, -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT) * (float)rotateSign;

  // Confirmed logical order [BR, FR, BL, FL].
  rawTargetRpm[0] = vx - vy + omega;
  rawTargetRpm[1] = vx + vy + omega;
  rawTargetRpm[2] = vx + vy - omega;
  rawTargetRpm[3] = vx - vy - omega;

  float maxAbsTarget = 0.0f;
  for (uint8_t i = 0; i < 4; i++) {
    maxAbsTarget = max(maxAbsTarget, fabsf(rawTargetRpm[i]));
  }

  targetScale = 1.0f;
  if (maxAbsTarget > RPM_TARGET_LIMIT && maxAbsTarget > 0.0001f) {
    targetScale = RPM_TARGET_LIMIT / maxAbsTarget;
  }

  for (uint8_t i = 0; i < 4; i++) {
    targetRpm[i] = constrain(rawTargetRpm[i] * targetScale, -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT);
  }
}

float displayedOdomForwardCm() {
  return ((float)odomForwardSign) * odomForwardCm * odomForwardScale;
}

float displayedOdomStrafeCm() {
  return ((float)odomStrafeSign) * odomStrafeCm * odomStrafeScale;
}

float displayedOdomYawWheelCm() {
  return ((float)odomYawSign) * odomYawWheelCm;
}

float encoderYawDeg() {
  if (ENCODER_YAW_CM_PER_DEG <= 0.0001f) {
    return 0.0f;
  }
  return displayedOdomYawWheelCm() / ENCODER_YAW_CM_PER_DEG;
}

float displayedImuYawDeg() {
  return wrapAngleDeg(((float)odomYawSign) * imuYawDeg);
}

float displayedImuYawRawDeg() {
  return wrapAngleDeg(((float)odomYawSign) * imuYawRawDeg);
}

float displayedImuGyroDps() {
  return ((float)odomYawSign) * imuGyroDps;
}

float displayedImuGyroDpsFilt() {
  return ((float)odomYawSign) * imuGyroDpsFilt;
}

bool imuHeadingReady() {
  return imuOk && imuCalValid;
}

float displayedOdomYawDeg() {
  return imuHeadingReady() ? displayedImuYawDeg() : encoderYawDeg();
}

float runBodyPositionPi(uint8_t axis, float error, float kp, float ki, float dt, float outputLimitRpm, float tolerance) {
  BodyPiState& state = positionPi[axis];
  state.error = error;

  if (fabsf(error) <= tolerance) {
    resetBodyPiState(axis);
    return 0.0f;
  }

  state.pTerm = kp * error;

  float candidateIntegral = state.integralError;
  if (ki > 0.0001f && dt > 0.0001f) {
    candidateIntegral += error * dt;
  }

  float candidateITerm = ki * candidateIntegral;
  float candidateRawOutput = state.pTerm + candidateITerm;
  bool pushesHighSaturation = candidateRawOutput > outputLimitRpm && error > 0.0f;
  bool pushesLowSaturation = candidateRawOutput < -outputLimitRpm && error < 0.0f;
  if (!pushesHighSaturation && !pushesLowSaturation) {
    state.integralError = candidateIntegral;
  }

  state.iTerm = ki * state.integralError;
  state.rawOutputRpm = state.pTerm + state.iTerm;
  state.outputRpm = constrain(state.rawOutputRpm, -outputLimitRpm, outputLimitRpm);
  return state.outputRpm;
}

float runYawPositionController(float error, float outputLimitRpm) {
  BodyPiState& state = positionPi[2];
  state.error = error;

  const float yawRateDps = imuHeadingReady() ? displayedImuGyroDpsFilt() : 0.0f;
  const float absError = fabsf(error);
  const float absRate = fabsf(yawRateDps);

  float activeError = error;
  if (absError <= YAW_CONTROL_DEADBAND_DEG) {
    activeError = 0.0f;
  }

  state.pTerm = yawKp * activeError;
  state.iTerm = 0.0f;

  float dampingRpm = imuHeadingReady() ? (-YAW_DAMP_RPM_PER_DPS * yawRateDps) : 0.0f;
  if (absError <= yawToleranceDeg && absRate <= YAW_SETTLE_RATE_DPS) {
    resetBodyPiState(2);
    return 0.0f;
  }

  state.rawOutputRpm = state.pTerm + dampingRpm;

  float limit = outputLimitRpm;
  if (absError <= YAW_NEAR_TARGET_DEG) {
    limit = min(limit, YAW_NEAR_TARGET_MAX_RPM);
  }

  state.outputRpm = constrain(state.rawOutputRpm, -limit, limit);
  return state.outputRpm;
}

void updatePositionController(float dt) {
  if (!positionModeActive) {
    return;
  }

  positionErrorForwardCm = targetForwardCm - displayedOdomForwardCm();
  positionErrorStrafeCm = targetStrafeCm - displayedOdomStrafeCm();
  positionErrorYawDeg = shortestAngleErrorDeg(targetYawDeg, displayedOdomYawDeg());

  bool pureYawMove = fabsf(targetForwardCm) <= positionToleranceCm &&
                     fabsf(targetStrafeCm) <= positionToleranceCm;
  bool forwardDone = pureYawMove || fabsf(positionErrorForwardCm) <= positionToleranceCm;
  bool strafeDone = pureYawMove || fabsf(positionErrorStrafeCm) <= positionToleranceCm;
  bool yawErrorDone = fabsf(positionErrorYawDeg) <= yawToleranceDeg;
  bool yawRateDone = !imuHeadingReady() || fabsf(displayedImuGyroDpsFilt()) <= YAW_SETTLE_RATE_DPS;
  bool allAxesSettled = forwardDone && strafeDone && yawErrorDone && yawRateDone;
  if (allAxesSettled) {
    if (positionSettleTicks < POSITION_SETTLE_TICKS_REQUIRED) {
      positionSettleTicks++;
    }
  } else {
    positionSettleTicks = 0;
  }
  positionTargetReached = positionSettleTicks >= POSITION_SETTLE_TICKS_REQUIRED;

  if (positionTargetReached) {
    commandForwardRpm = 0.0f;
    commandStrafeRpm = 0.0f;
    commandRotateRpm = 0.0f;
    updateMecanumTargets();
    positionModeActive = false;
    controllerEnabled = false;
    resetAllPiStates();
    resetAllBodyPiStates();
    stopAllMotors();
    return;
  }

  if (pureYawMove) {
    commandForwardRpm = 0.0f;
    commandStrafeRpm = 0.0f;
    resetBodyPiState(0);
    resetBodyPiState(1);
  } else {
    commandForwardRpm = runBodyPositionPi(0, positionErrorForwardCm, positionKp, positionKi, dt, positionMaxRpm, positionToleranceCm);
    commandStrafeRpm = runBodyPositionPi(1, positionErrorStrafeCm, positionKp, positionKi, dt, positionMaxRpm, positionToleranceCm);
  }

  float yawControlRpm = runYawPositionController(positionErrorYawDeg, positionMaxRpm);
  // The heading loop works in the displayed/right-hand-rule yaw convention:
  // left/CCW is positive, right/CW is negative. Keep commandRotateRpm in that
  // logical convention and let updateMecanumTargets() apply the calibrated
  // hardware rotateSign exactly once at the wheel-mixing boundary.
  commandRotateRpm = yawControlRpm;
  updateMecanumTargets();
}

float wheelTargetRpm(uint8_t wheel) {
  if (!controllerEnabled) {
    return 0.0f;
  }
  return targetRpm[wheel];
}

void runVelocityPi(uint8_t wheel, float target, float dt) {
  if (fabsf(target) < 0.01f) {
    resetPiState(wheel);
    setMotorPwmSigned(wheel, 0);
    return;
  }

  WheelPiState& state = piState[wheel];
  state.errorRpm = target - measuredRpm[wheel];
  state.pTerm = velocityKp[wheel] * state.errorRpm;

  float candidateIntegral = state.integralError;
  if (velocityKi[wheel] > 0.0001f) {
    candidateIntegral += state.errorRpm * dt;
  }
  float candidateITerm = velocityKi[wheel] * candidateIntegral;
  float candidateRawOutput = state.pTerm + candidateITerm;

  bool pushesHighSaturation = candidateRawOutput > PWM_MAX && state.errorRpm > 0.0f;
  bool pushesLowSaturation = candidateRawOutput < -PWM_MAX && state.errorRpm < 0.0f;
  if (!pushesHighSaturation && !pushesLowSaturation) {
    state.integralError = candidateIntegral;
  }

  state.iTerm = velocityKi[wheel] * state.integralError;
  state.rawOutput = state.pTerm + state.iTerm;
  state.output = constrain(state.rawOutput, -((float)PWM_MAX), (float)PWM_MAX);

  setMotorPwmSigned(wheel, (int)lroundf(state.output));
}

void runControlTick(float dt) {
  if (timedRunActive && ((uint32_t)(millis() - timedRunStartMs) >= timedRunDurationMs)) {
    timedRunActive = false;
    positionModeActive = false;
    controllerEnabled = false;
    resetAllPiStates();
    resetAllBodyPiStates();
    stopAllMotors();
  }

  updateMeasuredRpm(dt);
  updateOdometry();
  imuUpdate(dt);
  updatePositionController(dt);
  lastLoopDtMs = dt * 1000.0f;

  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    runVelocityPi(wheel, wheelTargetRpm(wheel), dt);
  }
}

void captureTelemetry(TelemetrySnapshot& snap) {
  STATE_LOCK();
  snap.enabled = controllerEnabled;
  snap.timedRunActive = timedRunActive;
  snap.positionModeActive = positionModeActive;
  snap.positionTargetReached = positionTargetReached;
  snap.positionMoveId = positionMoveId;
  snap.positionKp = positionKp;
  snap.positionKi = positionKi;
  snap.yawKp = yawKp;
  snap.yawKi = yawKi;
  snap.positionMaxRpm = positionMaxRpm;
  snap.positionToleranceCm = positionToleranceCm;
  snap.yawToleranceDeg = yawToleranceDeg;
  snap.imuOk = imuOk;
  snap.headingUsesImu = imuHeadingReady();
  snap.imuYawDeg = displayedImuYawDeg();
  snap.imuYawRawDeg = displayedImuYawRawDeg();
  snap.encoderYawDeg = encoderYawDeg();
  snap.imuGyroDps = displayedImuGyroDps();
  snap.imuGyroDpsFilt = displayedImuGyroDpsFilt();
  snap.imuBiasDps = imuBiasDps;
  snap.imuCalStdDps = imuCalStdDps;
  snap.imuCalValid = imuCalValid;
  snap.loopDtMs = lastLoopDtMs;
  snap.nowMs = millis();
  snap.timedRunId = timedRunId;
  snap.timedRunDurationMs = timedRunDurationMs;
  if (timedRunStartMs == 0) {
    snap.timedRunElapsedMs = 0;
  } else {
    snap.timedRunElapsedMs = min((uint32_t)(snap.nowMs - timedRunStartMs), timedRunDurationMs);
  }
  snap.commandForwardRpm = commandForwardRpm;
  snap.commandStrafeRpm = commandStrafeRpm;
  snap.commandRotateRpm = commandRotateRpm;
  snap.targetScale = targetScale;
  snap.strafeSign = strafeSign;
  snap.rotateSign = rotateSign;
  snap.odomForwardSign = odomForwardSign;
  snap.odomStrafeSign = odomStrafeSign;
  snap.odomYawSign = odomYawSign;
  snap.odomForwardScale = odomForwardScale;
  snap.odomStrafeScale = odomStrafeScale;
  snap.odomForwardCm = displayedOdomForwardCm();
  snap.odomStrafeCm = displayedOdomStrafeCm();
  snap.odomYawDeg = displayedOdomYawDeg();
  snap.targetForwardCm = targetForwardCm;
  snap.targetStrafeCm = targetStrafeCm;
  snap.targetYawDeg = targetYawDeg;
  snap.positionErrorForwardCm = positionErrorForwardCm;
  snap.positionErrorStrafeCm = positionErrorStrafeCm;
  snap.positionErrorYawDeg = positionErrorYawDeg;
  for (uint8_t axis = 0; axis < 3; axis++) {
    snap.positionPTerm[axis] = positionPi[axis].pTerm;
    snap.positionITerm[axis] = positionPi[axis].iTerm;
    snap.positionOutputRpm[axis] = positionPi[axis].outputRpm;
  }

  for (uint8_t i = 0; i < 4; i++) {
    snap.kp[i] = velocityKp[i];
    snap.ki[i] = velocityKi[i];
    snap.odomWheelCm[i] = odomWheelCm[i];
    snap.rawTargetRpm[i] = rawTargetRpm[i];
    snap.targetRpm[i] = targetRpm[i];
    snap.appliedTargetRpm[i] = wheelTargetRpm(i);
    snap.rawRpm[i] = rawRpm[i];
    snap.measuredRpm[i] = measuredRpm[i];
    snap.errorRpm[i] = piState[i].errorRpm;
    snap.pTerm[i] = piState[i].pTerm;
    snap.iTerm[i] = piState[i].iTerm;
    snap.rawOutput[i] = piState[i].rawOutput;
    snap.output[i] = piState[i].output;
    snap.integralError[i] = piState[i].integralError;
    snap.pwm[i] = pwmCmd[i];
    snap.hardwarePwm[i] = hardwarePwmCmd[i];
    snap.encoderSign[i] = encoderSign[i];
    snap.motorSign[i] = motorSign[i];
  }
  STATE_UNLOCK();

  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    int32_t rawCount = readEncoderCount(motors[wheel].encoderIndex);
    snap.rawCounts[wheel] = rawCount;
    snap.signedCounts[wheel] = ((int32_t)snap.encoderSign[wheel]) * rawCount;
  }
}

void appendFloatArray(String& body, const float values[4], uint8_t digits) {
  body += "[";
  for (uint8_t i = 0; i < 4; i++) {
    body += String(values[i], (unsigned int)digits);
    if (i < 3) body += ",";
  }
  body += "]";
}

void appendFloatArray3(String& body, const float values[3], uint8_t digits) {
  body += "[";
  for (uint8_t i = 0; i < 3; i++) {
    body += String(values[i], (unsigned int)digits);
    if (i < 2) body += ",";
  }
  body += "]";
}

void appendInt16Array(String& body, const int16_t values[4]) {
  body += "[";
  for (uint8_t i = 0; i < 4; i++) {
    body += String(values[i]);
    if (i < 3) body += ",";
  }
  body += "]";
}

void appendInt32Array(String& body, const int32_t values[4]) {
  body += "[";
  for (uint8_t i = 0; i < 4; i++) {
    body += String(values[i]);
    if (i < 3) body += ",";
  }
  body += "]";
}

void appendInt8Array(String& body, const int8_t values[4]) {
  body += "[";
  for (uint8_t i = 0; i < 4; i++) {
    body += String(values[i]);
    if (i < 3) body += ",";
  }
  body += "]";
}

String telemetryJson() {
  TelemetrySnapshot snap = {};
  captureTelemetry(snap);

  String body;
  body.reserve(3600);
  body += "{";
  body += "\"enabled\":" + String(snap.enabled ? "true" : "false") + ",";
  body += "\"positionModeActive\":" + String(snap.positionModeActive ? "true" : "false") + ",";
  body += "\"positionTargetReached\":" + String(snap.positionTargetReached ? "true" : "false") + ",";
  body += "\"positionMoveId\":" + String(snap.positionMoveId) + ",";
  body += "\"nowMs\":" + String(snap.nowMs) + ",";
  body += "\"kp\":";
  appendFloatArray(body, snap.kp, 4);
  body += ",\"ki\":";
  appendFloatArray(body, snap.ki, 4);
  body += ",";
  body += "\"loopDtMs\":" + String(snap.loopDtMs, 3) + ",";
  body += "\"timed\":{";
  body += "\"active\":" + String(snap.timedRunActive ? "true" : "false") + ",";
  body += "\"id\":" + String(snap.timedRunId) + ",";
  body += "\"durationMs\":" + String(snap.timedRunDurationMs) + ",";
  body += "\"elapsedMs\":" + String(snap.timedRunElapsedMs);
  body += "},";
  body += "\"countsPerRev\":" + String(COUNTS_PER_WHEEL_REV, 1) + ",";
  body += "\"rpmLimit\":" + String(RPM_TARGET_LIMIT, 1) + ",";
  body += "\"rpmFilterAlpha\":" + String(RPM_FILTER_ALPHA, 3) + ",";
  body += "\"command\":{";
  body += "\"forwardRpm\":" + String(snap.commandForwardRpm, 3) + ",";
  body += "\"strafeRpm\":" + String(snap.commandStrafeRpm, 3) + ",";
  body += "\"rotateRpm\":" + String(snap.commandRotateRpm, 3) + ",";
  body += "\"targetScale\":" + String(snap.targetScale, 4) + ",";
  body += "\"strafeSign\":" + String(snap.strafeSign) + ",";
  body += "\"rotateSign\":" + String(snap.rotateSign) + ",";
  body += "\"odomForwardSign\":" + String(snap.odomForwardSign) + ",";
  body += "\"odomStrafeSign\":" + String(snap.odomStrafeSign) + ",";
  body += "\"odomYawSign\":" + String(snap.odomYawSign) + ",";
  body += "\"odomForwardScale\":" + String(snap.odomForwardScale, 5) + ",";
  body += "\"odomStrafeScale\":" + String(snap.odomStrafeScale, 5);
  body += "},";
  body += "\"odometry\":{";
  body += "\"forwardCm\":" + String(snap.odomForwardCm, 3) + ",";
  body += "\"strafeCm\":" + String(snap.odomStrafeCm, 3) + ",";
  body += "\"yawDeg\":" + String(snap.odomYawDeg, 3) + ",";
  body += "\"yawWheelCm\":" + String(displayedOdomYawWheelCm(), 3) + ",";
  body += "\"wheelCm\":";
  appendFloatArray(body, snap.odomWheelCm, 3);
  body += "},";
  body += "\"wheelDiameterCm\":" + String(WHEEL_DIAMETER_CM, 3) + ",";
  body += "\"imu\":{";
  body += "\"ok\":" + String(snap.imuOk ? "true" : "false") + ",";
  body += "\"headingUsesImu\":" + String(snap.headingUsesImu ? "true" : "false") + ",";
  body += "\"yawDeg\":" + String(snap.imuYawDeg, 3) + ",";
  body += "\"yawRawDeg\":" + String(snap.imuYawRawDeg, 3) + ",";
  body += "\"encoderYawDeg\":" + String(snap.encoderYawDeg, 3) + ",";
  body += "\"gyroDps\":" + String(snap.imuGyroDps, 3) + ",";
  body += "\"gyroFiltDps\":" + String(snap.imuGyroDpsFilt, 3) + ",";
  body += "\"biasDps\":" + String(snap.imuBiasDps, 4) + ",";
  body += "\"calStdDps\":" + String(snap.imuCalStdDps, 4) + ",";
  body += "\"calValid\":" + String(snap.imuCalValid ? "true" : "false");
  body += "},";
  body += "\"position\":{";
  body += "\"targetForwardCm\":" + String(snap.targetForwardCm, 3) + ",";
  body += "\"targetStrafeCm\":" + String(snap.targetStrafeCm, 3) + ",";
  body += "\"targetYawDeg\":" + String(snap.targetYawDeg, 3) + ",";
  body += "\"errorForwardCm\":" + String(snap.positionErrorForwardCm, 3) + ",";
  body += "\"errorStrafeCm\":" + String(snap.positionErrorStrafeCm, 3) + ",";
  body += "\"errorYawDeg\":" + String(snap.positionErrorYawDeg, 3) + ",";
  body += "\"kp\":" + String(snap.positionKp, 4) + ",";
  body += "\"ki\":" + String(snap.positionKi, 4) + ",";
  body += "\"yawKp\":" + String(snap.yawKp, 4) + ",";
  body += "\"yawKi\":" + String(snap.yawKi, 4) + ",";
  body += "\"maxRpm\":" + String(snap.positionMaxRpm, 3) + ",";
  body += "\"toleranceCm\":" + String(snap.positionToleranceCm, 3) + ",";
  body += "\"yawToleranceDeg\":" + String(snap.yawToleranceDeg, 3) + ",";
  body += "\"pTerm\":";
  appendFloatArray3(body, snap.positionPTerm, 3);
  body += ",\"iTerm\":";
  appendFloatArray3(body, snap.positionITerm, 3);
  body += ",\"outputRpm\":";
  appendFloatArray3(body, snap.positionOutputRpm, 3);
  body += "},";
  body += "\"rawTargetRpm\":";
  appendFloatArray(body, snap.rawTargetRpm, 3);
  body += ",";
  body += "\"targetRpm\":";
  appendFloatArray(body, snap.targetRpm, 3);
  body += ",\"appliedTargetRpm\":";
  appendFloatArray(body, snap.appliedTargetRpm, 3);
  body += ",\"rawRpm\":";
  appendFloatArray(body, snap.rawRpm, 3);
  body += ",\"rpm\":";
  appendFloatArray(body, snap.measuredRpm, 3);
  body += ",\"errorRpm\":";
  appendFloatArray(body, snap.errorRpm, 3);
  body += ",\"pTerm\":";
  appendFloatArray(body, snap.pTerm, 3);
  body += ",\"iTerm\":";
  appendFloatArray(body, snap.iTerm, 3);
  body += ",\"rawOutput\":";
  appendFloatArray(body, snap.rawOutput, 3);
  body += ",\"output\":";
  appendFloatArray(body, snap.output, 3);
  body += ",\"integralError\":";
  appendFloatArray(body, snap.integralError, 3);
  body += ",\"pwm\":";
  appendInt16Array(body, snap.pwm);
  body += ",\"hardwarePwm\":";
  appendInt16Array(body, snap.hardwarePwm);
  body += ",\"encoderSign\":";
  appendInt8Array(body, snap.encoderSign);
  body += ",\"motorSign\":";
  appendInt8Array(body, snap.motorSign);
  body += ",\"rawEncoderCounts\":";
  appendInt32Array(body, snap.rawCounts);
  body += ",\"signedEncoderCounts\":";
  appendInt32Array(body, snap.signedCounts);
  body += "}";
  return body;
}

void sendJsonLine(const String& line) {
  SERIAL_LOCK();
  Serial.println(line);
  SERIAL_UNLOCK();
}

const char* piResultText(uint8_t code) {
  switch (code) {
    case 1: return "completed";
    case 2: return "stopped";
    default: return "idle";
  }
}

const char* piModeText() {
  if (positionModeActive) return "driving";
  if (timedRunActive) return "timed";
  return "idle";
}

float piPoseForwardCm() {
  return piMoveActive ? (piMoveStartForwardCm + displayedOdomForwardCm()) : piOdomForwardCm;
}

float piPoseStrafeCm() {
  return piMoveActive ? (piMoveStartStrafeCm + displayedOdomStrafeCm()) : piOdomStrafeCm;
}

void latchPiMoveOdom() {
  if (!piMoveActive) {
    return;
  }
  piOdomForwardCm = piMoveStartForwardCm + displayedOdomForwardCm();
  piOdomStrafeCm = piMoveStartStrafeCm + displayedOdomStrafeCm();
  piMoveActive = false;
}

void resetPiBridgeOdom() {
  piOdomForwardCm = 0.0f;
  piOdomStrafeCm = 0.0f;
  piMoveStartForwardCm = 0.0f;
  piMoveStartStrafeCm = 0.0f;
  piMoveActive = false;
}

void queuePiDone(uint32_t seq, uint8_t code) {
  if (seq == 0) {
    return;
  }
  piPendingDone = true;
  piPendingDoneSeq = seq;
  piPendingDoneCode = code;
}

String buildPiAckJson(uint32_t seq, const char* cmd, bool ok, const String& message) {
  String s;
  s.reserve(160);
  s += "{\"type\":\"ack\",\"seq\":" + String(seq);
  s += ",\"cmd\":\"" + String(cmd) + "\"";
  s += ",\"ok\":" + String(ok ? "true" : "false");
  s += ",\"message\":\"" + message + "\"}";
  return s;
}

String buildPiLimitStatusJson(uint32_t seq, uint8_t pin, bool activeLow, int raw) {
  bool pressed = activeLow ? raw == LOW : raw == HIGH;
  String s;
  s.reserve(140);
  s += "{\"type\":\"limit\",\"seq\":" + String(seq);
  s += ",\"pin\":" + String(pin);
  s += ",\"pressed\":" + String(pressed ? "true" : "false");
  s += ",\"raw\":" + String(raw);
  s += ",\"activeLow\":" + String(activeLow ? "true" : "false");
  s += "}";
  return s;
}

String buildPiDoneJson(uint32_t seq, uint8_t code) {
  String s;
  s.reserve(220);
  s += "{\"type\":\"done\",\"seq\":" + String(seq);
  s += ",\"result\":\"" + String(piResultText(code)) + "\"";
  s += ",\"headingDeg\":" + String(displayedOdomYawDeg(), 2);
  s += ",\"forwardCm\":" + String(piPoseForwardCm(), 2);
  s += ",\"strafeCm\":" + String(piPoseStrafeCm(), 2);
  float localForward = displayedOdomForwardCm();
  float localStrafe = displayedOdomStrafeCm();
  s += ",\"progressCm\":" + String(sqrtf(localForward * localForward + localStrafe * localStrafe), 2);
  s += "}";
  return s;
}

String buildPiTelemetryJson() {
  String s;
  s.reserve(900);
  float localForward = displayedOdomForwardCm();
  float localStrafe = displayedOdomStrafeCm();
  float localProgress = sqrtf(localForward * localForward + localStrafe * localStrafe);
  float localRemaining = sqrtf(positionErrorForwardCm * positionErrorForwardCm + positionErrorStrafeCm * positionErrorStrafeCm);

  s += "{\"type\":\"telemetry\"";
  s += ",\"mode\":\"" + String(piModeText()) + "\"";
  s += ",\"seq\":" + String(piActiveSeq);
  s += ",\"imu\":{\"ok\":" + String(imuOk ? "true" : "false");
  s += ",\"headingUsesImu\":" + String(imuHeadingReady() ? "true" : "false");
  s += ",\"calValid\":" + String(imuCalValid ? "true" : "false");
  s += ",\"yawDeg\":" + String(displayedOdomYawDeg(), 3);
  s += ",\"rawYawDeg\":" + String(displayedImuYawRawDeg(), 3);
  s += ",\"controlYawDeg\":" + String(displayedImuYawDeg(), 3);
  s += ",\"encoderYawDeg\":" + String(encoderYawDeg(), 3);
  s += ",\"gyroDps\":" + String(displayedImuGyroDps(), 3);
  s += ",\"gyroFiltDps\":" + String(displayedImuGyroDpsFilt(), 3);
  s += ",\"biasDps\":" + String(imuBiasDps, 4);
  s += ",\"calStdDps\":" + String(imuCalStdDps, 4);
  s += "}";
  s += ",\"pose\":{\"forwardCm\":" + String(piPoseForwardCm(), 2);
  s += ",\"strafeCm\":" + String(piPoseStrafeCm(), 2);
  s += ",\"yawDeg\":" + String(displayedOdomYawDeg(), 2);
  s += ",\"progressCm\":" + String(localProgress, 2);
  s += ",\"remainingCm\":" + String(localRemaining, 2);
  s += ",\"localForwardCm\":" + String(localForward, 2);
  s += ",\"localStrafeCm\":" + String(localStrafe, 2);
  s += "}";
  s += ",\"move\":{\"phase\":" + String(positionModeActive ? 2 : 0);
  s += ",\"angleDeg\":" + String(piLastMoveAngleDeg, 2);
  s += ",\"distanceCm\":" + String(piLastMoveDistanceCm, 2);
  s += ",\"headingTargetDeg\":" + String(targetYawDeg, 2);
  s += ",\"headingErrorDeg\":" + String(positionErrorYawDeg, 2);
  s += ",\"done\":" + String(positionTargetReached ? "true" : "false");
  s += "}";
  s += ",\"rpm\":[" + String(measuredRpm[0], 2) + "," + String(measuredRpm[1], 2) + "," + String(measuredRpm[2], 2) + "," + String(measuredRpm[3], 2) + "]";
  s += ",\"targetRpm\":[" + String(targetRpm[0], 2) + "," + String(targetRpm[1], 2) + "," + String(targetRpm[2], 2) + "," + String(targetRpm[3], 2) + "]";
  s += ",\"pwm\":[" + String(pwmCmd[0]) + "," + String(pwmCmd[1]) + "," + String(pwmCmd[2]) + "," + String(pwmCmd[3]) + "]";
  s += ",\"rawEncoderCounts\":[" + String(readEncoderCount(motors[0].encoderIndex)) + "," + String(readEncoderCount(motors[1].encoderIndex)) + "," + String(readEncoderCount(motors[2].encoderIndex)) + "," + String(readEncoderCount(motors[3].encoderIndex)) + "]";
  s += ",\"signedEncoderCounts\":[" + String(((int32_t)encoderSign[0]) * readEncoderCount(motors[0].encoderIndex)) + "," + String(((int32_t)encoderSign[1]) * readEncoderCount(motors[1].encoderIndex)) + "," + String(((int32_t)encoderSign[2]) * readEncoderCount(motors[2].encoderIndex)) + "," + String(((int32_t)encoderSign[3]) * readEncoderCount(motors[3].encoderIndex)) + "]";
  s += "}";
  return s;
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

void stopPiMotion(uint8_t resultCode) {
  uint32_t oldSeq = piActiveSeq;
  latchPiMoveOdom();
  piActiveSeq = 0;
  controllerEnabled = false;
  positionModeActive = false;
  positionTargetReached = false;
  timedRunActive = false;
  timedRunStartMs = 0;
  commandForwardRpm = 0.0f;
  commandStrafeRpm = 0.0f;
  commandRotateRpm = 0.0f;
  updateMecanumTargets();
  resetAllPiStates();
  resetAllBodyPiStates();
  stopAllMotors();
  queuePiDone(oldSeq, resultCode);
}

void startPiPositionMove(uint32_t seq, float angleDeg, float distanceCm, float headingDeg) {
  if (piActiveSeq != 0) {
    stopPiMotion(2);
  }

  if (distanceCm < 0.0f) {
    distanceCm = -distanceCm;
    angleDeg = wrapAngleDeg(angleDeg + 180.0f);
  }

  float rad = angleDeg * DEG_TO_RAD;
  targetForwardCm = cosf(rad) * distanceCm;
  targetStrafeCm = -sinf(rad) * distanceCm;
  targetYawDeg = wrapAngleDeg(headingDeg);

  positionMoveId++;
  if (positionMoveId == 0) positionMoveId = 1;
  piActiveSeq = seq;
  piMoveActive = true;
  piMoveStartForwardCm = piOdomForwardCm;
  piMoveStartStrafeCm = piOdomStrafeCm;
  piLastMoveAngleDeg = angleDeg;
  piLastMoveDistanceCm = distanceCm;

  resetOdometryToCurrentCounts();
  resetRpmMeasurementsToCurrentCounts();
  resetAllPiStates();
  resetAllBodyPiStates();
  positionErrorForwardCm = targetForwardCm;
  positionErrorStrafeCm = targetStrafeCm;
  positionErrorYawDeg = shortestAngleErrorDeg(targetYawDeg, displayedOdomYawDeg());
  positionTargetReached = false;
  positionModeActive = true;
  timedRunActive = false;
  timedRunStartMs = 0;
  controllerEnabled = true;
  updatePositionController(NOMINAL_DT_S);
}

void startPiTwist(float forwardRpm, float strafeRpm, float turnRpm, uint32_t timeoutMs) {
  if (piActiveSeq != 0) {
    stopPiMotion(2);
  }

  latchPiMoveOdom();
  piActiveSeq = 0;
  positionModeActive = false;
  positionTargetReached = false;
  timedRunActive = false;
  timedRunStartMs = 0;

  commandForwardRpm = constrain(forwardRpm, -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT);
  commandStrafeRpm = constrain(strafeRpm, -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT);
  commandRotateRpm = constrain(turnRpm, -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT);
  updateMecanumTargets();

  if (fabsf(commandForwardRpm) <= 0.001f &&
      fabsf(commandStrafeRpm) <= 0.001f &&
      fabsf(commandRotateRpm) <= 0.001f) {
    controllerEnabled = false;
    resetAllPiStates();
    resetAllBodyPiStates();
    stopAllMotors();
    return;
  }

  timedRunDurationMs = timeoutMs < 50 ? 50 : timeoutMs;
  timedRunStartMs = millis();
  timedRunId++;
  if (timedRunId == 0) timedRunId = 1;
  timedRunActive = true;
  controllerEnabled = true;
  resetAllPiStates();
  resetAllBodyPiStates();
  resetRpmMeasurementsToCurrentCounts();
}

void handlePiCommandLine(String line) {
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
    sendJsonLine(buildPiAckJson(seq, "STATUS", true, "telemetry_follows"));
    STATE_LOCK();
    String telemetry = buildPiTelemetryJson();
    STATE_UNLOCK();
    sendJsonLine(telemetry);
    return;
  }

  if (cmd == "LIMIT_STATUS") {
    uint8_t pin = (uint8_t)getUIntArg(line, "pin", DEFAULT_LIMIT_STATUS_PIN);
    bool activeLow = getUIntArg(line, "activeLow", 1) != 0;
    pinMode(pin, activeLow ? INPUT_PULLUP : INPUT);
    delayMicroseconds(50);
    int raw = digitalRead(pin);
    sendJsonLine(buildPiLimitStatusJson(seq, pin, activeLow, raw));
    return;
  }

  if (cmd == "STOP") {
    STATE_LOCK();
    stopPiMotion(2);
    STATE_UNLOCK();
    sendJsonLine(buildPiAckJson(seq, "STOP", true, "stopped"));
    return;
  }

  if (cmd == "RESET_ENC") {
    STATE_LOCK();
    resetEncoderMeasurements();
    resetOdometryToCurrentCounts();
    resetPiBridgeOdom();
    resetAllPiStates();
    resetAllBodyPiStates();
    STATE_UNLOCK();
    sendJsonLine(buildPiAckJson(seq, "RESET_ENC", true, "encoders_reset"));
    return;
  }

  if (cmd == "RESET_ODOM") {
    STATE_LOCK();
    resetOdometryToCurrentCounts();
    resetPiBridgeOdom();
    positionErrorForwardCm = targetForwardCm;
    positionErrorStrafeCm = targetStrafeCm;
    positionErrorYawDeg = shortestAngleErrorDeg(targetYawDeg, displayedOdomYawDeg());
    resetAllBodyPiStates();
    STATE_UNLOCK();
    sendJsonLine(buildPiAckJson(seq, "RESET_ODOM", true, "odometry_reset"));
    return;
  }

  if (cmd == "ZERO_IMU") {
    STATE_LOCK();
    stopPiMotion(2);
    imuYawDeg = 0.0f;
    imuYawRawDeg = 0.0f;
    targetYawDeg = 0.0f;
    positionErrorYawDeg = 0.0f;
    STATE_UNLOCK();
    sendJsonLine(buildPiAckJson(seq, "ZERO_IMU", true, "yaw_zeroed"));
    return;
  }

  if (cmd == "CAL_IMU") {
    bool ok = false;
    STATE_LOCK();
    stopPiMotion(2);
    ok = imuCalibrateBias(500);
    STATE_UNLOCK();
    sendJsonLine(buildPiAckJson(seq, "CAL_IMU", ok, ok ? "imu_bias_calibrated" : "imu_calibration_failed"));
    return;
  }

  if (cmd == "INIT_IMU") {
    bool ok = false;
    bool imuPresent = false;
    STATE_LOCK();
    stopPiMotion(2);
    imuPresent = imuOk;
    if (imuOk) {
      ok = imuCalibrateBias(500);
      if (ok) {
        imuYawDeg = 0.0f;
        imuYawRawDeg = 0.0f;
        targetYawDeg = 0.0f;
        positionErrorYawDeg = 0.0f;
      }
    }
    STATE_UNLOCK();
    if (imuPresent) {
      sendJsonLine(buildPiAckJson(seq, "INIT_IMU", ok, ok ? "imu_calibrated_and_zeroed" : "imu_calibration_failed"));
    } else {
      sendJsonLine(buildPiAckJson(seq, "INIT_IMU", true, "imu_missing_using_encoder_heading"));
    }
    return;
  }

  if (cmd == "MOVE") {
    float angleDeg = getFloatArg(line, "angle", 0.0f);
    float distCm = getFloatArg(line, "dist", 0.0f);
    float headingDeg = getFloatArg(line, "heading", 0.0f);
    STATE_LOCK();
    startPiPositionMove(seq, angleDeg, distCm, headingDeg);
    STATE_UNLOCK();
    sendJsonLine(buildPiAckJson(seq, "MOVE", true, "accepted"));
    return;
  }

  if (cmd == "TWIST") {
    float forwardRpm = getFloatArg(line, "forward", 0.0f);
    float strafeRpm = getFloatArg(line, "strafe", 0.0f);
    float turnRpm = getFloatArg(line, "turn", 0.0f);
    uint32_t timeoutMs = getUIntArg(line, "timeout", 300);
    STATE_LOCK();
    startPiTwist(forwardRpm, strafeRpm, turnRpm, timeoutMs);
    STATE_UNLOCK();
    sendJsonLine(buildPiAckJson(seq, "TWIST", true, "accepted"));
    return;
  }

  if (cmd == "TURN") {
    float headingDeg = getFloatArg(line, "heading", 0.0f);
    STATE_LOCK();
    startPiPositionMove(seq, 0.0f, 0.0f, headingDeg);
    STATE_UNLOCK();
    sendJsonLine(buildPiAckJson(seq, "TURN", true, "accepted"));
    return;
  }

  sendJsonLine(buildPiAckJson(seq, "UNKNOWN", false, "unknown_command"));
}

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>Clean Mecanum Position</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0b0f14;
      --panel: #111923;
      --panel-2: #172331;
      --line: #2b3a4d;
      --text: #eef4fb;
      --muted: #9cb0c7;
      --accent: #2ec4a6;
      --danger: #d95d63;
      --warn: #f0b65a;
      --blue: #7bb7ff;
      --pink: #ff7ab6;
      --lime: #b7df58;
    }
    * { box-sizing: border-box; letter-spacing: 0; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Segoe UI, Tahoma, Arial, sans-serif;
    }
    main { max-width: 1180px; margin: 0 auto; padding: 20px 14px 28px; }
    h1 { margin: 0 0 4px; font-size: 28px; font-weight: 650; }
    h2 { margin: 0 0 12px; font-size: 17px; font-weight: 600; }
    p { margin: 0; color: var(--muted); line-height: 1.45; }
    .band {
      border: 1px solid var(--line);
      border-radius: 8px;
      background: var(--panel);
      padding: 14px;
      margin-top: 12px;
    }
    .controls {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
      gap: 10px;
      align-items: end;
    }
    label { display: grid; gap: 5px; color: var(--muted); font-size: 13px; min-width: 0; }
    input, select, button {
      width: 100%;
      min-width: 0;
      min-height: 38px;
      border-radius: 6px;
      border: 1px solid var(--line);
      background: #0d141c;
      color: var(--text);
      font: inherit;
      padding: 8px 10px;
    }
    button { cursor: pointer; font-weight: 600; }
    button.primary { background: var(--accent); border-color: var(--accent); color: #04110e; }
    button.stop { background: var(--danger); border-color: var(--danger); color: #fff; }
    button.flat { background: var(--panel-2); }
    .actions { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 10px; }
    .status {
      display: flex;
      flex-wrap: wrap;
      gap: 8px 16px;
      padding-top: 12px;
      color: var(--muted);
      font-size: 14px;
    }
    .status strong { color: var(--text); }
    .gain-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(170px, 1fr));
      gap: 10px;
      margin-top: 10px;
    }
    .gain-cell {
      min-width: 0;
      border: 1px solid var(--line);
      border-radius: 7px;
      background: var(--panel-2);
      padding: 10px;
    }
    .gain-cell strong {
      display: block;
      margin-bottom: 8px;
      font-size: 13px;
    }
    .gain-inputs {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .metric-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      gap: 10px;
    }
    .metric {
      border: 1px solid var(--line);
      border-radius: 7px;
      background: var(--panel-2);
      padding: 12px;
    }
    .metric span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 8px;
    }
    .metric strong {
      display: block;
      font-size: 24px;
      font-variant-numeric: tabular-nums;
    }
    .plot {
      border: 1px solid var(--line);
      border-radius: 7px;
      background: var(--panel-2);
      padding: 10px;
    }
    .plot-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 10px;
    }
    .plot-head {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      gap: 10px;
      margin-bottom: 8px;
    }
    .plot-head strong { font-size: 14px; font-weight: 600; }
    .plot-head span { color: var(--muted); font-size: 12px; text-align: right; }
    canvas {
      display: block;
      width: 100%;
      height: 340px;
      border-radius: 6px;
      background: #0d141c;
    }
    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      color: var(--muted);
      font-size: 12px;
      padding-top: 8px;
    }
    .key { display: inline-flex; align-items: center; gap: 5px; }
    .key::before {
      content: "";
      display: inline-block;
      width: 16px;
      height: 3px;
      border-radius: 99px;
      background: var(--c);
    }
    .table-wrap { overflow-x: auto; }
    table { width: 100%; min-width: 1040px; border-collapse: collapse; }
    th, td { border-bottom: 1px solid var(--line); padding: 9px 8px; text-align: right; font-variant-numeric: tabular-nums; }
    th { color: var(--muted); font-size: 12px; font-weight: 600; }
    td:first-child, th:first-child { text-align: left; }
    .note { color: var(--warn); font-size: 13px; padding-top: 10px; }
    @media (max-width: 760px) {
      main { padding-top: 14px; }
      h1 { font-size: 23px; }
      canvas { height: 260px; }
    }
  </style>
</head>
<body>
  <main>
    <h1>Clean Mecanum Position</h1>
    <p>Outer body-position commands generate velocity targets for the tuned inner wheel PI loops. Logical wheel order is BR, FR, BL, FL.</p>

    <section class="band">
      <h2>Body Velocity Command</h2>
      <div class="controls">
        <label>Forward RPM<input id="forward" type="number" min="-45" max="45" step="0.5" value="0" /></label>
        <label>Strafe RPM<input id="strafe" type="number" min="-45" max="45" step="0.5" value="0" /></label>
        <label>Rotate RPM<input id="rotate" type="number" min="-45" max="45" step="0.5" value="0" /></label>
        <label>Timed Test Seconds<input id="duration" type="number" min="0.25" max="120" step="0.25" value="3" /></label>
        <label>Strafe Sign
          <select id="strafeSign">
            <option value="1">Normal</option>
            <option value="-1">Invert</option>
          </select>
        </label>
        <label>Rotate Sign
          <select id="rotateSign">
            <option value="1">Normal</option>
            <option value="-1">Invert</option>
          </select>
        </label>
        <label>Odom Forward Sign
          <select id="odomForwardSign">
            <option value="1">Normal</option>
            <option value="-1">Invert</option>
          </select>
        </label>
        <label>Odom Strafe Sign
          <select id="odomStrafeSign">
            <option value="1">Normal</option>
            <option value="-1">Invert</option>
          </select>
        </label>
        <label>Odom Rotate Sign
          <select id="odomYawSign">
            <option value="1">Normal</option>
            <option value="-1">Invert</option>
          </select>
        </label>
      </div>
      <div class="gain-grid">
        <div class="gain-cell">
          <strong>BR PI</strong>
          <div class="gain-inputs">
            <label>Kp<input id="kp0" type="number" min="0" step="0.05" value="15" /></label>
            <label>Ki<input id="ki0" type="number" min="0" step="0.05" value="200" /></label>
          </div>
        </div>
        <div class="gain-cell">
          <strong>FR PI</strong>
          <div class="gain-inputs">
            <label>Kp<input id="kp1" type="number" min="0" step="0.05" value="15" /></label>
            <label>Ki<input id="ki1" type="number" min="0" step="0.05" value="200" /></label>
          </div>
        </div>
        <div class="gain-cell">
          <strong>BL PI</strong>
          <div class="gain-inputs">
            <label>Kp<input id="kp2" type="number" min="0" step="0.05" value="15" /></label>
            <label>Ki<input id="ki2" type="number" min="0" step="0.05" value="200" /></label>
          </div>
        </div>
        <div class="gain-cell">
          <strong>FL PI</strong>
          <div class="gain-inputs">
            <label>Kp<input id="kp3" type="number" min="0" step="0.05" value="15" /></label>
            <label>Ki<input id="ki3" type="number" min="0" step="0.05" value="200" /></label>
          </div>
        </div>
      </div>
      <div class="actions">
        <button class="flat" onclick="setBody(30,0,0)">Forward</button>
        <button class="flat" onclick="setBody(-30,0,0)">Backward</button>
        <button class="flat" onclick="setBody(0,30,0)">Strafe +</button>
        <button class="flat" onclick="setBody(0,-30,0)">Strafe -</button>
        <button class="flat" onclick="setBody(0,0,25)">Rotate +</button>
        <button class="flat" onclick="setBody(0,0,-25)">Rotate -</button>
        <button class="flat" onclick="setBody(0,0,0)">Zero Command</button>
        <button class="flat" onclick="applyConfig()">Apply</button>
        <button class="primary" onclick="startController()">Start</button>
        <button class="primary" onclick="startTimedTest()">Run Timed Test</button>
        <button class="stop" onclick="stopController()">Stop</button>
        <button class="flat" onclick="resetIntegral()">Reset Integral</button>
        <button class="flat" onclick="resetEncoders()">Reset Encoders</button>
        <button class="flat" onclick="resetOdometry()">Reset Odometry</button>
        <button class="flat" onclick="clearTimedPlot()">Clear Timed Plot</button>
      </div>
      <div class="status">
        <span>Status <strong id="runState">STOPPED</strong></span>
        <span>Timed run <strong id="timedState">IDLE</strong></span>
        <span>Loop dt <strong id="dt">0</strong> ms</span>
        <span>Limit <strong id="limit">0</strong> RPM</span>
        <span>Target scale <strong id="scale">1.00</strong></span>
        <span>CPR <strong id="cpr">0</strong></span>
        <span>RPM filter alpha <strong id="alpha">0</strong></span>
      </div>
      <div class="note">Manual velocity mode is still available for tuning. Position moves use the cascaded outer loop below.</div>
    </section>

    <section class="band">
      <h2>Position Move</h2>
      <div class="controls">
        <label>Forward Target cm<input id="moveForward" type="number" step="1" value="30" /></label>
        <label>Strafe Target cm<input id="moveStrafe" type="number" step="1" value="0" /></label>
        <label>Rotate Target deg<input id="moveYaw" type="number" step="1" value="0" /></label>
        <label>Position Kp rpm/cm<input id="positionKp" type="number" min="0" step="0.05" value="100" /></label>
        <label>Heading Kp rpm/deg<input id="yawKp" type="number" min="0" step="0.05" value="10" /></label>
        <label>Position Max RPM<input id="positionMaxRpm" type="number" min="1" max="45" step="1" value="40" /></label>
        <label>Position Tolerance cm<input id="positionToleranceCm" type="number" min="0.1" max="10" step="0.1" value="2.0" /></label>
        <label>Heading Tolerance deg<input id="yawToleranceDeg" type="number" min="0.5" max="15" step="0.5" value="2.0" /></label>
        <label>Forward Odom Scale<input id="odomForwardScale" type="number" min="0.5" max="1.5" step="0.0005" value="0.9854" /></label>
        <label>Strafe Odom Scale<input id="odomStrafeScale" type="number" min="0.5" max="1.5" step="0.0005" value="0.9375" /></label>
      </div>
      <div class="actions">
        <button class="primary" onclick="startPositionMove()">Start Position Move</button>
        <button class="flat" onclick="setMove(30,0,0)">Forward 30 cm</button>
        <button class="flat" onclick="setMove(-30,0,0)">Backward 30 cm</button>
        <button class="flat" onclick="setMove(0,30,0)">Right 30 cm</button>
        <button class="flat" onclick="setMove(0,-30,0)">Left 30 cm</button>
        <button class="flat" onclick="setMove(0,0,45)">CCW 45 deg</button>
        <button class="flat" onclick="setMove(0,0,-45)">CW 45 deg</button>
        <button class="flat" onclick="imuZero()">Zero IMU Heading</button>
        <button class="flat" onclick="imuCalibrate()">Calibrate IMU Bias</button>
      </div>
      <div class="status">
        <span>Position mode <strong id="positionMode">IDLE</strong></span>
        <span>Forward target <strong id="targetForward">0.00</strong> cm</span>
        <span>Strafe target <strong id="targetStrafe">0.00</strong> cm</span>
        <span>Heading target <strong id="targetYaw">0.00</strong> deg</span>
        <span>Heading source <strong id="headingSource">ENCODER</strong></span>
      </div>
      <div class="note">Position control is cascaded: body position Kp outside, wheel velocity PI inside. Heading adds internal gyro-rate damping near the target.</div>
    </section>

    <section class="band">
      <h2>Encoder Odometry</h2>
      <div class="metric-grid">
        <div class="metric"><span>Forward travel cm</span><strong id="odomForward">0.00</strong></div>
        <div class="metric"><span>Strafe travel cm</span><strong id="odomStrafe">0.00</strong></div>
        <div class="metric"><span>Heading deg</span><strong id="odomYaw">0.00</strong></div>
        <div class="metric"><span>Forward error cm</span><strong id="errorForward">0.00</strong></div>
        <div class="metric"><span>Strafe error cm</span><strong id="errorStrafe">0.00</strong></div>
        <div class="metric"><span>Heading error deg</span><strong id="errorYaw">0.00</strong></div>
      </div>
    </section>

    <section class="band">
      <h2>Heading Diagnostics</h2>
      <div class="metric-grid">
        <div class="metric"><span>Control yaw deg</span><strong id="imuYaw">0.00</strong></div>
        <div class="metric"><span>Raw yaw deg</span><strong id="imuYawRaw">0.00</strong></div>
        <div class="metric"><span>Gyro Z dps</span><strong id="imuGyro">0.00</strong></div>
        <div class="metric"><span>Filtered gyro Z dps</span><strong id="imuGyroFilt">0.00</strong></div>
        <div class="metric"><span>Gyro bias dps</span><strong id="imuBias">0.000</strong></div>
        <div class="metric"><span>Calibration std dps</span><strong id="imuCalStd">0.000</strong></div>
      </div>
    </section>

    <section class="band">
      <h2>Position Response</h2>
      <div class="plot-grid">
        <div class="plot">
          <div class="plot-head"><strong>Forward Position</strong><span id="posLiveMeta0">Waiting</span></div>
          <canvas id="posLive0"></canvas>
          <div class="legend"><span class="key" style="--c:#f0b65a">target</span><span class="key" style="--c:#2ec4a6">measured</span></div>
        </div>
        <div class="plot">
          <div class="plot-head"><strong>Strafe Position</strong><span id="posLiveMeta1">Waiting</span></div>
          <canvas id="posLive1"></canvas>
          <div class="legend"><span class="key" style="--c:#f0b65a">target</span><span class="key" style="--c:#7bb7ff">measured</span></div>
        </div>
        <div class="plot">
          <div class="plot-head"><strong>Heading</strong><span id="posLiveMeta2">Waiting</span></div>
          <canvas id="posLive2"></canvas>
          <div class="legend"><span class="key" style="--c:#f0b65a">target</span><span class="key" style="--c:#ff7ab6">measured</span></div>
        </div>
      </div>
    </section>

    <section class="band">
      <h2>Held Position Move</h2>
      <div class="plot-grid">
        <div class="plot">
          <div class="plot-head"><strong>Forward Capture</strong><span id="posHeldMeta0">No move captured</span></div>
          <canvas id="posHeld0"></canvas>
        </div>
        <div class="plot">
          <div class="plot-head"><strong>Strafe Capture</strong><span id="posHeldMeta1">No move captured</span></div>
          <canvas id="posHeld1"></canvas>
        </div>
        <div class="plot">
          <div class="plot-head"><strong>Heading Capture</strong><span id="posHeldMeta2">No move captured</span></div>
          <canvas id="posHeld2"></canvas>
        </div>
      </div>
    </section>

    <section class="band">
      <h2>Velocity Response</h2>
      <div class="plot">
        <div class="plot-head">
          <strong>Live All-Wheel Response</strong>
          <span id="liveMeta">Waiting for samples</span>
        </div>
        <canvas id="livePlot"></canvas>
        <div class="legend">
          <span class="key" style="--c:#2ec4a6">BR measured</span>
          <span class="key" style="--c:#7bb7ff">FR measured</span>
          <span class="key" style="--c:#ff7ab6">BL measured</span>
          <span class="key" style="--c:#b7df58">FL measured</span>
          <span class="key" style="--c:#f0b65a">targets</span>
        </div>
      </div>
    </section>

    <section class="band">
      <h2>Timed Test Capture</h2>
      <div class="plot">
        <div class="plot-head">
          <strong>Held Response</strong>
          <span id="timedMeta">No timed test captured</span>
        </div>
        <canvas id="timedPlot"></canvas>
        <div class="legend">
          <span class="key" style="--c:#2ec4a6">BR measured</span>
          <span class="key" style="--c:#7bb7ff">FR measured</span>
          <span class="key" style="--c:#ff7ab6">BL measured</span>
          <span class="key" style="--c:#b7df58">FL measured</span>
          <span class="key" style="--c:#f0b65a">targets</span>
        </div>
      </div>
    </section>

    <section class="band">
      <h2>Wheel Telemetry</h2>
      <div class="table-wrap">
        <table>
          <thead>
            <tr>
              <th>Wheel</th><th>Target</th><th>Raw RPM</th><th>Filtered RPM</th><th>Error</th>
              <th>P</th><th>I</th><th>Raw Out</th><th>PWM</th><th>HW PWM</th>
              <th>Integral</th><th>Signed Count</th><th>Raw Count</th><th>Enc Sign</th><th>Motor Sign</th>
            </tr>
          </thead>
          <tbody>
            <tr><td>BR</td><td id="tar0">0</td><td id="raw0">0</td><td id="rpm0">0</td><td id="err0">0</td><td id="p0">0</td><td id="i0">0</td><td id="u0">0</td><td id="pwm0">0</td><td id="hpwm0">0</td><td id="int0">0</td><td id="cnt0">0</td><td id="rawCnt0">0</td><td id="sgn0">0</td><td id="msgn0">0</td></tr>
            <tr><td>FR</td><td id="tar1">0</td><td id="raw1">0</td><td id="rpm1">0</td><td id="err1">0</td><td id="p1">0</td><td id="i1">0</td><td id="u1">0</td><td id="pwm1">0</td><td id="hpwm1">0</td><td id="int1">0</td><td id="cnt1">0</td><td id="rawCnt1">0</td><td id="sgn1">0</td><td id="msgn1">0</td></tr>
            <tr><td>BL</td><td id="tar2">0</td><td id="raw2">0</td><td id="rpm2">0</td><td id="err2">0</td><td id="p2">0</td><td id="i2">0</td><td id="u2">0</td><td id="pwm2">0</td><td id="hpwm2">0</td><td id="int2">0</td><td id="cnt2">0</td><td id="rawCnt2">0</td><td id="sgn2">0</td><td id="msgn2">0</td></tr>
            <tr><td>FL</td><td id="tar3">0</td><td id="raw3">0</td><td id="rpm3">0</td><td id="err3">0</td><td id="p3">0</td><td id="i3">0</td><td id="u3">0</td><td id="pwm3">0</td><td id="hpwm3">0</td><td id="int3">0</td><td id="cnt3">0</td><td id="rawCnt3">0</td><td id="sgn3">0</td><td id="msgn3">0</td></tr>
          </tbody>
        </table>
      </div>
    </section>
  </main>
  <script>
    const field = (id) => document.getElementById(id);
    const fmt = (value, digits = 2) => Number(value || 0).toFixed(digits);
    const colors = ['#2ec4a6', '#7bb7ff', '#ff7ab6', '#b7df58'];
    const names = ['BR', 'FR', 'BL', 'FL'];
    const liveWindowMs = 20000;
    let formLoaded = false;
    let latestData = null;
    let liveHistory = [];
    let timedHistory = [];
    let positionLiveHistory = [];
    let positionHeldHistory = [];
    let timedRunId = -1;
    let lastTimedSampleMs = -1;
    let positionMoveId = -1;
    let lastPositionSampleMs = -1;

    async function call(path) {
      const response = await fetch(path);
      if (!response.ok) throw new Error(await response.text());
      return response.text();
    }

    function clampRpm(value, limit = 45) {
      return Math.min(limit, Math.max(-limit, Number(value || 0)));
    }

    function setBody(forward, strafe, rotate) {
      field('forward').value = forward;
      field('strafe').value = strafe;
      field('rotate').value = rotate;
    }

    function setMove(forward, strafe, yaw) {
      field('moveForward').value = forward;
      field('moveStrafe').value = strafe;
      field('moveYaw').value = yaw;
    }

    function configQuery() {
      const params = new URLSearchParams({
        forward: field('forward').value,
        strafe: field('strafe').value,
        rotate: field('rotate').value,
        strafeSign: field('strafeSign').value,
        rotateSign: field('rotateSign').value,
        odomForwardSign: field('odomForwardSign').value,
        odomStrafeSign: field('odomStrafeSign').value,
        odomYawSign: field('odomYawSign').value,
        positionKp: field('positionKp').value,
        yawKp: field('yawKp').value,
        positionMaxRpm: field('positionMaxRpm').value,
        positionToleranceCm: field('positionToleranceCm').value,
        yawToleranceDeg: field('yawToleranceDeg').value,
        odomForwardScale: field('odomForwardScale').value,
        odomStrafeScale: field('odomStrafeScale').value
      });
      for (let i = 0; i < 4; i++) {
        params.set('kp' + i, field('kp' + i).value);
        params.set('ki' + i, field('ki' + i).value);
      }
      return params.toString();
    }

    function timedRunMs() {
      const seconds = Math.min(120, Math.max(0.25, Number(field('duration').value || 3)));
      field('duration').value = seconds;
      return Math.round(seconds * 1000);
    }

    async function applyConfig() {
      try {
        await call('/cmd/config?' + configQuery());
        await refresh();
      } catch (error) {
        alert(error.message);
      }
    }

    async function startPositionMove() {
      try {
        const params = configQuery()
          + '&targetForwardCm=' + encodeURIComponent(field('moveForward').value)
          + '&targetStrafeCm=' + encodeURIComponent(field('moveStrafe').value)
          + '&targetYawDeg=' + encodeURIComponent(field('moveYaw').value);
        await call('/cmd/positionStart?' + params);
        await refresh();
      } catch (error) {
        alert(error.message);
      }
    }

    async function startController() {
      try {
        await call('/cmd/start?' + configQuery());
        await refresh();
      } catch (error) {
        alert(error.message);
      }
    }

    async function startTimedTest() {
      try {
        await call('/cmd/timedStart?' + configQuery() + '&ms=' + timedRunMs());
        await refresh();
      } catch (error) {
        alert(error.message);
      }
    }

    async function stopController() { await call('/cmd/stop'); await refresh(); }
    async function resetIntegral() { await call('/cmd/resetI'); await refresh(); }
    async function resetEncoders() { await call('/cmd/resetEnc'); await refresh(); }
    async function resetOdometry() { await call('/cmd/resetOdom'); await refresh(); }
    async function imuZero() { await call('/cmd/imuZero'); await refresh(); }
    async function imuCalibrate() { await call('/cmd/imuCal'); await refresh(); }

    function clearTimedPlot() {
      timedHistory = [];
      lastTimedSampleMs = -1;
      drawPlots();
    }

    function appendLiveSample(data) {
      const sample = {
        t: Number(data.nowMs),
        target: data.appliedTargetRpm.map(Number),
        rpm: data.rpm.map(Number)
      };
      if (!liveHistory.length || sample.t !== liveHistory[liveHistory.length - 1].t) {
        liveHistory.push(sample);
      }
      const oldest = sample.t - liveWindowMs;
      while (liveHistory.length && liveHistory[0].t < oldest) liveHistory.shift();
    }

    function appendTimedSample(t, target, rpm) {
      if (!Number.isFinite(t) || t < 0 || t === lastTimedSampleMs) return;
      timedHistory.push({ t, target: target.map(Number), rpm: rpm.map(Number) });
      lastTimedSampleMs = t;
    }

    function captureTimedSample(data) {
      if (Number(data.timed.id) !== timedRunId) {
        timedRunId = Number(data.timed.id);
        if (timedRunId > 0) {
          timedHistory = [];
          lastTimedSampleMs = -1;
        }
      }
      if (timedRunId <= 0) return;
      const elapsed = Number(data.timed.elapsedMs);
      if (data.timed.active) {
        appendTimedSample(elapsed, data.appliedTargetRpm, data.rpm);
      } else if (timedHistory.length && elapsed > lastTimedSampleMs && elapsed <= Number(data.timed.durationMs)) {
        appendTimedSample(elapsed, timedHistory[timedHistory.length - 1].target, data.rpm);
      }
    }

    function positionSample(data) {
      return {
        t: Number(data.nowMs),
        moveId: Number(data.positionMoveId || 0),
        target: [
          Number(data.position.targetForwardCm),
          Number(data.position.targetStrafeCm),
          Number(data.position.targetYawDeg)
        ],
        measured: [
          Number(data.odometry.forwardCm),
          Number(data.odometry.strafeCm),
          Number(data.odometry.yawDeg)
        ],
        error: [
          Number(data.position.errorForwardCm),
          Number(data.position.errorStrafeCm),
          Number(data.position.errorYawDeg)
        ]
      };
    }

    function capturePositionSample(data) {
      const sample = positionSample(data);
      if (!positionLiveHistory.length || sample.t !== positionLiveHistory[positionLiveHistory.length - 1].t) {
        positionLiveHistory.push(sample);
      }

      const oldest = sample.t - liveWindowMs;
      while (positionLiveHistory.length && positionLiveHistory[0].t < oldest) positionLiveHistory.shift();

      if (sample.moveId !== positionMoveId) {
        positionMoveId = sample.moveId;
        if (positionMoveId > 0) {
          positionHeldHistory = [];
          lastPositionSampleMs = -1;
        }
      }

      if (positionMoveId <= 0) return;
      const lastHeld = positionHeldHistory[positionHeldHistory.length - 1];
      const shouldHold = data.positionModeActive || (positionHeldHistory.length && !lastHeld.done);
      if (shouldHold) {
        const relativeT = positionHeldHistory.length ? sample.t - positionHeldHistory[0].absoluteT : 0;
        if (sample.t !== lastPositionSampleMs) {
          positionHeldHistory.push({ ...sample, absoluteT: sample.t, t: Math.max(0, relativeT), done: !data.positionModeActive });
          lastPositionSampleMs = sample.t;
        }
      }
    }

    function sizeCanvas(canvas) {
      const rect = canvas.getBoundingClientRect();
      const scale = window.devicePixelRatio || 1;
      const width = Math.max(1, Math.floor(rect.width));
      const height = Math.max(1, Math.floor(rect.height));
      const pixelWidth = Math.floor(width * scale);
      const pixelHeight = Math.floor(height * scale);
      if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
        canvas.width = pixelWidth;
        canvas.height = pixelHeight;
      }
      const ctx = canvas.getContext('2d');
      ctx.setTransform(scale, 0, 0, scale, 0, 0);
      return { ctx, width, height };
    }

    function plotLine(ctx, samples, xOf, yOf, source, wheel, color, width, dash) {
      let started = false;
      ctx.beginPath();
      for (const sample of samples) {
        const arr = sample[source];
        if (!arr) continue;
        const value = Number(arr[wheel]);
        if (!Number.isFinite(sample.t) || !Number.isFinite(value)) continue;
        const x = xOf(sample.t);
        const y = yOf(value);
        if (!started) {
          ctx.moveTo(x, y);
          started = true;
        } else {
          ctx.lineTo(x, y);
        }
      }
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.setLineDash(dash || []);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    function drawPlot(canvas, samples, options) {
      const { ctx, width, height } = sizeCanvas(canvas);
      const left = 48;
      const right = 12;
      const top = 16;
      const bottom = 30;
      const plotWidth = Math.max(1, width - left - right);
      const plotHeight = Math.max(1, height - top - bottom);

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = '#0d141c';
      ctx.fillRect(0, 0, width, height);

      if (!samples.length) {
        ctx.fillStyle = '#9cb0c7';
        ctx.font = '13px Segoe UI, Arial, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(options.empty, width / 2, height / 2);
        return;
      }

      const latestT = samples[samples.length - 1].t;
      const xMin = options.live ? Math.max(0, latestT - liveWindowMs) : 0;
      const xMax = options.live ? Math.max(xMin + 1000, latestT) : Math.max(1000, Number(options.durationMs || 0), latestT);
      const visible = samples.filter((sample) => sample.t >= xMin && sample.t <= xMax);

      let maxAbs = 10;
      for (const sample of visible) {
        for (let i = 0; i < 4; i++) {
          maxAbs = Math.max(maxAbs, Math.abs(Number(sample.target[i]) || 0), Math.abs(Number(sample.rpm[i]) || 0));
        }
      }
      const yLimit = Math.ceil((maxAbs * 1.15) / 5) * 5;
      const xOf = (value) => left + ((value - xMin) / (xMax - xMin)) * plotWidth;
      const yOf = (value) => top + ((yLimit - value) / (2 * yLimit)) * plotHeight;

      ctx.font = '12px Segoe UI, Arial, sans-serif';
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      ctx.textAlign = 'right';
      for (let i = 0; i <= 4; i++) {
        const value = yLimit - (i * yLimit) / 2;
        const y = yOf(value);
        ctx.strokeStyle = value === 0 ? '#51657d' : '#2b3a4d';
        ctx.beginPath();
        ctx.moveTo(left, y);
        ctx.lineTo(width - right, y);
        ctx.stroke();
        ctx.fillStyle = '#9cb0c7';
        ctx.fillText(fmt(value, 0), left - 7, y);
      }

      ctx.textBaseline = 'top';
      ctx.textAlign = 'center';
      for (let i = 0; i <= 4; i++) {
        const t = xMin + ((xMax - xMin) * i) / 4;
        const x = xOf(t);
        ctx.strokeStyle = '#2b3a4d';
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, height - bottom);
        ctx.stroke();
        const seconds = options.live ? (t - xMax) / 1000 : t / 1000;
        ctx.fillStyle = '#9cb0c7';
        ctx.fillText((seconds > 0 ? '+' : '') + fmt(seconds, options.live ? 0 : 1) + 's', x, height - bottom + 7);
      }

      ctx.textAlign = 'left';
      ctx.fillStyle = '#9cb0c7';
      ctx.fillText('RPM', 8, 4);
      for (let i = 0; i < 4; i++) plotLine(ctx, visible, xOf, yOf, 'target', i, '#f0b65a', 1, [5, 5]);
      for (let i = 0; i < 4; i++) plotLine(ctx, visible, xOf, yOf, 'rpm', i, colors[i], 2, []);
    }

    function drawPositionPlot(canvas, samples, axis, options) {
      const { ctx, width, height } = sizeCanvas(canvas);
      const left = 48;
      const right = 12;
      const top = 16;
      const bottom = 30;
      const plotWidth = Math.max(1, width - left - right);
      const plotHeight = Math.max(1, height - top - bottom);

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = '#0d141c';
      ctx.fillRect(0, 0, width, height);

      if (!samples.length) {
        ctx.fillStyle = '#9cb0c7';
        ctx.font = '13px Segoe UI, Arial, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(options.empty, width / 2, height / 2);
        return;
      }

      const latestT = samples[samples.length - 1].t;
      const xMin = options.live ? Math.max(0, latestT - liveWindowMs) : 0;
      const xMax = options.live ? Math.max(xMin + 1000, latestT) : Math.max(1000, latestT);
      const visible = samples.filter((sample) => sample.t >= xMin && sample.t <= xMax);

      let minValue = 0;
      let maxValue = 0;
      for (const sample of visible) {
        minValue = Math.min(minValue, Number(sample.target[axis]) || 0, Number(sample.measured[axis]) || 0);
        maxValue = Math.max(maxValue, Number(sample.target[axis]) || 0, Number(sample.measured[axis]) || 0);
      }
      const span = Math.max(5, maxValue - minValue);
      const yMin = Math.floor((minValue - span * 0.15) / 5) * 5;
      const yMax = Math.ceil((maxValue + span * 0.15) / 5) * 5;
      const xOf = (value) => left + ((value - xMin) / (xMax - xMin)) * plotWidth;
      const yOf = (value) => top + ((yMax - value) / Math.max(0.001, yMax - yMin)) * plotHeight;

      ctx.font = '12px Segoe UI, Arial, sans-serif';
      ctx.lineWidth = 1;
      ctx.textBaseline = 'middle';
      ctx.textAlign = 'right';
      for (let i = 0; i <= 4; i++) {
        const value = yMax - ((yMax - yMin) * i) / 4;
        const y = yOf(value);
        ctx.strokeStyle = Math.abs(value) < 0.0001 ? '#51657d' : '#2b3a4d';
        ctx.beginPath();
        ctx.moveTo(left, y);
        ctx.lineTo(width - right, y);
        ctx.stroke();
        ctx.fillStyle = '#9cb0c7';
        ctx.fillText(fmt(value, 0), left - 7, y);
      }

      ctx.textBaseline = 'top';
      ctx.textAlign = 'center';
      for (let i = 0; i <= 4; i++) {
        const t = xMin + ((xMax - xMin) * i) / 4;
        const x = xOf(t);
        ctx.strokeStyle = '#2b3a4d';
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, height - bottom);
        ctx.stroke();
        const seconds = options.live ? (t - xMax) / 1000 : t / 1000;
        ctx.fillStyle = '#9cb0c7';
        ctx.fillText((seconds > 0 ? '+' : '') + fmt(seconds, options.live ? 0 : 1) + 's', x, height - bottom + 7);
      }

      const drawSeries = (source, color, width, dash) => {
        let started = false;
        ctx.beginPath();
        for (const sample of visible) {
          const value = Number(sample[source][axis]);
          if (!Number.isFinite(sample.t) || !Number.isFinite(value)) continue;
          const x = xOf(sample.t);
          const y = yOf(value);
          if (!started) {
            ctx.moveTo(x, y);
            started = true;
          } else {
            ctx.lineTo(x, y);
          }
        }
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.setLineDash(dash || []);
        ctx.stroke();
        ctx.setLineDash([]);
      };

      ctx.textAlign = 'left';
      ctx.fillStyle = '#9cb0c7';
      ctx.fillText(options.unit, 8, 4);
      drawSeries('target', '#f0b65a', 1, [5, 5]);
      drawSeries('measured', colors[axis], 2, []);
    }

    function drawPlots() {
      field('liveMeta').textContent = liveHistory.length
        ? 'Last ' + fmt(liveWindowMs / 1000, 0) + ' s, ' + liveHistory.length + ' samples'
        : 'Waiting for samples';
      field('timedMeta').textContent = timedHistory.length
        ? 'Test #' + timedRunId + ', ' + fmt(timedHistory[timedHistory.length - 1].t / 1000, 2) + ' s captured'
        : 'No timed test captured';
      drawPlot(field('livePlot'), liveHistory, { live: true, empty: 'Live four-wheel RPM response will appear here.' });
      drawPlot(field('timedPlot'), timedHistory, {
        live: false,
        durationMs: latestData ? latestData.timed.durationMs : timedRunMs(),
        empty: 'Run a timed test to hold a four-wheel response plot here.'
      });
      const units = ['cm', 'cm', 'deg'];
      for (let axis = 0; axis < 3; axis++) {
        field('posLiveMeta' + axis).textContent = positionLiveHistory.length
          ? 'Last ' + fmt(liveWindowMs / 1000, 0) + ' s, ' + positionLiveHistory.length + ' samples'
          : 'Waiting';
        field('posHeldMeta' + axis).textContent = positionHeldHistory.length
          ? 'Move #' + positionMoveId + ', ' + fmt(positionHeldHistory[positionHeldHistory.length - 1].t / 1000, 2) + ' s captured'
          : 'No move captured';
        drawPositionPlot(field('posLive' + axis), positionLiveHistory, axis, {
          live: true,
          unit: units[axis],
          empty: 'Live position response will appear here.'
        });
        drawPositionPlot(field('posHeld' + axis), positionHeldHistory, axis, {
          live: false,
          unit: units[axis],
          empty: 'Run a position move to hold this plot.'
        });
      }
    }

    async function refresh() {
      const response = await fetch('/data');
      if (!response.ok) return;
      const data = await response.json();
      latestData = data;

      if (!formLoaded) {
        field('forward').value = data.command.forwardRpm;
        field('strafe').value = data.command.strafeRpm;
        field('rotate').value = data.command.rotateRpm;
        field('strafeSign').value = String(data.command.strafeSign);
        field('rotateSign').value = String(data.command.rotateSign);
        field('odomForwardSign').value = String(data.command.odomForwardSign);
        field('odomStrafeSign').value = String(data.command.odomStrafeSign);
        field('odomYawSign').value = String(data.command.odomYawSign);
        field('positionKp').value = data.position.kp;
        field('yawKp').value = data.position.yawKp;
        field('positionMaxRpm').value = data.position.maxRpm;
        field('positionToleranceCm').value = data.position.toleranceCm;
        field('yawToleranceDeg').value = data.position.yawToleranceDeg;
        field('odomForwardScale').value = data.command.odomForwardScale;
        field('odomStrafeScale').value = data.command.odomStrafeScale;
        for (let i = 0; i < 4; i++) {
          field('kp' + i).value = data.kp[i];
          field('ki' + i).value = data.ki[i];
        }
        field('duration').value = fmt(Number(data.timed.durationMs) / 1000, 2);
        formLoaded = true;
      }

      field('runState').textContent = data.enabled ? 'RUNNING' : 'STOPPED';
      field('timedState').textContent = data.timed.active
        ? 'CAPTURING ' + fmt(data.timed.elapsedMs / 1000, 2) + ' / ' + fmt(data.timed.durationMs / 1000, 2) + ' s'
        : (data.timed.id > 0 ? 'HELD TEST #' + data.timed.id : 'IDLE');
      field('dt').textContent = fmt(data.loopDtMs, 2);
      field('limit').textContent = fmt(data.rpmLimit, 1);
      field('scale').textContent = fmt(data.command.targetScale, 3);
      field('cpr').textContent = fmt(data.countsPerRev, 1);
      field('alpha').textContent = fmt(data.rpmFilterAlpha, 2);
      field('odomForward').textContent = fmt(data.odometry.forwardCm, 2);
      field('odomStrafe').textContent = fmt(data.odometry.strafeCm, 2);
      field('odomYaw').textContent = fmt(data.odometry.yawDeg, 2);
      field('positionMode').textContent = data.positionModeActive ? 'MOVING' : (data.positionTargetReached ? 'REACHED' : 'IDLE');
      field('targetForward').textContent = fmt(data.position.targetForwardCm, 2);
      field('targetStrafe').textContent = fmt(data.position.targetStrafeCm, 2);
      field('targetYaw').textContent = fmt(data.position.targetYawDeg, 2);
      field('errorForward').textContent = fmt(data.position.errorForwardCm, 2);
      field('errorStrafe').textContent = fmt(data.position.errorStrafeCm, 2);
      field('errorYaw').textContent = fmt(data.position.errorYawDeg, 2);
      field('headingSource').textContent = data.imu.headingUsesImu
        ? 'IMU FILTERED'
        : (data.imu.ok ? 'IMU CHECK CAL' : 'ENCODER');
      field('imuYaw').textContent = fmt(data.imu.yawDeg, 2);
      field('imuYawRaw').textContent = fmt(data.imu.yawRawDeg, 2);
      field('imuGyro').textContent = fmt(data.imu.gyroDps, 2);
      field('imuGyroFilt').textContent = fmt(data.imu.gyroFiltDps, 2);
      field('imuBias').textContent = fmt(data.imu.biasDps, 3);
      field('imuCalStd').textContent = fmt(data.imu.calStdDps, 3);

      for (let i = 0; i < 4; i++) {
        field('tar' + i).textContent = fmt(data.appliedTargetRpm[i], 2);
        field('raw' + i).textContent = fmt(data.rawRpm[i], 2);
        field('rpm' + i).textContent = fmt(data.rpm[i], 2);
        field('err' + i).textContent = fmt(data.errorRpm[i], 2);
        field('p' + i).textContent = fmt(data.pTerm[i], 1);
        field('i' + i).textContent = fmt(data.iTerm[i], 1);
        field('u' + i).textContent = fmt(data.rawOutput[i], 1);
        field('pwm' + i).textContent = data.pwm[i];
        field('hpwm' + i).textContent = data.hardwarePwm[i];
        field('int' + i).textContent = fmt(data.integralError[i], 2);
        field('cnt' + i).textContent = data.signedEncoderCounts[i];
        field('rawCnt' + i).textContent = data.rawEncoderCounts[i];
        field('sgn' + i).textContent = data.encoderSign[i] > 0 ? '+1' : '-1';
        field('msgn' + i).textContent = data.motorSign[i] > 0 ? '+1' : '-1';
      }

      appendLiveSample(data);
      captureTimedSample(data);
      capturePositionSample(data);
      drawPlots();
    }

    window.addEventListener('resize', drawPlots);
    setInterval(refresh, 100);
    refresh();
  </script>
</body>
</html>
)HTML";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  server.send(200, "application/json", telemetryJson());
}

bool applyConfigArgs() {
  bool gainsChanged = false;

  if (server.hasArg("forward")) {
    commandForwardRpm = constrain(server.arg("forward").toFloat(), -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT);
  }

  if (server.hasArg("strafe")) {
    commandStrafeRpm = constrain(server.arg("strafe").toFloat(), -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT);
  }

  if (server.hasArg("rotate")) {
    commandRotateRpm = constrain(server.arg("rotate").toFloat(), -RPM_TARGET_LIMIT, RPM_TARGET_LIMIT);
  }

  if (server.hasArg("strafeSign")) {
    strafeSign = server.arg("strafeSign").toInt() >= 0 ? 1 : -1;
  }

  if (server.hasArg("rotateSign")) {
    rotateSign = server.arg("rotateSign").toInt() >= 0 ? 1 : -1;
  }

  if (server.hasArg("odomForwardSign")) {
    odomForwardSign = server.arg("odomForwardSign").toInt() >= 0 ? 1 : -1;
  }

  if (server.hasArg("odomStrafeSign")) {
    odomStrafeSign = server.arg("odomStrafeSign").toInt() >= 0 ? 1 : -1;
  }

  if (server.hasArg("odomYawSign")) {
    odomYawSign = server.arg("odomYawSign").toInt() >= 0 ? 1 : -1;
  }

  if (server.hasArg("odomForwardScale")) {
    float value = server.arg("odomForwardScale").toFloat();
    if (value < 0.5f || value > 1.5f) {
      return false;
    }
    odomForwardScale = value;
  }

  if (server.hasArg("odomStrafeScale")) {
    float value = server.arg("odomStrafeScale").toFloat();
    if (value < 0.5f || value > 1.5f) {
      return false;
    }
    odomStrafeScale = value;
  }

  if (server.hasArg("positionKp")) {
    float value = server.arg("positionKp").toFloat();
    if (value < 0.0f) {
      return false;
    }
    positionKp = value;
  }

  if (server.hasArg("yawKp")) {
    float value = server.arg("yawKp").toFloat();
    if (value < 0.0f) {
      return false;
    }
    yawKp = value;
  }

  if (server.hasArg("positionMaxRpm")) {
    float value = server.arg("positionMaxRpm").toFloat();
    if (value <= 0.0f || value > RPM_TARGET_LIMIT) {
      return false;
    }
    positionMaxRpm = value;
  }

  if (server.hasArg("positionToleranceCm")) {
    float value = server.arg("positionToleranceCm").toFloat();
    if (value < 0.1f || value > 10.0f) {
      return false;
    }
    positionToleranceCm = value;
  }

  if (server.hasArg("yawToleranceDeg")) {
    float value = server.arg("yawToleranceDeg").toFloat();
    if (value < 0.5f || value > 15.0f) {
      return false;
    }
    yawToleranceDeg = value;
  }

  for (uint8_t wheel = 0; wheel < 4; wheel++) {
    String kpName = "kp" + String(wheel);
    String kiName = "ki" + String(wheel);

    if (server.hasArg(kpName)) {
      float value = server.arg(kpName).toFloat();
      if (value < 0.0f) {
        return false;
      }
      gainsChanged = gainsChanged || fabsf(velocityKp[wheel] - value) > 0.0001f;
      velocityKp[wheel] = value;
    }

    if (server.hasArg(kiName)) {
      float value = server.arg(kiName).toFloat();
      if (value < 0.0f) {
        return false;
      }
      gainsChanged = gainsChanged || fabsf(velocityKi[wheel] - value) > 0.0001f;
      velocityKi[wheel] = value;
    }
  }

  if (gainsChanged) {
    resetAllPiStates();
  }
  updateMecanumTargets();
  return true;
}

void handleConfig() {
  STATE_LOCK();
  bool ok = applyConfigArgs();
  STATE_UNLOCK();
  server.send(ok ? 200 : 400, "text/plain", ok ? "Config applied" : "Invalid config");
}

void handleStart() {
  STATE_LOCK();
  bool ok = applyConfigArgs();
  if (ok) {
    positionModeActive = false;
    positionTargetReached = false;
    resetAllPiStates();
    resetAllBodyPiStates();
    resetRpmMeasurementsToCurrentCounts();
    timedRunActive = false;
    timedRunStartMs = 0;
    controllerEnabled = true;
  }
  STATE_UNLOCK();
  server.send(ok ? 200 : 400, "text/plain", ok ? "Four-wheel velocity PI started" : "Invalid config");
}

void handlePositionStart() {
  STATE_LOCK();
  bool ok = applyConfigArgs();

  if (ok && server.hasArg("targetForwardCm")) {
    targetForwardCm = server.arg("targetForwardCm").toFloat();
  }

  if (ok && server.hasArg("targetStrafeCm")) {
    targetStrafeCm = server.arg("targetStrafeCm").toFloat();
  }

  float requestedYawDeg = 0.0f;
  if (ok && server.hasArg("targetYawDeg")) {
    requestedYawDeg = server.arg("targetYawDeg").toFloat();
  }

  if (ok) {
    positionMoveId++;
    if (positionMoveId == 0) positionMoveId = 1;
    float startYawDeg = displayedOdomYawDeg();
    targetYawDeg = wrapAngleDeg(startYawDeg + requestedYawDeg);
    resetOdometryToCurrentCounts();
    resetRpmMeasurementsToCurrentCounts();
    resetAllPiStates();
    resetAllBodyPiStates();
    positionErrorForwardCm = targetForwardCm;
    positionErrorStrafeCm = targetStrafeCm;
    positionErrorYawDeg = shortestAngleErrorDeg(targetYawDeg, displayedOdomYawDeg());
    positionTargetReached = false;
    positionModeActive = true;
    timedRunActive = false;
    timedRunStartMs = 0;
    controllerEnabled = true;
    updatePositionController(NOMINAL_DT_S);
  }

  STATE_UNLOCK();
  server.send(ok ? 200 : 400, "text/plain", ok ? "Position move started" : "Invalid position config");
}

void handleTimedStart() {
  STATE_LOCK();
  bool ok = applyConfigArgs();
  uint32_t durationMs = timedRunDurationMs;

  if (ok && server.hasArg("ms")) {
    long value = server.arg("ms").toInt();
    if (value < (long)TIMED_RUN_MIN_MS || value > (long)TIMED_RUN_MAX_MS) {
      ok = false;
    } else {
      durationMs = (uint32_t)value;
    }
  }

  if (ok) {
    positionModeActive = false;
    positionTargetReached = false;
    timedRunDurationMs = durationMs;
    timedRunStartMs = millis();
    timedRunId++;
    if (timedRunId == 0) timedRunId = 1;
    timedRunActive = true;
    controllerEnabled = true;
    resetAllPiStates();
    resetAllBodyPiStates();
    resetRpmMeasurementsToCurrentCounts();
  }
  STATE_UNLOCK();
  server.send(ok ? 200 : 400, "text/plain", ok ? "Timed four-wheel velocity test started" : "Invalid timed test config");
}

void handleStop() {
  STATE_LOCK();
  controllerEnabled = false;
  positionModeActive = false;
  timedRunActive = false;
  timedRunStartMs = 0;
  resetAllPiStates();
  resetAllBodyPiStates();
  stopAllMotors();
  STATE_UNLOCK();
  server.send(200, "text/plain", "Four-wheel velocity PI stopped");
}

void handleResetIntegral() {
  STATE_LOCK();
  resetAllPiStates();
  resetAllBodyPiStates();
  STATE_UNLOCK();
  server.send(200, "text/plain", "PI integrals reset");
}

void handleResetEncoders() {
  STATE_LOCK();
  resetEncoderMeasurements();
  resetOdometryToCurrentCounts();
  resetAllPiStates();
  resetAllBodyPiStates();
  STATE_UNLOCK();
  server.send(200, "text/plain", "Encoders reset");
}

void handleResetOdometry() {
  STATE_LOCK();
  resetOdometryToCurrentCounts();
  positionErrorForwardCm = targetForwardCm;
  positionErrorStrafeCm = targetStrafeCm;
  positionErrorYawDeg = shortestAngleErrorDeg(targetYawDeg, displayedOdomYawDeg());
  resetAllBodyPiStates();
  STATE_UNLOCK();
  server.send(200, "text/plain", "Odometry reset");
}

void handleImuZero() {
  STATE_LOCK();
  controllerEnabled = false;
  positionModeActive = false;
  timedRunActive = false;
  timedRunStartMs = 0;
  resetAllPiStates();
  resetAllBodyPiStates();
  stopAllMotors();
  imuYawDeg = 0.0f;
  imuYawRawDeg = 0.0f;
  targetYawDeg = 0.0f;
  positionErrorYawDeg = 0.0f;
  STATE_UNLOCK();
  server.send(200, "text/plain", "IMU heading zeroed");
}

void handleImuCal() {
  STATE_LOCK();
  controllerEnabled = false;
  positionModeActive = false;
  timedRunActive = false;
  timedRunStartMs = 0;
  resetAllPiStates();
  resetAllBodyPiStates();
  stopAllMotors();
  bool ok = imuCalibrateBias(500);
  if (ok) {
    imuYawDeg = 0.0f;
    imuYawRawDeg = 0.0f;
    targetYawDeg = 0.0f;
    positionErrorYawDeg = 0.0f;
  }
  STATE_UNLOCK();
  server.send(ok ? 200 : 400, "text/plain", ok ? "IMU bias calibrated" : "IMU calibration failed: keep robot completely still");
}

void controlTask(void* arg) {
  (void)arg;
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastUs = micros();

  for (;;) {
    uint32_t nowUs = micros();
    float dt = ((uint32_t)(nowUs - lastUs)) / 1000000.0f;
    lastUs = nowUs;
    if (dt < 0.001f || dt > 0.200f) {
      dt = NOMINAL_DT_S;
    }

    STATE_LOCK();
    runControlTick(dt);
    STATE_UNLOCK();

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_INTERVAL_MS));
  }
}

void webTask(void* arg) {
  (void)arg;
  for (;;) {
    server.handleClient();
    vTaskDelay(1);
  }
}

void uartTask(void* arg) {
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
        handlePiCommandLine(String(lineBuf));
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

void piTelemetryTask(void* arg) {
  (void)arg;
  uint32_t lastTelemetryMs = 0;

  for (;;) {
    String telemetry;
    String done;

    STATE_LOCK();
    uint32_t now = millis();
    if (now - lastTelemetryMs >= PI_TELEMETRY_INTERVAL_MS) {
      telemetry = buildPiTelemetryJson();
      lastTelemetryMs = now;
    }

    if (piActiveSeq != 0 && piMoveActive && !positionModeActive && positionTargetReached) {
      latchPiMoveOdom();
      queuePiDone(piActiveSeq, 1);
      piActiveSeq = 0;
    }

    if (piPendingDone) {
      done = buildPiDoneJson(piPendingDoneSeq, piPendingDoneCode);
      piPendingDone = false;
    }
    STATE_UNLOCK();

    if (telemetry.length() > 0) {
      sendJsonLine(telemetry);
    }
    if (done.length() > 0) {
      sendJsonLine(done);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


String buildMicroRosTelemetryJson() {
  String s;
  s.reserve(420);
  float localForward = displayedOdomForwardCm();
  float localStrafe = displayedOdomStrafeCm();
  float localProgress = sqrtf(localForward * localForward + localStrafe * localStrafe);
  float localRemaining = sqrtf(positionErrorForwardCm * positionErrorForwardCm + positionErrorStrafeCm * positionErrorStrafeCm);

  s += "{\"type\":\"telemetry\"";
  s += ",\"mode\":\"" + String(piModeText()) + "\"";
  s += ",\"seq\":" + String(piActiveSeq);
  s += ",\"imu\":{\"ok\":" + String(imuHeadingReady() ? "true" : "false");
  s += ",\"yawDeg\":" + String(displayedOdomYawDeg(), 3);
  s += ",\"rawYawDeg\":" + String(displayedImuYawRawDeg(), 3);
  s += ",\"gyroDps\":" + String(displayedImuGyroDps(), 3);
  s += ",\"gyroFiltDps\":" + String(displayedImuGyroDpsFilt(), 3);
  s += "}";
  s += ",\"pose\":{\"forwardCm\":" + String(piPoseForwardCm(), 2);
  s += ",\"strafeCm\":" + String(piPoseStrafeCm(), 2);
  s += ",\"yawDeg\":" + String(displayedOdomYawDeg(), 2);
  s += ",\"progressCm\":" + String(localProgress, 2);
  s += ",\"remainingCm\":" + String(localRemaining, 2);
  s += ",\"localForwardCm\":" + String(localForward, 2);
  s += ",\"localStrafeCm\":" + String(localStrafe, 2);
  s += "}";
  s += ",\"move\":{\"phase\":" + String(positionModeActive ? 2 : 0);
  s += ",\"angleDeg\":" + String(piLastMoveAngleDeg, 2);
  s += ",\"distanceCm\":" + String(piLastMoveDistanceCm, 2);
  s += ",\"headingTargetDeg\":" + String(targetYawDeg, 2);
  s += ",\"headingErrorDeg\":" + String(positionErrorYawDeg, 2);
  s += ",\"done\":" + String(positionTargetReached ? "true" : "false");
  s += "}}";
  return s;
}

void moveGoalCallback(const void* msgIn) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgIn;
  if (msg == nullptr || msg->data.data == nullptr || msg->data.size == 0) {
    return;
  }

  size_t n = min((size_t)msg->data.size, (size_t)(UART_LINE_MAX - 1));
  memcpy(moveGoalBuffer, msg->data.data, n);
  moveGoalBuffer[n] = '\0';

  String line(moveGoalBuffer);
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String cmd = firstToken(line);
  cmd.toUpperCase();
  uint32_t seq = getUIntArg(line, "seq", 0);

  if (cmd == "MOVE") {
    float angleDeg = getFloatArg(line, "angle", 0.0f);
    float distCm = getFloatArg(line, "dist", 0.0f);
    float headingDeg = getFloatArg(line, "heading", 0.0f);
    STATE_LOCK();
    startPiPositionMove(seq, angleDeg, distCm, headingDeg);
    STATE_UNLOCK();
    return;
  }

  if (cmd == "TURN") {
    float headingDeg = getFloatArg(line, "heading", 0.0f);
    STATE_LOCK();
    startPiPositionMove(seq, 0.0f, 0.0f, headingDeg);
    STATE_UNLOCK();
    return;
  }

  if (cmd == "TWIST") {
    float forwardRpm = getFloatArg(line, "forward", 0.0f);
    float strafeRpm = getFloatArg(line, "strafe", 0.0f);
    float turnRpm = getFloatArg(line, "turn", 0.0f);
    uint32_t timeoutMs = getUIntArg(line, "timeout", CMD_VEL_TIMEOUT_MS);
    STATE_LOCK();
    startPiTwist(forwardRpm, strafeRpm, turnRpm, timeoutMs);
    STATE_UNLOCK();
    return;
  }

  if (cmd == "STOP") {
    STATE_LOCK();
    stopPiMotion(2);
    STATE_UNLOCK();
    return;
  }

  if (cmd == "RESET_ODOM") {
    STATE_LOCK();
    resetOdometryToCurrentCounts();
    resetPiBridgeOdom();
    positionErrorForwardCm = targetForwardCm;
    positionErrorStrafeCm = targetStrafeCm;
    positionErrorYawDeg = shortestAngleErrorDeg(targetYawDeg, displayedOdomYawDeg());
    resetAllBodyPiStates();
    STATE_UNLOCK();
    return;
  }

  if (cmd == "RESET_ENC") {
    STATE_LOCK();
    resetEncoderMeasurements();
    resetOdometryToCurrentCounts();
    resetPiBridgeOdom();
    resetAllPiStates();
    resetAllBodyPiStates();
    STATE_UNLOCK();
    return;
  }

  if (cmd == "ZERO_IMU") {
    STATE_LOCK();
    stopPiMotion(2);
    imuYawDeg = 0.0f;
    imuYawRawDeg = 0.0f;
    targetYawDeg = 0.0f;
    positionErrorYawDeg = 0.0f;
    STATE_UNLOCK();
    return;
  }

  if (cmd == "CAL_IMU" || cmd == "INIT_IMU") {
    STATE_LOCK();
    stopPiMotion(2);
    if (imuOk && imuCalibrateBias(500)) {
      imuYawDeg = 0.0f;
      imuYawRawDeg = 0.0f;
      targetYawDeg = 0.0f;
      positionErrorYawDeg = 0.0f;
    }
    STATE_UNLOCK();
    return;
  }
}

void telemetryTimerCallback(rcl_timer_t* timer, int64_t lastCallTime) {
  (void)lastCallTime;
  if (timer == nullptr) {
    return;
  }

  String telemetry;
  STATE_LOCK();
  telemetry = buildMicroRosTelemetryJson();
  if (piActiveSeq != 0 && piMoveActive && !positionModeActive && positionTargetReached) {
    latchPiMoveOdom();
    piActiveSeq = 0;
  }
  STATE_UNLOCK();

  telemetry.toCharArray(telemetryJsonBuffer, MICRO_ROS_TELEMETRY_BUFFER);
  telemetryJsonMsg.data.data = telemetryJsonBuffer;
  telemetryJsonMsg.data.size = strlen(telemetryJsonBuffer);
  telemetryJsonMsg.data.capacity = MICRO_ROS_TELEMETRY_BUFFER;

  RCSOFTCHECK(rcl_publish(&telemetryJsonPub, &telemetryJsonMsg, nullptr));
}

bool initMicroRosEntities() {
  if (rmw_uros_ping_agent(MICRO_ROS_AGENT_PING_TIMEOUT_MS, MICRO_ROS_AGENT_PING_ATTEMPTS) != RMW_RET_OK) {
    return false;
  }

  microrosAllocator = rcl_get_default_allocator();

  if (rclc_support_init(&microrosSupport, 0, nullptr, &microrosAllocator) != RCL_RET_OK) {
    return false;
  }
  if (rclc_node_init_default(&microrosNode, "esp32_controller", "audix", &microrosSupport) != RCL_RET_OK) {
    return false;
  }

  std_msgs__msg__String__init(&moveGoalMsg);
  std_msgs__msg__String__init(&telemetryJsonMsg);

  moveGoalMsg.data.data = moveGoalBuffer;
  moveGoalMsg.data.size = 0;
  moveGoalMsg.data.capacity = UART_LINE_MAX;
  telemetryJsonMsg.data.data = telemetryJsonBuffer;
  telemetryJsonMsg.data.size = 0;
  telemetryJsonMsg.data.capacity = MICRO_ROS_TELEMETRY_BUFFER;

  if (rclc_subscription_init_best_effort(
          &moveGoalSub,
          &microrosNode,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "esp/move_goal") != RCL_RET_OK) {
    return false;
  }
  if (rclc_publisher_init_best_effort(
          &telemetryJsonPub,
          &microrosNode,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "esp/telemetry_json") != RCL_RET_OK) {
    return false;
  }
  if (rclc_timer_init_default(
          &telemetryTimer,
          &microrosSupport,
          RCL_MS_TO_NS(PI_TELEMETRY_INTERVAL_MS),
          telemetryTimerCallback) != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_init(&microrosExecutor, &microrosSupport.context, 2, &microrosAllocator) != RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_add_subscription(&microrosExecutor, &moveGoalSub, &moveGoalMsg, &moveGoalCallback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_add_timer(&microrosExecutor, &telemetryTimer) != RCL_RET_OK) {
    return false;
  }

  return true;
}

void destroyMicroRosEntities() {
  if (!microrosEntitiesCreated) {
    return;
  }

  rmw_context_t* rmwContext = rcl_context_get_rmw_context(&microrosSupport.context);
  if (rmwContext != nullptr) {
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, 0);
  }

  (void)rcl_timer_fini(&telemetryTimer);
  (void)rcl_subscription_fini(&moveGoalSub, &microrosNode);
  (void)rcl_publisher_fini(&telemetryJsonPub, &microrosNode);
  (void)rclc_executor_fini(&microrosExecutor);
  (void)rcl_node_fini(&microrosNode);
  (void)rclc_support_fini(&microrosSupport);

  microrosNode = rcl_get_zero_initialized_node();
  telemetryJsonPub = rcl_get_zero_initialized_publisher();
  moveGoalSub = rcl_get_zero_initialized_subscription();
  telemetryTimer = rcl_get_zero_initialized_timer();
  microrosExecutor = rclc_executor_get_zero_initialized_executor();
  microrosEntitiesCreated = false;
}

void microRosTask(void* arg) {
  (void)arg;
  set_microros_transports();
  delay(2000);

  for (;;) {
    if (!microrosEntitiesCreated) {
      uint32_t nowMs = millis();
      if ((uint32_t)(nowMs - microrosLastConnectAttemptMs) >= MICRO_ROS_RECONNECT_PERIOD_MS) {
        microrosLastConnectAttemptMs = nowMs;
        microrosEntitiesCreated = initMicroRosEntities();
      }
      vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_SPIN_MS));
      continue;
    }

    if (rmw_uros_ping_agent(MICRO_ROS_AGENT_PING_TIMEOUT_MS, MICRO_ROS_AGENT_PING_ATTEMPTS) != RMW_RET_OK) {
      destroyMicroRosEntities();
      vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_RECONNECT_PERIOD_MS));
      continue;
    }

    rcl_ret_t spinResult = rclc_executor_spin_some(&microrosExecutor, RCL_MS_TO_NS(MICRO_ROS_SPIN_MS));
    if (spinResult != RCL_RET_OK && spinResult != RCL_RET_TIMEOUT) {
      destroyMicroRosEntities();
    }
    vTaskDelay(pdMS_TO_TICKS(MICRO_ROS_SPIN_MS));
  }
}

void setup() {
  Serial.begin(UART_BAUD);
  delay(250);

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
  imuInit();
  resetEncoderMeasurements();
  resetOdometryToCurrentCounts();
  resetAllPiStates();
  resetAllBodyPiStates();
  updateMecanumTargets();

  stateMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  if (!stateMutex || !serialMutex) {
    Serial.println("ERROR: mutex creation failed.");
    return;
  }

  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/cmd/config", HTTP_GET, handleConfig);
  server.on("/cmd/start", HTTP_GET, handleStart);
  server.on("/cmd/positionStart", HTTP_GET, handlePositionStart);
  server.on("/cmd/timedStart", HTTP_GET, handleTimedStart);
  server.on("/cmd/stop", HTTP_GET, handleStop);
  server.on("/cmd/resetI", HTTP_GET, handleResetIntegral);
  server.on("/cmd/resetEnc", HTTP_GET, handleResetEncoders);
  server.on("/cmd/resetOdom", HTTP_GET, handleResetOdometry);
  server.on("/cmd/imuZero", HTTP_GET, handleImuZero);
  server.on("/cmd/imuCal", HTTP_GET, handleImuCal);
  server.begin();

  (void)apOk;

  xTaskCreatePinnedToCore(controlTask, "mecanum-position-pi", 6144, nullptr, 4, &controlTaskHandle, 1);
  xTaskCreatePinnedToCore(microRosTask, "micro-ros", 20000, nullptr, 5, &microRosTaskHandle, 0);
  xTaskCreatePinnedToCore(webTask, "mecanum-web", 8192, nullptr, 2, &webTaskHandle, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
