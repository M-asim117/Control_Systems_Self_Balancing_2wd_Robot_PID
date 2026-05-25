// =====================================================
// SELF-BALANCING TWO-WHEEL ROBOT USING PID CONTROL
// Control Systems Lab - Lab 12
// ESP32 S3 + MPU6050 + L298N
// =====================================================
#include <Wire.h>
#include <MPU6050.h>
// =====================================================
// MPU6050 ADDRESS AND REGISTER DEFINITIONS
// =====================================================
#define MPU6050_ADDR 0x68          // Default I2C address
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
// =====================================================
// PIN DEFINITIONS FOR ESP32 S3 + L298N MOTOR DRIVER
// =====================================================
// Left Motor Control (ENA, IN1, IN2)
#define PWM_LEFT_PIN 4       // GPIO4 - ENA (PWM for left motor speed)
#define DIR_LEFT_PIN_1 5     // GPIO5 - IN1 (Left motor direction control)
#define DIR_LEFT_PIN_2 6     // GPIO6 - IN2 (Left motor direction control)
// Right Motor Control (IN3, IN4, ENB)
#define DIR_RIGHT_PIN_1 7    // GPIO7 - IN3 (Right motor direction control)
#define DIR_RIGHT_PIN_2 15   // GPIO15 - IN4 (Right motor direction control)
#define PWM_RIGHT_PIN 16     // GPIO16 - ENB (PWM for right motor speed)
// PWM Configuration
#define PWM_FREQUENCY 5000   // 5 kHz PWM frequency
#define PWM_RESOLUTION 8     // 8-bit resolution (0-255)
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
// =====================================================
// IMU RAW DATA
// =====================================================
int16_t acc_x = 0, acc_y = 0, acc_z = 0;
int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
float acc_x_cal = 0.0, acc_z_cal = 0.0;
float gyro_y_cal = 0.0;
// Calibration offsets
int16_t acc_x_offset = 0;
int16_t acc_z_offset = 0;
int16_t gyro_y_offset = 0;
// =====================================================
// CONTROL LOOP TIMING
// =====================================================
#define CONTROL_FREQUENCY_HZ 80
#define CONTROL_LOOP_TIME_MS (1000 / CONTROL_FREQUENCY_HZ)
unsigned long last_loop_time = 0;
// =====================================================
// ANGLE ESTIMATION
// =====================================================
float angle_acc = 0.0;
float angle_gyro = 0.0;
float angle_filtered = 0.0;
float alpha = 0.98;  // Complementary filter weight
// =====================================================
// PID CONTROLLER
// =====================================================
float Kp = 37.0;   // Proportional gain
float Ki = 0.19;    // Integral gain
float Kd = 6.5;    // Derivative gain
float error = 0.0;
float error_prev = 0.0;
float error_integral = 0.0;
float u = 0.0;
float desired_angle = 0.0;  // Target: 0 degrees (upright)
// =====================================================
// MOTOR CONTROL
// =====================================================
float motor_output_left = 0.0;
float motor_output_right = 0.0;
int pwm_left = 0;
int pwm_right = 0;
// =====================================================
// DATA RECORDING
// =====================================================
unsigned long data_start_time = 0;
const unsigned long DATA_WINDOW_MS = 60000;
bool data_recording = false;
// =====================================================
// FUNCTION DECLARATIONS
// =====================================================
void initMPU6050();
void readMPU6050();
void calibrateMPU6050();
void estimateAngle(float dt);
void computePID(float dt);
void mapMotorCommand();
void updateMotorPWM();
void recordData(unsigned long elapsed_time);
float limitValue(float value, float min_val, float max_val);
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);
void readBytes(uint8_t reg, uint8_t *data, uint8_t length);
// =====================================================
// SETUP
// =====================================================
void setup() { 
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n========== SELF-BALANCING ROBOT INITIALIZATION ==========\n");
  // Initialize I2C
  // CORRECTED: SDA=GPIO8, SCL=GPIO9
  Wire.begin(8, 9);
  Wire.setClock(400000);  // 400 kHz I2C speed
  delay(100);
  // Initialize MPU6050
  Serial.println(">> Initializing MPU6050 IMU Sensor...");
  initMPU6050();
  delay(100);
  // Calibrate MPU6050
  Serial.println(">> Starting Sensor Calibration (keep robot still and upright)...");
  calibrateMPU6050();
  Serial.println(">> Calibration Complete\n");
  // Initialize motor driver pins
  Serial.println(">> Configuring Motor Driver Pins...");
  // Left motor
  ledcAttach(PWM_LEFT_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  pinMode(DIR_LEFT_PIN_1, OUTPUT);
  pinMode(DIR_LEFT_PIN_2, OUTPUT);
  digitalWrite(DIR_LEFT_PIN_1, LOW);
  digitalWrite(DIR_LEFT_PIN_2, LOW);
  // Right motor
  ledcAttach(PWM_RIGHT_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  pinMode(DIR_RIGHT_PIN_1, OUTPUT);
  pinMode(DIR_RIGHT_PIN_2, OUTPUT);
  digitalWrite(DIR_RIGHT_PIN_1, LOW);
  digitalWrite(DIR_RIGHT_PIN_2, LOW);
  Serial.println(">> Motor Driver Configured\n");
  Serial.println(">> Pin Configuration:");
  Serial.println("   Left Motor:  ENA=GPIO4, IN1=GPIO5, IN2=GPIO6");
  Serial.println("   Right Motor: ENB=GPIO16, IN3=GPIO7, IN4=GPIO15");
  Serial.println("   I2C: SDA=GPIO8, SCL=GPIO9\n");
  // Initialize timing
  last_loop_time = millis();
  data_start_time = millis();
  data_recording = true;
  Serial.println("========== INITIALIZATION COMPLETE ==========");
  Serial.println("Robot will start balancing in 3 seconds...\n");
  delay(3000);
  Serial.println("TIME_MS\tANGLE_ACC\tANGLE_GYRO\tANGLE_FILTERED\tERROR\tMOTOR_CMD");
  Serial.println("-----------------------------------------------------------");
}
// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  unsigned long current_time = millis();
  if ((current_time - last_loop_time) >= CONTROL_LOOP_TIME_MS) { 
    float dt = (current_time - last_loop_time) / 1000.0;
    last_loop_time = current_time; 
    // STEP 3: Read IMU
    readMPU6050();
    // STEP 4: Estimate Angle
    estimateAngle(dt);
    // STEP 5: Compute PID
    computePID(dt);
    // STEP 6: Map Motor Command
    mapMotorCommand();
    // STEP 7: Update Motors
    updateMotorPWM();
    // STEP 8: Record Data
    if (data_recording && (current_time - data_start_time) < DATA_WINDOW_MS) {
      recordData(current_time - data_start_time);
    } else if (data_recording) {
      Serial.println("\n========== 1-MINUTE DATA RECORDING COMPLETE ==========\n");
      data_recording = false;
    }
  }
}
// =====================================================
// INITIALIZE MPU6050 VIA I2C
// =====================================================
void initMPU6050() {
  // Wake up MPU6050
  writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00);
  delay(100);
  // Set gyro sensitivity: ±2000 dps
  writeRegister(MPU6050_REG_GYRO_CONFIG, 0x18);
  delay(10);
  // Set accel sensitivity: ±16g
  writeRegister(MPU6050_REG_ACCEL_CONFIG, 0x18);
  delay(10);
  // Set low-pass filter
  writeRegister(MPU6050_REG_CONFIG, 0x06);
  delay(10);
  Serial.println(">> MPU6050 Initialized Successfully");
}
// =====================================================
// READ MPU6050 SENSOR DATA
// =====================================================
void readMPU6050() {
  uint8_t buffer[14];
  readBytes(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
  // Extract accelerometer data (first 6 bytes)
  acc_x = (int16_t)((buffer[0] << 8) | buffer[1]);
  acc_y = (int16_t)((buffer[2] << 8) | buffer[3]);
  acc_z = (int16_t)((buffer[4] << 8) | buffer[5]);
  // Extract gyroscope data (last 6 bytes, skip temp)
  gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
  gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
  gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
  // Apply calibration offsets
  acc_x -= acc_x_offset;
  acc_z -= acc_z_offset;
  gyro_y -= gyro_y_offset;
  // Convert to physical units
  acc_x_cal = acc_x / 2048.0;   // ±16g: 2048 LSB/g
  acc_z_cal = acc_z / 2048.0;
  gyro_y_cal = gyro_y / 131.0;  // ±2000 dps: 131 LSB/(deg/s)
}
// =====================================================
// CALIBRATE MPU6050
// =====================================================
void calibrateMPU6050() {
  const int samples = 200;
  int32_t acc_x_sum = 0, acc_z_sum = 0, gyro_y_sum = 0;
  for (int i = 0; i < samples; i++) {
    uint8_t buffer[14];
    readBytes(MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
    acc_x_sum += (int16_t)((buffer[0] << 8) | buffer[1]);
    acc_z_sum += (int16_t)((buffer[4] << 8) | buffer[5]);
    gyro_y_sum += (int16_t)((buffer[10] << 8) | buffer[11]);
    delay(10);
  }
  acc_x_offset = acc_x_sum / samples;
  acc_z_offset = acc_z_sum / samples;
  gyro_y_offset = gyro_y_sum / samples;
  Serial.print("  Acc_X Offset: ");
  Serial.print(acc_x_offset);
  Serial.print("  Acc_Z Offset: ");
  Serial.print(acc_z_offset);
  Serial.print("  Gyro_Y Offset: ");
  Serial.println(gyro_y_offset);
}
// =====================================================
// ANGLE ESTIMATION WITH COMPLEMENTARY FILTER
// =====================================================
void estimateAngle(float dt) {
  // Accelerometer angle
  angle_acc = atan2(acc_x_cal, acc_z_cal) * 180.0 / PI;
  // Gyroscope integration
  angle_gyro = angle_gyro + (gyro_y_cal * dt);
  // Complementary filter
  angle_filtered = alpha * (angle_filtered + gyro_y_cal * dt) + (1.0 - alpha) * angle_acc;
}
// =====================================================
// PID CONTROLLER
// =====================================================
void computePID(float dt) {
  // Error calculation: desired_angle - measured_angle
  // Desired angle = 0 degrees (upright)
  error = desired_angle - angle_filtered;
  // Proportional term
  float p_term = Kp * error;
  // Integral term with anti-windup
  error_integral += error * dt;
  if (error_integral > 50.0) error_integral = 50.0;
  if (error_integral < -50.0) error_integral = -50.0;
  float i_term = Ki * error_integral;
  // Derivative term
  float derivative = (error - error_prev) / dt;
  float d_term = Kd * derivative;
  error_prev = error;
  // PID output
  u = p_term + i_term + d_term;
  // Saturate output
  u = limitValue(u, -255.0, 255.0);
}
// =====================================================
// MAP MOTOR COMMAND
// =====================================================
void mapMotorCommand() { 
  motor_output_left = u;
  motor_output_right = u;
}
// =====================================================
// UPDATE MOTOR PWM
// =====================================================
void updateMotorPWM() {
  // Left motor control
  if (motor_output_left > 0) {
    // Forward: IN1=HIGH, IN2=LOW
    digitalWrite(DIR_LEFT_PIN_1, LOW);
    digitalWrite(DIR_LEFT_PIN_2, HIGH);
    pwm_left = (int)limitValue(motor_output_left, 0, 255);
  } else {
    // Backward: IN1=LOW, IN2=HIGH
    digitalWrite(DIR_LEFT_PIN_1, HIGH);
    digitalWrite(DIR_LEFT_PIN_2, LOW);
    pwm_left = (int)limitValue(-motor_output_left, 0, 255);
  }
  // Right motor control
  if (motor_output_right > 0) {
    // Forward: IN3=HIGH, IN4=LOW
    digitalWrite(DIR_RIGHT_PIN_1, LOW);
    digitalWrite(DIR_RIGHT_PIN_2, HIGH);
    pwm_right = (int)limitValue(motor_output_right, 0, 255);
  } else {
    // Backward: IN3=LOW, IN4=HIGH
    digitalWrite(DIR_RIGHT_PIN_1, HIGH);
    digitalWrite(DIR_RIGHT_PIN_2, LOW);
    pwm_right = (int)limitValue(-motor_output_right, 0, 255);
  }
  // Apply PWM
  ledcWrite(PWM_LEFT_PIN, pwm_left);
  ledcWrite(PWM_RIGHT_PIN, pwm_right);
}
// =====================================================
// RECORD DATA
// =====================================================
void recordData(unsigned long elapsed_time) {
  Serial.print(elapsed_time);
  Serial.print("\t");
  Serial.print(angle_acc, 2);
  Serial.print("\t");
  Serial.print(angle_gyro, 2);
  Serial.print("\t");
  Serial.print(angle_filtered, 2);
  Serial.print("\t");
  Serial.print(error, 2);
  Serial.print("\t");
  Serial.println(u, 2);
}
// =====================================================
// I2C COMMUNICATION FUNCTIONS
// =====================================================
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  return Wire.read();
}
void readBytes(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (int)length, true);
  
  for (uint8_t i = 0; i < length; i++) {
    data[i] = Wire.read();
  }
}
// =====================================================
// HELPER FUNCTION (doesn't conflict with macro)
// =====================================================
float limitValue(float value, float min_val, float max_val) {
  if (value > max_val) return max_val;
  if (value < min_val) return min_val;
  return value;
}
// =====================================================
// END OF CODE
// =====================================================
