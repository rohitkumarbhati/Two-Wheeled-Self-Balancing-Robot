#include <Wire.h>

// -------------------- MPU6050 --------------------
int MPUaddress = 0x68; // I2C address

// Raw sensor variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// Offsets
float accelX_off = 0, accelY_off = 0, accelZ_off = 0;
float gyroX_off = 0, gyroY_off = 0, gyroZ_off = 0;

// State variables
float theta = 0.0;     // tilt angle (rad)
float omega = 0.0;     // tilt rate (rad/s)
float phi   = 0.0;     // wheel angle (rad)
float phirate = 0.0;   // wheel angular velocity (rad/s)
float u = 0.0;         // control input
float balanceAngle = 0.0; // learned upright angle (rad)

// Complementary filter parameter
float cf_alpha = 0.98;

// Timing
unsigned long lastMicros;

// -------------------- Motor Driver (L298N) --------------------
const int L_FWD = 4;
const int L_REV = 5;
const int L_PWM = 3;
const int R_FWD = 7;
const int R_REV = 8;
const int R_PWM = 6;

// -------------------- Robot/Motor Parameters --------------------
// These must be measured for your motors!
float motorA = 157.0;     // rad/s² per unit input
float motorB = 3.57;      // damping

// -------------------- State Feedback Gains --------------------
float K_theta   = 40;
float K_omega   = .8;
float K_phi     = 0;
float K_phirate = 0.02;

// Deadband (deg) -> inside this, motors stop
float deadband = 1.0;

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake up MPU6050
  Wire.beginTransmission(MPUaddress);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wakes up MPU-6050
  Wire.endTransmission(true);

  // Motor pins
  pinMode(L_FWD, OUTPUT);
  pinMode(L_REV, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_FWD, OUTPUT);
  pinMode(R_REV, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  // ---- Calibrate IMU ----
  Serial.println("Keep robot upright and still for 5 seconds...");
  delay(2000);
  calibrateIMU(1000); // take 1000 samples for offset

  // Record current tilt as balance reference
  getAccel();
  getGyro();
  balanceAngle = atan2(accelX, accelZ);  // radians
  Serial.print("Calibration done. Balance angle (deg): ");
  Serial.println(balanceAngle * 180.0 / 3.14159, 2);

  // ✅ CSV header for logging
  Serial.println("Theta_deg,Error_deg,Omega_rad_s,AccelX_g,AccelY_g,AccelZ_g,Phi_rad,Phirate_rad_s,Control_u");

  lastMicros = micros();
}

// -------------------- Main Loop --------------------
void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6;
  if (dt <= 0) dt = 0.001;
  lastMicros = now;

  // Get IMU data
  getAccel();
  getGyro();

  // Tilt angle from accelerometer
  float accelAngle = atan2(accelX, accelZ);   // radians

  // Angular rate from gyro (°/s → rad/s)
  float gyroRate = gyroX * 3.14159 / 180.0;   // rad/s

  // Complementary filter
  theta = cf_alpha * (theta + gyroRate * dt) + (1.0 - cf_alpha) * accelAngle;
  omega = gyroRate;

  // Wheel dynamics (simplified)
  phi += phirate * dt;
  phirate += (motorA * u - motorB * phirate) * dt;

  // -------------------- Control Law --------------------
  float error = theta - balanceAngle; // deviation from upright
  u = K_theta * error + K_omega * omega + K_phi * phi + K_phirate * phirate;

  // Deadband: stop motors near upright
  if (abs(error * 180.0 / 3.14159) < deadband) {
    u = 0;
  }

  // Limit control
  u = lim(u);

  // Drive motors (both sides)
  setdrive(u, L_FWD, L_REV, L_PWM);
  setdrive(u, R_FWD, R_REV, R_PWM);

  // -------------------- CSV Logging --------------------
  Serial.print(theta * 180.0 / 3.14159, 2); Serial.print(",");
  Serial.print(error * 180.0 / 3.14159, 2); Serial.print(",");
  Serial.print(omega, 3); Serial.print(",");
  Serial.print(accelX, 3); Serial.print(",");
  Serial.print(accelY, 3); Serial.print(",");
  Serial.print(accelZ, 3); Serial.print(",");
  Serial.print(phi, 3); Serial.print(",");
  Serial.print(phirate, 3); Serial.print(",");
  Serial.println(u, 3);

  // Example turning (manual test, comment/uncomment)
  // leftTurn(120);
  // rightTurn(120);
}

// -------------------- getAccel() --------------------
void getAccel() {
  Wire.beginTransmission(MPUaddress);
  Wire.write(0x3B); // starting register for AccelX
  Wire.endTransmission(false);
  Wire.requestFrom(MPUaddress, 6, true);

  accelX = (Wire.read() << 8 | Wire.read()) / 16384.0 - accelX_off;
  accelY = (Wire.read() << 8 | Wire.read()) / 16384.0 - accelY_off;
  accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0 - accelZ_off;
}

// -------------------- getGyro() --------------------
void getGyro() {
  Wire.beginTransmission(MPUaddress);
  Wire.write(0x43); // starting register for GyroX
  Wire.endTransmission(false);
  Wire.requestFrom(MPUaddress, 6, true);

  gyroX = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroX_off;
  gyroY = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroY_off;
  gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - gyroZ_off;
}

// -------------------- IMU Calibration --------------------
void calibrateIMU(int samples) {
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPUaddress);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPUaddress, 6, true);
    int16_t ax = (Wire.read() << 8 | Wire.read());
    int16_t ay = (Wire.read() << 8 | Wire.read());
    int16_t az = (Wire.read() << 8 | Wire.read());

    Wire.beginTransmission(MPUaddress);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPUaddress, 6, true);
    int16_t gx = (Wire.read() << 8 | Wire.read());
    int16_t gy = (Wire.read() << 8 | Wire.read());
    int16_t gz = (Wire.read() << 8 | Wire.read());

    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;

    delay(2);
  }

  accelX_off = (ax_sum / (float)samples) / 16384.0;
  accelY_off = (ay_sum / (float)samples) / 16384.0;
  accelZ_off = (az_sum / (float)samples) / 16384.0 - 1.0; // remove gravity

  gyroX_off = (gx_sum / (float)samples) / 131.0;
  gyroY_off = (gy_sum / (float)samples) / 131.0;
  gyroZ_off = (gz_sum / (float)samples) / 131.0;
}

// -------------------- Motor Drive Helpers --------------------
int lim(int x) {
  if (x > 255) x = 255;
  if (x < -255) x = -255;
  return x;
}

void setdrive(int m, int f, int r, int p) {
  if (m > 0) {
    digitalWrite(f, HIGH);
    digitalWrite(r, LOW);
  } else if (m < 0) {
    digitalWrite(f, LOW);
    digitalWrite(r, HIGH);
  } else {
    digitalWrite(f, LOW);
    digitalWrite(r, LOW);
  }
  m = lim(abs(m));
  analogWrite(p, m);
}

// -------------------- Extra Functions for Turning --------------------
void leftTurn(int pwmVal) {
  // Left wheel backward, right wheel forward
  digitalWrite(L_FWD, LOW);
  digitalWrite(L_REV, HIGH);
  analogWrite(L_PWM, pwmVal);

  digitalWrite(R_FWD, HIGH);
  digitalWrite(R_REV, LOW);
  analogWrite(R_PWM, pwmVal);
}

void rightTurn(int pwmVal) {
  // Left wheel forward, right wheel backward
  digitalWrite(L_FWD, HIGH);
  digitalWrite(L_REV, LOW);
  analogWrite(L_PWM, pwmVal);

  digitalWrite(R_FWD, LOW);
  digitalWrite(R_REV, HIGH);
  analogWrite(R_PWM, pwmVal);
}
