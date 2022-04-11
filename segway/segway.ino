#include <Wire.h>
#include <Kalman.h>

#define motor_speed_pin 3
#define motor_dir_pin1 4
#define motor_dir_pin2 5

/***** START OF TUNABLE PARAMETERS *****/
//#define DO_PRINT_FREQUENCY
//#define DO_DATA_LOGGING // Doesn't work that great (slows down the control loop too much)
const double logging_freq = 256; // Hz (Arduino Uno has 1KB of EEPROM, and we store 4 bytes at a time)

// PID parameters (Calculated values: kp=0.35, ki=0.001, kd=95) */
const double command_angle = -2.7;
const uint8_t derivative_dp = 4; // Number of datapoints for the derivative moving average
const double integral_max = 2;
const double kp = 40;
const double ki = 300;
const double kd = 0.4;

/* Ease of use parameters */
const uint8_t fall_angle = 40;
const uint16_t wake_up_delay = 1000; //ms
/***** END OF TUNABLE PARAMETERS *****/

/* Controller variables */
double angle_error, angle_error_prev, angle_error_itg, angle_error_drv;
double controller;
uint8_t controller_corrected;

uint32_t wake_up_timer;
byte is_fallen;

Kalman kalman_filter;

// IMU Data
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyro_angle;
double kalman_angle; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// Data Logging
#ifdef DO_DATA_LOGGING
#include <EEPROM.h>
uint32_t logging_timer;
uint16_t logging_addr = 0;
#endif

/* Sampling Frequency */
#ifdef DO_PRINT_FREQUENCY
double dt_acc = 0;
uint16_t dt_i = 0;
uint16_t n_dt = 500;
#endif

void setup() {  
  // Motor driver pins
  pinMode(motor_speed_pin, OUTPUT);
  pinMode(motor_dir_pin1, OUTPUT);
  pinMode(motor_dir_pin2, OUTPUT);

  // Serial for debugging, Wire for MPU6050
  Serial.begin(500000);

  // Dump data log
  #ifdef DO_DATA_LOGGING
  Serial.print("Logging freq:\t"); Serial.println(logging_freq);
  Serial.print("i"); Serial.print('\t');
  Serial.print("angle_error*16"); Serial.print('\t');
  Serial.print("proportional"); Serial.print('\t');
  Serial.print("integral"); Serial.print('\t');
  Serial.println("derivative");
  int8_t data = 0;
  for (int addr=0; addr<EEPROM.length(); addr += 4) {
    Serial.print(addr/4); Serial.print('\t');
    Serial.print(EEPROM.get(addr, data)); Serial.print('\t');
    Serial.print(EEPROM.get(addr+1, data)); Serial.print('\t');
    Serial.print(EEPROM.get(addr+2, data)); Serial.print('\t');
    Serial.println(EEPROM.get(addr+3, data));
  }
  Serial.println("End of logged data");
  #endif
  
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accZ = (int16_t)((i2cData[0] << 8) | i2cData[1]); // accX on MPU6050
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]); // accY on MPU6050
  accX = -(int16_t)((i2cData[4] << 8) | i2cData[5]); // accZ on MPU6050

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  double acc_angle  = atan2(accY, accZ) * RAD_TO_DEG;

  kalman_filter.setAngle(acc_angle); // Set starting angle

  timer = micros();
}

void loop() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accZ = (int16_t)((i2cData[0] << 8) | i2cData[1]); // accX on MPU6050
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]); // accY on MPU6050
  accX = -(int16_t)((i2cData[4] << 8) | i2cData[5]); // accZ on MPU6050
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroZ = (int16_t)((i2cData[8] << 8) | i2cData[9]); // gyroX on MPU6050
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]); // gyroY on MPU6050
  gyroX = -(int16_t)((i2cData[12] << 8) | i2cData[13]); // gyroZ on MPU6050

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  #ifdef DO_PRINT_FREQUENCY
  dt_acc += dt;
  dt_i += 1;
  if (dt_i == n_dt) {
    Serial.println(n_dt/dt_acc); // Print the sampling frequency
    dt_acc = dt_i = 0;
  }
  #endif

  // https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf eq.25
  double acc_angle  = atan2(accY, accZ) * RAD_TO_DEG;

  double gyro_angle_rate = gyroX / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((acc_angle < -90 && kalman_angle > 90) || (acc_angle > 90 && kalman_angle < -90)) {
    kalman_filter.setAngle(acc_angle);
    kalman_angle = acc_angle;
  } else
    kalman_angle = kalman_filter.getAngle(acc_angle, gyro_angle_rate, dt); // Calculate the angle using a Kalman filter

  gyro_angle += gyro_angle_rate * dt; // Calculate gyro angle without any filter
  //gyro_angle += kalman_filter.getRate() * dt; // Calculate gyro angle using the unbiased rate

  // Reset the gyro angle when it has drifted too much
  if (gyro_angle < -180 || gyro_angle > 180)
    gyro_angle = kalman_angle;

  
  //Serial.print(acc_angle); Serial.print("\t");
  //Serial.print(kalman_angle); Serial.print("\t");


  // Controller
  angle_error = command_angle - kalman_angle;
  
  if (fabs(angle_error) > fall_angle) {
    // Segway IS down... reset everything
    angle_error_itg = 0;
    controller_corrected = 0;
    is_fallen = 1;
    wake_up_timer = micros();
  #ifdef DO_DATA_LOGGING
    logging_addr = 0;
  #endif
  } else if (is_fallen && (micros()-wake_up_timer)/1000 < wake_up_delay) {
    // Segway is up, but was down... wait a moment before doing anything
  } else {
    // Segway is up
    // PID Controller
    is_fallen = 0;
    angle_error_itg += (angle_error_prev+angle_error)*dt/2;
    angle_error_itg = fmax(-integral_max, fmin(integral_max, angle_error_itg));
    angle_error_drv = 1/derivative_dp * ((derivative_dp-1)*angle_error_drv + (angle_error-angle_error_prev)/dt);
    double proportional = kp*angle_error;
    double integral = ki*angle_error_itg;
    double derivative = kd*angle_error_drv;
  #ifdef DO_DATA_LOGGING
    if (logging_addr+4 <= EEPROM.length() && (micros()-logging_timer) >= (1000000/logging_freq)) {
      EEPROM.write(logging_addr, (int8_t) fmax(-128, fmin(127, angle_error*16)));
      EEPROM.write(logging_addr+1, (int8_t) fmax(-128, fmin(127, proportional)));
      EEPROM.write(logging_addr+2, (int8_t) fmax(-128, fmin(127, integral)));
      EEPROM.write(logging_addr+3, (int8_t) fmax(-128, fmin(127, derivative)));
      logging_addr += 4;
      logging_timer = micros();
    }
  #endif
    controller = proportional + integral + derivative;
    angle_error_prev = angle_error;
    controller_corrected = fmin(255, fabs(controller));
    
  }

  digitalWrite(motor_dir_pin1, controller < 0);
  digitalWrite(motor_dir_pin2, controller >= 0);
  analogWrite(motor_speed_pin, controller_corrected);
  
  //Serial.print(angle_error_itg); Serial.print('\t');
  //Serial.print(controller_corrected);

  //Serial.print("\r\n");
}
