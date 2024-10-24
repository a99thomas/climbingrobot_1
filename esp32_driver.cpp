#include <Arduino.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include "driver/ledc.h"

// https://lastminuteengineers.com/esp32-pinout-reference/
// Servo Motors
Servo motor1, motor2, motor5, motor6;

// Motor 3 and 7 Encoder Counts
volatile long encoder_count_3 = 0;
volatile long encoder_count_7 = 0;

// PID Control Variables for Motors 3 and 7
double setpoint3, input3, output3;
double setpoint7, input7, output7;

double Kp3 = 1.0, Ki3 = 0.0, Kd3 = 0.0;
double Kp7 = 1.0, Ki7 = 0.0, Kd7 = 0.0;

PID pid3(&input3, &output3, &setpoint3, Kp3, Ki3, Kd3, DIRECT);
PID pid7(&input7, &output7, &setpoint7, Kp7, Ki7, Kd7, DIRECT);


// Pin Assignments
const int MOTOR1_PWM = 33;
const int MOTOR2_PWM = 32;
const int MOTOR3_PWM = 21, MOTOR3_IN1 = 14, MOTOR3_IN2 = 27; //Change 21 to 27 for ESP32 Dev
const int MOTOR4_IN1 = 26, MOTOR4_IN2 = 25;
const int MOTOR5_PWM = 23;
const int MOTOR6_PWM = 22;
const int MOTOR7_PWM = 12, MOTOR7_IN1 = 18, MOTOR7_IN2 = 19;
const int MOTOR8_IN1 = 17, MOTOR8_IN2 = 5;

const int MOTOR3_CHA = 16, MOTOR3_CHB = 2;
const int MOTOR7_CHA = 4, MOTOR7_CHB = 5;


const int PWM_FREQ = 50; // 50 Hz for servos
const int PWM_RESOLUTION = 15; // 8-bit resolution
const int PWM_CHANNEL_1 = 0; // Channel for Motor 1
const int PWM_CHANNEL_2 = 1; // Channel for Motor 2
const int PWM_CHANNEL_3 = 2; // Channel for Motor 3
// const int PWM_CHANNEL_4 = 3; // Channel for Motor 4
const int PWM_CHANNEL_5 = 4; // Channel for Motor 5
const int PWM_CHANNEL_6 = 5; // Channel for Motor 6
const int PWM_CHANNEL_7 = 6; // Channel for Motor 7
// const int PWM_CHANNEL_8 = 7; // Channel for Motor 8
const int MIN_SERVO_ANGLE = 0;
const int MAX_SERVO_ANGLE = 270;
const int MAX_DUTY_CYCLE = (1 << PWM_RESOLUTION) - 1; // 4095 for 12-bit resolution
const int MIN_PWM_WIDTH = 500; // Minimum pulse width in microseconds
const int MAX_PWM_WIDTH = 2500; // Maximum pulse width in microseconds
const int PERIOD_MICROSECONDS = 1000000/PWM_FREQ; // 20ms for 50Hz

const int MIN_DUTY_CYCLE = MAX_DUTY_CYCLE*MIN_PWM_WIDTH/PERIOD_MICROSECONDS; // Minimum pulse width in microseconds
const int FULL_DUTY_CYCLE = MAX_DUTY_CYCLE*MAX_PWM_WIDTH/PERIOD_MICROSECONDS; // Maximum pulse width in microseconds


void IRAM_ATTR encoder_isr_3() {
  encoder_count_3 += (digitalRead(MOTOR3_CHA) == digitalRead(MOTOR3_CHB)) ? 1 : -1;
}

void IRAM_ATTR encoder_isr_7() {
  encoder_count_7 += (digitalRead(MOTOR7_CHA) == digitalRead(MOTOR7_CHB)) ? 1 : -1;
}

// Function prototypes
void processCommand(String command);
void setMotorDirection(int in1, int in2, int speed);
void sendJointPositions();

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void setup() {
  Serial.begin(115200);

  // Attach Servo Motors and Initialize LEDC
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_3, PWM_FREQ, PWM_RESOLUTION);
  // ledcSetup(PWM_CHANNEL_4, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_5, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_6, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_7, PWM_FREQ, PWM_RESOLUTION);
  // ledcSetup(PWM_CHANNEL_8, PWM_FREQ, PWM_RESOLUTION);

  // Attach the channels to the pins
  ledcAttachPin(MOTOR1_PWM, PWM_CHANNEL_1);
  ledcAttachPin(MOTOR2_PWM, PWM_CHANNEL_2);
  ledcAttachPin(MOTOR3_PWM, PWM_CHANNEL_3);
  // ledcAttachPin(MOTOR4_PWM, PWM_CHANNEL_4);
  ledcAttachPin(MOTOR5_PWM, PWM_CHANNEL_5);
  ledcAttachPin(MOTOR6_PWM, PWM_CHANNEL_6);
  ledcAttachPin(MOTOR7_PWM, PWM_CHANNEL_7);
  // ledcAttachPin(MOTOR8_PWM, PWM_CHANNEL_8);

  // Set initial duty cycle to 0
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);
  ledcWrite(PWM_CHANNEL_3, 0);
  ledcWrite(PWM_CHANNEL_5, 0);
  ledcWrite(PWM_CHANNEL_6, 0);
  ledcWrite(PWM_CHANNEL_7, 0);

  // Set DC Motor Pins as Outputs
  // pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);

  pinMode(MOTOR4_IN1, OUTPUT);
  pinMode(MOTOR4_IN2, OUTPUT);

  pinMode(MOTOR7_PWM, OUTPUT);
  pinMode(MOTOR7_IN1, OUTPUT);
  pinMode(MOTOR7_IN2, OUTPUT);

  pinMode(MOTOR8_IN1, OUTPUT);
  pinMode(MOTOR8_IN2, OUTPUT);

  // Encoder Pins
  pinMode(MOTOR3_CHA, INPUT);
  pinMode(MOTOR3_CHB, INPUT);
  pinMode(MOTOR7_CHA, INPUT);
  pinMode(MOTOR7_CHB, INPUT);

  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(MOTOR3_CHA), encoder_isr_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR7_CHA), encoder_isr_7, CHANGE);


  // Initialize PID Controllers
  pid3.SetMode(AUTOMATIC);
  pid7.SetMode(AUTOMATIC);
}


void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // Update PID Control for Motors 3 and 7
  input3 = encoder_count_3;
  pid3.Compute();
  setMotorDirection(MOTOR3_IN1, MOTOR3_IN2, output3);
  ledcWrite(MOTOR3_PWM, abs(output3)); // Use ledcWrite for ESP32 PWM

  input7 = encoder_count_7;
  pid7.Compute();
  setMotorDirection(MOTOR7_IN1, MOTOR7_IN2, output7);
  // ledcWrite(MOTOR7_PWM, abs(output7)); // Use ledcWrite for ESP32 PWM

  // Send joint positions to Raspberry Pi
  sendJointPositions();

  // Add a small delay to avoid WDT reset
  delay(200); // Adjust as necessary
}

void processCommand(String command) {
  //PID commands look like "PID<motor_number>,<Kp_value>,<Ki_value>,<Kd_value>"
  if (command.startsWith("PID")) {
    int motor = command.substring(3, 4).toInt();
    double p = command.substring(5, command.indexOf(',', 5)).toDouble();
    double i = command.substring(command.indexOf(',', 5) + 1, command.lastIndexOf(',')).toDouble();
    double d = command.substring(command.lastIndexOf(',') + 1).toDouble();

    // if (motor == 3) pid3.SetTunings(p, i, d);
    // if (motor == 7) pid7.SetTunings(p, i, d);
        if (motor == 3) {
        pid3.SetTunings(p, i, d);
        // Serial.print("Motor 3 PID updated: P = "); Serial.print(p);
        // Serial.print(", I = "); Serial.print(i);
        // Serial.print(", D = "); Serial.println(d);
    }
    if (motor == 7) {
        pid7.SetTunings(p, i, d);
        // Serial.print("Motor 7 PID updated: P = "); Serial.print(p);
        // Serial.print(", I = "); Serial.print(i);
        // Serial.print(", D = "); Serial.println(d);
    }
  } else {
    String values[8];
    int motor = 0;
    int startIndex = 0;
    for (int i = 0; i < 8; i++) {
      int endIndex = command.indexOf(',', startIndex);
      values[i] = command.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }
    // Serial.println(values[0].toInt());
    //     for (int i = 0; i < 8; i++) {
    //   Serial.print("Value "); Serial.print(i); Serial.print(": "); Serial.println(values[i]);
    // }
    // Map angle to pulse width
    // Convert angles to pulse widths
    int pulseWidth1 = mapFloat(values[0].toFloat(), MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, MIN_PWM_WIDTH, MAX_PWM_WIDTH);
    int pulseWidth2 = mapFloat(values[1].toFloat(), MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, MIN_PWM_WIDTH, MAX_PWM_WIDTH);
    int pulseWidth5 = mapFloat(values[4].toFloat(), MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, MIN_PWM_WIDTH, MAX_PWM_WIDTH);
    int pulseWidth6 = mapFloat(values[5].toFloat(), MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, MIN_PWM_WIDTH, MAX_PWM_WIDTH);

    // Calculate duty cycle for 12-bit resolution
    int dutyCycle1 = (pulseWidth1 * MAX_DUTY_CYCLE) / PERIOD_MICROSECONDS; // Convert pulse width to duty cycle
    int dutyCycle2 = (pulseWidth2 * MAX_DUTY_CYCLE) / PERIOD_MICROSECONDS;
    int dutyCycle5 = (pulseWidth5 * MAX_DUTY_CYCLE) / PERIOD_MICROSECONDS;
    int dutyCycle6 = (pulseWidth6 * MAX_DUTY_CYCLE) / PERIOD_MICROSECONDS;

    // Write duty cycles to PWM channels
    ledcWrite(PWM_CHANNEL_1, dutyCycle1);
    ledcWrite(PWM_CHANNEL_2, dutyCycle2);
    ledcWrite(PWM_CHANNEL_5, dutyCycle5);
    ledcWrite(PWM_CHANNEL_6, dutyCycle6);
  
    // Set DC Motor 4 and 8 directions
    setMotorDirection(MOTOR4_IN1, MOTOR4_IN2, values[3].toInt());
    setMotorDirection(MOTOR8_IN1, MOTOR8_IN2, values[7].toInt());
  }
}

void setMotorDirection(int in1, int in2, int speed) {
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
}

void sendJointPositions() {
  String positions = String(mapFloat(ledcRead(PWM_CHANNEL_1), MIN_DUTY_CYCLE, FULL_DUTY_CYCLE, 0.0, 270.0)) + ","
                       + String(mapFloat(ledcRead(PWM_CHANNEL_2), MIN_DUTY_CYCLE, FULL_DUTY_CYCLE, 0.0, 270.0)) + ","
                       + String(encoder_count_3) + ",0,"
                       + String(mapFloat(ledcRead(PWM_CHANNEL_5), MIN_DUTY_CYCLE, FULL_DUTY_CYCLE, 0.0, 270.0)) + ","
                       + String(mapFloat(ledcRead(PWM_CHANNEL_6), MIN_DUTY_CYCLE, FULL_DUTY_CYCLE, 0.0, 270.0)) + ","
                       + String(encoder_count_7) + ",0\n";
  Serial.print(positions);
}
