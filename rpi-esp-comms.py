
#include <Arduino.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include "driver/ledc.h"

// https://lastminuteengineers.com/esp32-pinout-reference/
// Servo Motors
Servo motor1, motor2, motor5, motor6;

// PID Control Variables for Motors 3 and 7
double setpoint3, input3, output3;
double setpoint7, input7, output7;

double Kp3a = 40, Ki3a = 0.00, Kd3a = 3.;
double Kp3b = 40, Ki3b = 5, Kd3b = 3.;
double Kp7a = 40, Ki7a = 0.00, Kd7a = 3.;
double Kp7b = 40, Ki7b = 5, Kd7b = 3.;

PID pid3(&input3, &output3, &setpoint3, Kp3a, Ki3a, Kd3a, DIRECT);
PID pid7(&input7, &output7, &setpoint7, Kp7a, Ki7a, Kd7a, DIRECT);


// Pin Assignments
const double PITCH = 0.003;   //m
const int GEARING = 2171;  //PPR
const double PULSETOMET = 1/(PITCH/GEARING); //723667
const int MOTOR1_PWM = 33;
const int MOTOR2_PWM = 32;
const int MOTOR3_PWM = 13, MOTOR3_IN1 = 14, MOTOR3_IN2 = 27; //Change 21 to 27 for ESP32 Dev
const int MOTOR4_IN1 = 17, MOTOR4_IN2 = 5;
const int MOTOR5_PWM = 23;
const int MOTOR6_PWM = 22;
const int MOTOR7_PWM = 12, MOTOR7_IN1 = 18, MOTOR7_IN2 = 19; //Change 21 to 27 for ESP32 Dev
const int MOTOR8_IN1 = 26, MOTOR8_IN2 = 25;

const int MOTOR3_CHA = 16, MOTOR3_CHB = 4;
const int MOTOR7_CHA = 2, MOTOR7_CHB = 15;


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

const int slow_mark = 70;

unsigned long previousMillis = 0;  // Store the last time sendJointPositions was called
const long interval = 100;          // Interval at which to send positions (100 ms)


const int MIN_DUTY_CYCLE = MAX_DUTY_CYCLE*MIN_PWM_WIDTH/PERIOD_MICROSECONDS; // Minimum pulse width in microseconds
const int FULL_DUTY_CYCLE = MAX_DUTY_CYCLE*MAX_PWM_WIDTH/PERIOD_MICROSECONDS; // Maximum pulse width in microseconds

volatile double m3_count = 0;         // Raw encoder count
int protectedCount3 = 0;          // Safe copy of count for main loop
int previousCount3 = 0;           // Track previous count for change detection

volatile double m7_count = 0;         // Raw encoder count for motor 7
int protectedCount7 = 0;           // Safe copy of count for main loop
int previousCount7 = 0;            // Track previous count for change detection

#define read3A bitRead(GPIO.in, MOTOR3_CHA)  // Faster than digitalRead()
#define read3B bitRead(GPIO.in, MOTOR3_CHB)  // Faster than digitalRead()

#define read7A bitRead(GPIO.in, MOTOR7_CHA)  // Faster than digitalRead()
#define read7B bitRead(GPIO.in, MOTOR7_CHB)  // Faster than digitalRead()

void isr3() {
  m3_count += (read3A == read3B) ? -1 : 1; // Original optimization
}

void isr7() {
  m7_count += (read7A == read7B) ? -1 : 1; // Original optimization
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
  // Serial.begin(115200, SERIAL_8N1, 1, 3, true);  // Enable RTS/CTS

  // Serial.println(MAX_DUTY_CYCLE);

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

  // pinMode(MOTOR7_PWM, OUTPUT);
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
  // attachInterrupt(digitalPinToInterrupt(MOTOR3_CHA), encoder_isr_3, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTOR3_CHB), encoder_isr_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_CHA), isr3, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTOR3_CHB), isr3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR7_CHA), isr7, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTOR7_CHB), isr7B, CHANGE);


  // Initialize PID Controllers
  pid3.SetTunings(Kp3a, Ki3a, Kd3a);
  pid3.SetTunings(Kp7a, Ki7a, Kd7a);
  pid3.SetMode(AUTOMATIC);
  pid3.SetOutputLimits(-MAX_DUTY_CYCLE, MAX_DUTY_CYCLE);
  pid7.SetMode(AUTOMATIC);
  pid7.SetOutputLimits(-MAX_DUTY_CYCLE, MAX_DUTY_CYCLE);
}


void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
  // Serial.println(PULSETOMET);

  // Update PID Control for Motors 3 and 7
  if (abs(input3 - setpoint3) < slow_mark) {
      pid3.SetTunings(Kp3b, Ki3b, Kd3b);
  }
  else if (abs(input3 - setpoint3) > slow_mark){
      pid3.SetTunings(Kp3a, Ki3a, Kd3a);
  }
  if (abs(input7 - setpoint7) < slow_mark) {
      pid7.SetTunings(Kp7b, Ki7b, Kd7b);
  }
  else if (abs(input7 - setpoint7) > slow_mark){
      pid7.SetTunings(Kp7a, Ki7a, Kd7a);
  }

  
  input3 = m3_count;
  pid3.Compute();
  setMotorDirection(MOTOR3_IN1, MOTOR3_IN2, output3);
  ledcWrite(PWM_CHANNEL_3, abs(output3)); // Use ledcWrite for ESP32 PWM


  input7 = m7_count;
  pid7.Compute();
  setMotorDirection(MOTOR7_IN1, MOTOR7_IN2, output7);
  ledcWrite(PWM_CHANNEL_7, abs(output7));
  // ledcWrite(MOTOR7_PWM, abs(output7)); // Use ledcWrite for ESP32 PWM

  // Send joint positions to Raspberry Pi
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Save the last time you sent positions
    sendJointPositions(); // Call your function to send joint positions
  }
  // Add a small delay to avoid WDT reset
  delay(10); // Adjust as necessary
}

void processCommand(String command) {
  //PID commands look like "PID<motor_number>,<Kp_value>,<Ki_value>,<Kd_value>"
  if (command.startsWith("PIDa")) {
    int motor = command.substring(4, 5).toInt();
    if (motor == 3){
      double Kp3a = command.substring(6, command.indexOf(',', 6)).toDouble();
      double Ki3a = command.substring(command.indexOf(',', 6) + 1, command.lastIndexOf(',')).toDouble();
      double Kd3a = command.substring(command.lastIndexOf(',') + 1).toDouble();
    }
    if (motor == 7){
      double Kp7a = command.substring(7, command.indexOf(',', 6)).toDouble();
      double Ki7a = command.substring(command.indexOf(',', 6) + 1, command.lastIndexOf(',')).toDouble();
      double Kd7a = command.substring(command.lastIndexOf(',') + 1).toDouble();
    }
  }
  else if (command.startsWith("PIDb")) {
    int motor = command.substring(4, 5).toInt();
    if (motor == 3){
      double Kp3b = command.substring(6, command.indexOf(',', 6)).toDouble();
      double Ki3b = command.substring(command.indexOf(',', 6) + 1, command.lastIndexOf(',')).toDouble();
      double Kd3b = command.substring(command.lastIndexOf(',') + 1).toDouble();
    }
    if (motor == 7){
      double Kp7b = command.substring(7, command.indexOf(',', 6)).toDouble();
      double Ki7b = command.substring(command.indexOf(',', 6) + 1, command.lastIndexOf(',')).toDouble();
      double Kd7b = command.substring(command.lastIndexOf(',') + 1).toDouble();
    }
  }
  else {
    String values[8];
    int motor = 0;
    int startIndex = 0;
    for (int i = 0; i < 8; i++) {
      int endIndex = command.indexOf(',', startIndex);
      values[i] = command.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
    }
    
    
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
    setpoint3 = values[2].toDouble()*PULSETOMET;
    setpoint7 = values[6].toDouble()*PULSETOMET;
    setMotorDirection(MOTOR4_IN1, MOTOR4_IN2, values[3].toInt());
    setMotorDirection(MOTOR8_IN1, MOTOR8_IN2, values[7].toInt());
  }
}

void setMotorDirection(int in1, int in2, int speed) {
  digitalWrite(in1, speed > 0);
  digitalWrite(in2, speed < 0);
}

void sendJointPositions() {
  double m3_pos = m3_count/PULSETOMET;
  double m7_pos = m7_count/PULSETOMET;
  // Serial.println(m3_pos,5);
  String positions = String(mapFloat(ledcRead(PWM_CHANNEL_1), MIN_DUTY_CYCLE, FULL_DUTY_CYCLE, 0.0, 270.0), 4) + ","
                       + String(mapFloat(ledcRead(PWM_CHANNEL_2), MIN_DUTY_CYCLE, FULL_DUTY_CYCLE, 0.0, 270.0), 4) + ","
                       + String(m3_pos, 4) + ",0,"
                       + String(mapFloat(ledcRead(PWM_CHANNEL_5), MIN_DUTY_CYCLE, FULL_DUTY_CYCLE, 0.0, 270.0), 4) + ","
                       + String(mapFloat(ledcRead(PWM_CHANNEL_6), MIN_DUTY_CYCLE, FULL_DUTY_CYCLE, 0.0, 270.0), 4) + ","
                       + String(m7_pos, 4) + ",0\n";
  Serial.print(positions);
  Serial.flush();
  yield();
}
