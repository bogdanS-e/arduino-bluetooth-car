// pins
#define RIGHT_MOTOR_PWM_D 3
#define RIGHT_MOTOR_PWM 11
#define LEFT_MOTOR_PWM_D 9
#define LEFT_MOTOR_PWM 10
#define RX 8
#define TX 12
#define HEADLIGHT 7
#define BACKLIGHT 2

// config
#define SERIAL_BAUD 38400       // bluetooth module is configured at BAUD6 which represents 38400 bps
#define MOTORS_DEADTIME 20      // time in miliseconds. Delay of the motors before switching directions (backward and forward)
#define MOTORS_MIN_DUTY_PERC 25 // minimum PWM in percent for motor to start
#define MAX_SPEED_KMH 6         // maximum car speed in km/h

#include <Arduino.h>
#include <GyverMotor2.h>
// DRIVER2WIRE_PWM_POWER - two wire driver with two PWM pins - torque mode
GMotor2<DRIVER2WIRE_PWM_POWER> motorR(RIGHT_MOTOR_PWM_D, RIGHT_MOTOR_PWM); // right motor
GMotor2<DRIVER2WIRE_PWM_POWER> motorL(LEFT_MOTOR_PWM_D, LEFT_MOTOR_PWM);   // left motor

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(RX, TX); // RX, TX Bluetooth

#include "getDuty.h" // util function to calculate motors duty

int lastDutyR = 0; // variables to calculate car speed before sending it to client
int lastDutyL = 0;
int motorMax = 50;                           // max speed of the motors in percent
unsigned long previousMillis = 0;            // variable for keep track of time
unsigned long lastCommandTime = 0;           // remember when last command was sent to the client
const unsigned long SEND_INTERVAL = 200;     // interval of sending data about battery level and speed to client
const unsigned long AUTO_STOP_TIMEOUT = 700; // timeout after we stop motors if no data was received from client

void processCommand(char command);

void setup() {
  // enable fast PWM on pins D3, D9, D10, D11
  // Pins D3 и D11 62.5 кГц PWM 10bit
  TCCR2B = 0b00000001;
  TCCR2A = 0b00000011;

  // D9 и D10 62.5 kHz PWM 10bit
  TCCR1A = 0b00000001;
  TCCR1B = 0b00001001;

  // PWM phase correct in case if need to go to default values
  /*
  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00000011;
  TCCR2B = 0b00000100;
  TCCR2A = 0b00000001;
  */

  BTSerial.begin(SERIAL_BAUD);
  Serial.begin(SERIAL_BAUD);

  motorR.setDeadtime(MOTORS_DEADTIME);
  motorL.setDeadtime(MOTORS_DEADTIME);

  motorR.setMinDutyPerc(MOTORS_MIN_DUTY_PERC); // right motor is faster
  motorL.setMinDutyPerc(MOTORS_MIN_DUTY_PERC);

  pinMode(HEADLIGHT, OUTPUT);
  pinMode(BACKLIGHT, OUTPUT);

  delay(1000); // in case for all modules to turn on before go to loop
}

void loop() {
  unsigned long now = millis();

  if (BTSerial.available()) {                      // if there is data available from Bluetooth
    String input = BTSerial.readStringUntil('\n'); // Read complete command. Command format [F|B]nn[R|L]nn n=0..9 Example F67L30, B12R99
    input.trim();                                  // Remove unwanted spaces or newlines

    // ignore if the incoming command is not our format
    if (input.length() == 6) {
      lastCommandTime = now;

      MotorDuty motorDuty = getDuty(input, motorMax);

      lastDutyR = motorDuty.dutyR;
      lastDutyL = motorDuty.dutyL;

      motorL.setSpeedPerc(motorDuty.dutyL);
      motorR.setSpeedPerc(motorDuty.dutyR);
    } else if (input.length() == 1) {
      const char functionCommand = input[0];
      lastCommandTime = now;

      processCommand(functionCommand);
    }
  }

  // send data about battery level and speed to client every SEND_INTERVAL milliseconds
  if (now - previousMillis >= SEND_INTERVAL) {
    previousMillis = now;

    // motorL.getSpeed() + motorR.getSpeed();
    int avgDuty = (lastDutyR + lastDutyL) / 2;
    int speedKmh = abs(avgDuty) * MAX_SPEED_KMH / motorMax;

    // BTSerial.println("BAT:75,SPEED:" + String(speedKmh));
  }

  // autostop if no command is received for a AUTO_STOP_TIMEOUT period
  if (millis() - lastCommandTime > AUTO_STOP_TIMEOUT) {
    motorL.brake();
    motorR.brake();
    lastDutyR = 0;
    lastDutyL = 0;
  }
}

void processCommand(char command) {
  switch (command) {
  case 'F': // Move forward
    Serial.println("SERIAL_BAUD");
    motorL.setSpeedPerc(motorMax);
    motorR.setSpeedPerc(motorMax);
    break;
  case 'B': // Move backward
    motorL.setSpeedPerc(-motorMax);
    motorR.setSpeedPerc(-motorMax);
    break;
  case 'R': // Turn right
    motorL.setSpeedPerc(motorMax / 2);
    motorR.setSpeedPerc(-motorMax / 2);
    break;
  case 'L': // Turn left
    motorL.setSpeedPerc(-motorMax / 2);
    motorR.setSpeedPerc(motorMax / 2);
    break;
  case 'G': // Forward left
    motorL.setSpeedPerc(motorMax / 4);
    motorR.setSpeedPerc(motorMax);
    break;
  case 'H': // Forward right
    motorL.setSpeedPerc(motorMax);
    motorR.setSpeedPerc(motorMax / 4);
    break;
  case 'I': // Backward left
    motorL.setSpeedPerc(-motorMax / 4);
    motorR.setSpeedPerc(-motorMax);
    break;
  case 'J': // Backward right
    motorR.setSpeedPerc(-motorMax / 4);
    motorL.setSpeedPerc(-motorMax);
    break;
  case 'S':
    motorL.stop();
    motorR.stop();
    break;

  case 'U': // Turn headlight ON
    digitalWrite(HEADLIGHT, HIGH);
    break;
  case 'u': // Turn headlight OFF
    digitalWrite(HEADLIGHT, LOW);
    break;
  case 'V': // Turn backlight ON
    digitalWrite(BACKLIGHT, HIGH);
    break;
  case 'v': // Turn backlight OFF
    digitalWrite(BACKLIGHT, LOW);
    break;

  case '1':
    motorMax = 50;
    break;
  case '2':
    motorMax = 70;
    break;
  case '3':
    motorMax = 80;
    break;
  case '4':
    motorMax = 100;
    break;
  }
}
