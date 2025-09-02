//pins
#define RIGHT_MOTOR_PWM_D 3
#define RIGHT_MOTOR_PWM 11
#define LEFT_MOTOR_PWM_D 9
#define LEFT_MOTOR_PWM 10
#define RX 8
#define TX 12

//config
#define SERIAL_BAUD 38400 // bluetooth module is configured at BAUD6 which represents 38400 bps
#define MOTORS_DEADTIME 20 // time in miliseconds. Delay of the motors before switching directions (backward and forward)
#define MOTORS_MIN_DUTY_PERC 50 // minimum PWM in percent for motor to start
#define MAX_SPEED_KMH 6 // maximum car speed in km/h

#include <Arduino.h>
#include <GyverMotor2.h>
// DRIVER2WIRE_PWM_POWER - two wire driver with two PWM pins - torque mode
GMotor2<DRIVER2WIRE_PWM_POWER> motorR(RIGHT_MOTOR_PWM_D, RIGHT_MOTOR_PWM); // right motor
GMotor2<DRIVER2WIRE_PWM_POWER> motorL(LEFT_MOTOR_PWM_D, LEFT_MOTOR_PWM); // left motor

#include <SoftwareSerial.h>
SoftwareSerial BTSerial(RX, TX); // RX, TX Bluetooth

#include "processCommnad.h" // util function to calculate motors duty

int lastDutyR = 0; // variables to calculate car speed before sending it to client
int lastDutyL = 0;
unsigned long previousMillis = 0; // variable for keep track of time
unsigned long lastCommandTime = 0; // remember when last command was sent to the client
const unsigned long SEND_INTERVAL = 200;     // interval of sending data about battery level and speed to client
const unsigned long AUTO_STOP_TIMEOUT = 700; // timeout after we stop motors if no data was received from client

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

  // turn off the LED - optional
  digitalWrite(LED_BUILTIN, LOW);

  delay(3000); // in case for all modules to turn on before go to loop
}


void loop() {
  unsigned long now = millis();

  if (BTSerial.available()) { // if there is data available from Bluetooth
    String input = BTSerial.readStringUntil('\n'); // Read complete command. Command format [F|B]nn[R|L]nn n=0..9 Example F67L30, B12R99
    input.trim(); // Remove unwanted spaces or newlines
    Serial.println(input); //debug

    // ignore if the incoming command is not our format
    if (input.length() == 6) {
      lastCommandTime = now;

      MotorDuty motorDuty = processCommand(input);
      Serial.println("L: " + String(motorDuty.dutyL));
      Serial.println("R: " + String(motorDuty.dutyR));

      lastDutyR = motorDuty.dutyR;
      lastDutyL = motorDuty.dutyL;

      motorL.setSpeedPerc(motorDuty.dutyL);
      motorR.setSpeedPerc(motorDuty.dutyR);
    }
  }

  // send data about battery level and speed to client every SEND_INTERVAL milliseconds
  if (now - previousMillis >= SEND_INTERVAL) {
    previousMillis = now;

   // motorL.getSpeed() + motorR.getSpeed();
    int avgDuty = (lastDutyR + lastDutyL) / 2;
    int speedKmh = abs(avgDuty) * MAX_SPEED_KMH / MOTOR_MAX;

    BTSerial.println("BAT:75,SPEED:" + String(speedKmh));
  }

  // autostop if no command is received for a AUTO_STOP_TIMEOUT period
  if (millis() - lastCommandTime > AUTO_STOP_TIMEOUT) {
    motorL.brake();
    motorR.brake();
    lastDutyR = 0;
    lastDutyL = 0;
  }
}
