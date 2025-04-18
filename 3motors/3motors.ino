#include <Servo.h>
#include <AccelStepper.h>

const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
float angles[3];  // Array to store the three angles
float pos = 0;
const int servoPin = 10;
int stepsPerRevolution = 200 * 20 * 4;  // change this to fit the number of steps per revolution (steps * gear ratio * 4)

AccelStepper baseStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper shoulderStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
Servo elbowServo;


void setup() {
  Serial.begin(9600);
  //Serial.setTimeout(200);

  elbowServo.attach(servoPin);  // attaches the servo on pin 9 to the Servo object
  baseStepper.setMaxSpeed(100000);  // Steps per second (max value set by arduino clock speed)
  baseStepper.setAcceleration(6000.0);  // Steps per secon^2 (6000 default)
  shoulderStepper.setMaxSpeed(100000);  // Steps per second (max value set by arduino clock speed)
  shoulderStepper.setAcceleration(6000.0);  // Steps per secon^2 (6000 default)

  // NEED to write callibration code
  //calibrateSteppers();

  Serial.println("Arduino ready");
}


void loop() {
  if (Serial.available() > 0) {
    // Read incoming data until newline
    int bytesRead = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
    buffer[bytesRead] = '\0';  // Null terminate
   
    // Parse the three comma-separated values
    char* token = strtok(buffer, ",");
    for (int i = 0; i < 3 && token != NULL; i++) {
      angles[i] = atof(token);
      token = strtok(NULL, ",");
    }
  
    // Set motors to the given angles
    setMotors(angles);
   
  }
}

void printMotorAngles(float angles[3]) {

  Serial.print("Motor angle = ");
  Serial.print(angles[0]);
  Serial.print(", ");
  Serial.print(angles[1]);
  Serial.print(", ");
  Serial.println(angles[2]);

}

void setMotors(float angles[3]) {

  printMotorAngles(angles);

  baseStepper.runToNewPosition(angles[0] * stepsPerRevolution/360);
  shoulderStepper.runToNewPosition(angles[1] * stepsPerRevolution/360);

  pos = map(angles[2], 0, 166, 0, 180);   // Linearly maps 0-166 onto 0-180 for the servo
  pos = pos + 6;  // adds 6 for the constant offset
  elbowServo.write(180 - pos);
}



void calibrateMotors() {

  baseStepper.currentPosition();
  baseStepper.setCurrentPosition(0);


}





