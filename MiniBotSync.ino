const int potPins[] = {A0, A1, A2, A3};  // Potentiometer pins
const int startingPos[] = {50, 0, 70, 0};
const int endingPos[] = {780, 333, 830, 650};
const int startingAngle[] = {0, 0, 0, 0};
const int endingAngle[] = {180, 90, 180, 180};
const int numPots = 4;
bool recording = false;
int newZeroPos;

#include <AccelStepper.h>
AccelStepper ShoulderStepper;
int stepsPerRevolution = 200 * 20 * 4;

#include <Servo.h>
Servo elbowServo;

// Variables for smoothing motor movements
long previousShoulderTarget = 0;
int previousElbowTarget = 90;
float shoulderSmoothingFactor = 0.1; // Lower for smoother but slower response
float elbowSmoothingFactor = 0.15;   // Adjust as needed

void setup() {
  Serial.begin(115200);  // Start serial communication
  pinMode(potPins[0], INPUT);
  pinMode(potPins[1], INPUT);
  pinMode(potPins[2], INPUT);
  pinMode(potPins[3], INPUT);
  Serial.println("Ready. Type 'START' to begin recording and 'STOP' to stop.");

  // Set reasonable speed and acceleration for smoother movement
  ShoulderStepper.setMaxSpeed(20000);
  ShoulderStepper.setAcceleration(2000.0);

  elbowServo.attach(12);

  zeroMotor();
  
  // Initialize previous targets
  int rawShoulder = analogRead(potPins[1]);
  int mappedShoulder = map(rawShoulder, startingPos[1], endingPos[1], startingAngle[1], endingAngle[1]);
  previousShoulderTarget = (mappedShoulder + newZeroPos) * stepsPerRevolution / 360;
  ShoulderStepper.setCurrentPosition(previousShoulderTarget);
  
  // Initialize servo position
  int rawElbow = analogRead(potPins[2]);
  int mappedElbow = map(rawElbow, startingPos[2], endingPos[2], startingAngle[2], endingAngle[2]);
  previousElbowTarget = map(mappedElbow, 0, 166, 180, 0);
  elbowServo.write(previousElbowTarget);
}

void zeroMotor(){
  bool zeroDone = 0;
  int lastpos;
  long pos;
  String input;
  while(!zeroDone){
    if(Serial.available() > 0) {
      Serial.println("Waiting for command");  

      input = Serial.readString();  // Takes input in degrees
      pos = input.toInt();
      Serial.print("Position commanded: ");
      Serial.println(pos);

      if(pos==0){
        zeroDone = 1;
        newZeroPos = lastpos;
      }
      else{
        ShoulderStepper.runToNewPosition(pos*stepsPerRevolution / 360);
        lastpos = pos;
      }
    }
  }
  Serial.println("Zeroing Done!!");
  Serial.println(newZeroPos);
}

void loop() {
  int angles[numPots];
  
  // Read potentiometer values directly without filtering
  for (int i = 0; i < numPots; i++) {
    int rawValue = analogRead(potPins[i]);
    angles[i] = map(rawValue, startingPos[i], endingPos[i], startingAngle[i], endingAngle[i]);
  }

  Serial.print("SHOULDER: ");
  Serial.println(angles[1]);
  Serial.print("ELBOW: ");
  Serial.println(angles[2]);

  // Smooth movement for shoulder stepper using exponential moving average
  long targetShoulderSteps = (angles[1] + newZeroPos) * stepsPerRevolution / 360;
  long newShoulderTarget = previousShoulderTarget + 
                          (targetShoulderSteps - previousShoulderTarget) * shoulderSmoothingFactor;
  
  // Set the target position and run the stepper (non-blocking)
  ShoulderStepper.moveTo(newShoulderTarget);
  ShoulderStepper.run();
  
  // Update previous target
  previousShoulderTarget = newShoulderTarget;

  // Constrain and smooth elbow movement
  int maxAngle = 163;
  if (angles[2] > maxAngle){
    angles[2] = maxAngle;
  }
  
  int targetElbowAngle = map(angles[2], 0, 166, 180, 0);
  int newElbowTarget = previousElbowTarget + 
                      (targetElbowAngle - previousElbowTarget) * elbowSmoothingFactor;
  
  elbowServo.write(newElbowTarget);
  previousElbowTarget = newElbowTarget;
  
  // Small delay to allow stepper to move
  delay(5);
  
  // Run the stepper a bit more to ensure it continues moving
  for (int i = 0; i < 5; i++) {
    ShoulderStepper.run();
  }
}