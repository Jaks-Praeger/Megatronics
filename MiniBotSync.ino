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

const int filterSize = 10;
int angleBuffer[numPots][filterSize] = {0};
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);  // Start serial communication
  pinMode(potPins[0],INPUT);
  pinMode(potPins[1],INPUT);
  pinMode(potPins[2],INPUT);
  pinMode(potPins[3],INPUT);
  Serial.println("Ready. Type 'START' to begin recording and 'STOP' to stop.");

  ShoulderStepper.setMaxSpeed(50000);
  ShoulderStepper.setAcceleration(4000.0);

  elbowServo.attach(12);

  zeroMotor();
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
  for (int i = 0; i < numPots; i++) {
    int rawValue = analogRead(potPins[i]);
    int mappedAngle = map(rawValue, startingPos[i], endingPos[i], startingAngle[i], endingAngle[i]);  // Convert to angle
    if (i==2){
      angles[i] = getFilteredAngle(i, mappedAngle); // Filter!!
    } else{
      angles[i] = mappedAngle;
    }
  }
  //Serial.print("BASE: ");
  //Serial.println(angles[0]);
  Serial.print("SHOULDER: ");
  Serial.println(angles[1]);
  Serial.print("ELBOW: ");
  Serial.println(angles[2]);
  //Serial.print("WRIST: ");
  //Serial.println(angles[3]);

  long shoulderAngle = angles[1] + newZeroPos;
  ShoulderStepper.runToNewPosition(shoulderAngle*stepsPerRevolution / 360);

  int maxAngle = 163;
  if (angles[2] > maxAngle){
    angles[2] = maxAngle;
  }
  long elbowAngle = map(angles[2], 0, 166, 180, 0);
  elbowServo.write(elbowAngle);
  
  bufferIndex = (bufferIndex + 1) % filterSize;
  // delay(100); // Small delay to reduce data rate
}

int getFilteredAngle(int potIndex, int newValue) {
  angleBuffer[potIndex][bufferIndex] = newValue;
  int sum = 0;
  for (int i = 0; i < filterSize; i++) {
    sum += angleBuffer[potIndex][i];
  }
  return sum / filterSize;
}
