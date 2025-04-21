#include <Servo.h>
#include <AccelStepper.h>

float currentAngles[4] = {90, 90, 60, 1};  // Match your startup defaults
const float move_step_size = 2.0; // degrees per interpolation step


// SERIAL 
const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
float angles[4];  // Array to store the angles
float pos = 0;

// PIN DEFINITIONS
const int elbowServoPin = 12;
const int wristServoPin = 6; // WHITE
const int gripperServoPin = 7; // YELLOW
const int humerusLS_pin = 21; //Limit switch on the back of the humerus
const int base_cal_pin = 20;

const int shoulderStepsPerRev = 200 * 20 * 4;  // change this to fit the number of steps per revolution (steps * gear ratio * 4)
const int baseStepsPerRev = 3200 * 4;  // change this to fit the number of steps per revolution (steps * gear ratio * 4)

const int allOtherSwitches = 19;


float elbowSmoothingFactor = 0.15;   // Adjust as needed
float previousElbowTarget;

//Initialize stepper and servo motors
AccelStepper baseStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper shoulderStepper(AccelStepper::FULL4WIRE, 8, 9, 10, 11);
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

void setup() {
  Serial.begin(9600);
  //Serial.setTimeout(200);

  angles[0] = 90;
  angles[1] = 90;
  angles[2] = 60;
  angles[3] = 1; //Starts open

  // Initialize servo pin and stepper speeds and accelerations 
  elbowServo.attach(elbowServoPin);  // attaches the servo on pin 9 to the Servo object
  elbowServo.write(130);
  wristServo.attach(wristServoPin);
  gripperServo.attach(gripperServoPin);
  baseStepper.setMaxSpeed(800);  // Steps per second (max value set by arduino clock speed)
  baseStepper.setAcceleration(400.0);  // Steps per secon^2 (6000 default)
  shoulderStepper.setMaxSpeed(700);  // Steps per second (max value set by arduino clock speed) 5000
  shoulderStepper.setAcceleration(200.0);  // Steps per secon^2 (2000 default)

  shoulderStepper.setCurrentPosition(0);
  baseStepper.setCurrentPosition(0);

  previousElbowTarget = 180 - (map(angles[2], 0, 166, 0, 180) - 22);
  Serial.print("JUST SET:");
  Serial.println(previousElbowTarget);
  elbowServo.write(previousElbowTarget);

  // Run calibration before limit switch becomes an interupt
  pinMode(humerusLS_pin, INPUT_PULLUP);
  calibrateShoulder();
  calibrateBase();

  // Set the hummerous outside limit switch to an interupt pin so it will have imediate control
  delay(2000);
//  attachInterrupt(digitalPinToInterrupt(humerusLS_pin), limit_ISR, LOW);
//  attachInterrupt(digitalPinToInterrupt(base_cal_pin), limit_ISR, LOW);
//  attachInterrupt(digitalPinToInterrupt(allOtherSwitches), limit_ISR, LOW);

  Serial.println("Arduino ready");

  Serial.println("Calibration complete");
}


void loop() {
  if (Serial.available() > 0) {
    // Read incoming data until newline
    int bytesRead = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
    buffer[bytesRead] = '\0';  // Null terminate the string
  
    Serial.println(buffer); // Debug: show raw input
  
    // Optional: Remove first character if needed (e.g., a leading character)
    // For example, if you want to remove the first char:
    // memmove shifts the string one position left
    memmove(buffer, buffer + 1, strlen(buffer));
  
    // Tokenize and convert to float
    char* token = strtok(buffer, ",");
    int i = 0;
  
    while (token != NULL && i < 4) {
      angles[i] = atof(token);
      i++;
      token = strtok(NULL, ",");
    }
    
    // Set motors to the given angles
    interpolatedMove(angles);
  }
   
}

void interpolatedMove(float targetAngles[4]) {
  // 1. Calculate max angle change
  float maxChange = 0;
  for (int i = 0; i < 3; i++) { // Only joints 0-2 (gripper is binary)
    float diff = abs(targetAngles[i] - currentAngles[i]);
    if (diff > maxChange) {
      maxChange = diff;
    }
  }

  // 2. Determine number of interpolation steps
  int numSteps = max(1, int(maxChange / move_step_size));

  // 3. Interpolate and move
  for (int step = 1; step <= numSteps; step++) {
    float stepAngles[4];
    for (int i = 0; i < 3; i++) {
      stepAngles[i] = currentAngles[i] + (targetAngles[i] - currentAngles[i]) * ((float)step / numSteps);
    }
    stepAngles[3] = targetAngles[3]; // Gripper is not interpolated

    setMotors(stepAngles);
  }

  // 4. Update current angles
  for (int i = 0; i < 4; i++) {
    currentAngles[i] = targetAngles[i];
  }
  Serial.println("Done with move");
}


void printMotorAngles(float angles[4]) {

  Serial.print("Motor angle = ");
  Serial.print(angles[0]);
  Serial.print(", ");
  Serial.print(angles[1]);
  Serial.print(", ");
  Serial.print(angles[2]);
  Serial.print(", ");
  Serial.println(angles[3]);

}

 void setMotors(float angles[4]) {

  printMotorAngles(angles);

  Serial.print("Writing to Elbow: ");
  Serial.println(angles[2]);
  // === ELBOW SERVO SMOOTH SETUP ===
  float elbowTarget = map(angles[2], 0, 166, 0, 180) - 22;
  elbowTarget = 180 - elbowTarget;  // Final mapped angle
  
  float newElbowTarget = previousElbowTarget + 
                      (elbowTarget - previousElbowTarget) * elbowSmoothingFactor;
  Serial.println(previousElbowTarget);
  Serial.println(newElbowTarget);
  previousElbowTarget = newElbowTarget;
  
  float elbowCurrent = elbowServo.read();
  float elbowDirection = (newElbowTarget > elbowCurrent) ? 1 : -1;
  unsigned long lastUpdateTime = millis();
  const int elbowStepDelay = 10; // ms between servo steps

  Serial.println("//////////////////////////////");
  Serial.print(baseStepper.currentPosition() * 360/baseStepsPerRev);
  Serial.print(", ");
  Serial.println(shoulderStepper.currentPosition() * 360/shoulderStepsPerRev);
  Serial.println("/////////////////////////////");

  Serial.print("Writing to Base: ");
  Serial.println(angles[0]);
  pos = map(angles[0],90,270,0,-180) - 2;
  baseStepper.moveTo(pos * baseStepsPerRev/360);

  Serial.print("Writing to Shoulder: ");
  Serial.println(angles[1]);
  pos = map(angles[1],0,180,-180,0) + 7;
  shoulderStepper.moveTo(pos * shoulderStepsPerRev/360);

  Serial.print("Writing to wrist: ");
  float wristAngle = calculateWristAngle(pos, angles[2]);
  Serial.println(wristAngle);
  wristServo.write(wristAngle);

  while (baseStepper.distanceToGo() || shoulderStepper.distanceToGo() || elbowCurrent != newElbowTarget){
    
    if(baseStepper.distanceToGo()) {
      baseStepper.run();  
    }
    if(shoulderStepper.distanceToGo()) {
     shoulderStepper.run(); 
    }
    // Smooth elbow movement
    if (millis() - lastUpdateTime >= elbowStepDelay && elbowCurrent != newElbowTarget) {
      elbowCurrent += elbowDirection;
      elbowServo.write(elbowCurrent);
      lastUpdateTime = millis();
    }
  }

  if(angles[3] == 1){
    Serial.println("OPENING GRIPPER");
    openGripper();
  }
  if (angles[3] == 0){
    Serial.println("CLOSING GRIPPER");
    closeGripper();
  }
}

void openGripper(){
  gripperServo.write(180);
}

void closeGripper(){
  gripperServo.write(0);
}

void calibrateShoulder() {

 Serial.println("Enter 0 to continue:");

// while(1) {
//  Serial.print("Arm cal pin: " + String(digitalRead(humerusLS_pin)));
//  Serial.print("Base cal pin: ");
//  Serial.println(digitalRead(base_cal_pin));
//   if (Serial.available() > 0) {
//     char receivedChar = Serial.read();  // Read the incoming character
//     
//     // If the character received is '0', exit the loop and continue
//     if (receivedChar == '0') {
//       Serial.println("Received 0, continuing...");
//       break;  
//     }
//   }
// }

  elbowServo.write(90);

  // rotate stepper motor backwards at half speed
  shoulderStepper.setSpeed(650);

  Serial.println("Final Arm cal pin: " + String(digitalRead(humerusLS_pin)));

  while(1) {
    //Serial.println(digitalRead(humerusLS_pin));
    shoulderStepper.runSpeed();

    if (digitalRead(humerusLS_pin) == 0) {
      // if limit switch is triggered, stop
      shoulderStepper.stop();
      Serial.println("Stopping");

      // Set angle to whatever angle the arm is when the switch is triggered
      shoulderStepper.setCurrentPosition(0);  // measure this angle based on the current setup
      Serial.print("Current position: ");
      Serial.println(shoulderStepper.currentPosition() * 360/shoulderStepsPerRev);


      // have the arm go to the upright position
      shoulderStepper.setSpeed(1000);
      shoulderStepper.runToNewPosition(-83.0 * shoulderStepsPerRev/360);


      while(shoulderStepper.isRunning()){}

      Serial.println(shoulderStepper.currentPosition() * 360/shoulderStepsPerRev);
      
      return;
    }
    else if(shoulderStepper.currentPosition() >= shoulderStepsPerRev/2 || shoulderStepper.currentPosition() <= -1 * shoulderStepsPerRev/2) {
      // if the arm has done more than a 180, stop
      shoulderStepper.stop();

      Serial.println("Calibration Failed");
      return;
    }
  }


}

void calibrateBase() {
  Serial.println("Calibrating base...");
//  baseStepper.setSpeed(150);
//  while(1){
//    baseStepper.runSpeed();
//    if (digitalRead(base_cal_pin) == 0) {
//      // if limit switch is triggered, stop
//      baseStepper.stop();
//      Serial.println("Stopping base calibration");
      baseStepper.setCurrentPosition(0);  // Set the current position as zero
      Serial.print("Base calibration complete at position: ");
      Serial.println(baseStepper.currentPosition() * 360/baseStepsPerRev);
//      baseStepper.setSpeed(250);
//      baseStepper.runToNewPosition(-45 /360*baseStepsPerRev);  // Move to the zero position
//      while(baseStepper.distanceToGo() != 0){}  // Wait for the stepper to finish moving
//      delay(4000);
//      Serial.println("Exiting base calibration loop");
//      return;
//    }
//  }
  setMotors(angles);
}

float calculateWristAngle(float shoulderAngle, float elbowAngle) {
  // Ensure the wrist remains vertical by compensating for the shoulder and elbow angles
  float prewristAngle = 90 - (shoulderAngle - 180 + elbowAngle);
  float wristAngle = map(prewristAngle, -180, 0, 175, -5);

  // Normalize the wrist angle to stay within 0-180 degrees for the servo
  if (wristAngle < 0) {
    wristAngle += 360;
  }
  wristAngle = fmod(wristAngle, 360); // Keep it within 0-360
  if (wristAngle > 180) {
    wristAngle -= 360; // Adjust for servo range
  }

  return wristAngle;
}

void limit_ISR() {
  Serial.println("MADE IT TO ISR");
  shoulderStepper.stop();
  baseStepper.stop();
  Serial.println("STOPPED");
  while(1){}
}
