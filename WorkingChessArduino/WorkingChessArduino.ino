#include <Servo.h>
#include <AccelStepper.h>

float currentAngles[4] = {90, 90, 60, 1};  // Match your startup defaults
const float move_step_size = 10; // degrees per interpolation step


// SERIAL 
const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
float angles[4];  // Array to store the angles
float pos = 0;

// PIN DEFINITIONS
const uint8_t elbowServoPin = 12;
const uint8_t wristServoPin = 6; // WHITE
const uint8_t gripperServoPin = 7; // YELLOW
const uint8_t humerusLS_pin = 19; //Limit switch on the back of the humerus
const uint8_t base_cal_pin = 20;
const uint8_t allOtherSwitches = 21;


const int shoulderStepsPerRev = 200 * 20 * 4;  // change this to fit the number of steps per revolution (steps * gear ratio * 4)
const int baseStepsPerRev = 3200 * 4;  // change this to fit the number of steps per revolution (steps * gear ratio * 4)



float elbowSmoothingFactor = 0.15;   // Adjust as needed
float previousElbowTarget;

bool wristOpen = 1;
int wristCorrectionFactor = 0;

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
  angles[1] = 130;
  angles[2] = 50;
  angles[3] = 1; //Starts open

  // Initialize servo pin and stepper speeds and accelerations 
  pinMode(humerusLS_pin, INPUT_PULLUP);
  elbowServo.attach(elbowServoPin);  // attaches the servo on pin 9 to the Servo object
  elbowServo.write(110);
  wristServo.attach(wristServoPin);
  wristServo.write(45);
  gripperServo.attach(gripperServoPin);
  openGripper();
  baseStepper.setMaxSpeed(900);  // Steps per second (max value set by arduino clock speed)
  baseStepper.setAcceleration(600.0);  // Steps per secon^2 (6000 default)
  shoulderStepper.setMaxSpeed(1400);  // Steps per second (max value set by arduino clock speed) 5000
  shoulderStepper.setAcceleration(400.0);  // Steps per secon^2 (2000 default)

  shoulderStepper.setCurrentPosition(0);
  baseStepper.setCurrentPosition(0);

  previousElbowTarget = 180 - (map(angles[2], 0, 166, 0, 180) - 22);
  Serial.print("JUST SET:");
  Serial.println(previousElbowTarget);
  // elbowServo.write(previousElbowTarget);

  //Run calibration before limit switch becomes an interupt
  // Serial.println("Enter 0 to continue:");
  // while(1) {
  //   // Serial.println(digitalRead(humerusLS_pin));
  //   if (Serial.available() > 0) {
  //     char receivedChar = Serial.read();  // Read the incoming character
      
  //     // If the character received is '0', exit the loop and continue
  //     if (receivedChar == '0') {
  //       Serial.println("Received 0, continuing...");
  //       break;  
  //     }
  //   }
  // }
  calibrateBase();
  calibrateShoulder();

  baseStepper.runToNewPosition(-45.0 /360*baseStepsPerRev);  // Move to the zero position
    while(baseStepper.distanceToGo() != 0){}  // Wait for the stepper to finish moving

  // Set the hummerous outside limit switch to an interupt pin so it will have imediate control
  delay(2000);
//  attachInterrupt(digitalPinToInterrupt(humerusLS_pin), limit_ISR, LOW);
//  attachInterrupt(digitalPinToInterrupt(base_cal_pin), limit_ISR, LOW);
//  attachInterrupt(digitalPinToInterrupt(allOtherSwitches), limit_ISR, LOW);

  setMotors(angles);
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

    if(angles[0] < 45 || angles[0] > 250){
    } else if(angles[1] < 35 || angles[1] > 150) {
    } else {
      setMotors(angles); // Set motors to the given angles
    }
    
    
  }
   
}



void printMotorAngles(float angles[4]) {

  Serial.print("Motor angle = ");
  Serial.print(angles[0]);  // base
  Serial.print(", ");
  Serial.print(angles[1]); // shoulder
  Serial.print(", ");
  Serial.print(angles[2]); // elbow
  Serial.print(", ");
  Serial.println(angles[3]);  // gripper

}

 void setMotors(float angles[4]) {


    wristOpen = angles[3];

    // If the gripper needs to open, do that first
    if(angles[3] == 1){
      Serial.println("OPENING GRIPPER");
      openGripper();
    }
    
    printMotorAngles(angles);   // Print the angles the motor is going to
    Serial.print("Writing to Elbow: ");
    Serial.println(angles[2]);
    // === ELBOW SERVO SMOOTH SETUP ===
    float elbowTarget = map(angles[2], 0, 166, 0, 180) - 22;
    elbowTarget = 180 - elbowTarget;  // Final mapped angle
  
    float elbowCurrent = elbowServo.read();
    float elbowDirection = (elbowTarget > elbowCurrent) ? 1 : -1;
    float elbowStepsToGo = abs(elbowTarget - elbowCurrent);
    
    unsigned long lastUpdateTime = millis();
    const int elbowStepDelay = 10; // ms between servo steps

    Serial.println();
    Serial.println("//////////////////////////////");
    Serial.print(baseStepper.currentPosition() * 360/baseStepsPerRev);
    Serial.print(", ");
    Serial.println(shoulderStepper.currentPosition() * 360/shoulderStepsPerRev);
    Serial.println("/////////////////////////////");
    
    // Map the base angles
    Serial.print("Writing to Base: ");
    Serial.println(angles[0]);
    pos = map(angles[0],90,270,-45,-225) - 2;
    baseStepper.moveTo(pos * baseStepsPerRev/360);
    
    //Map the shoulder
    Serial.print("Writing to Shoulder: ");
    Serial.println(angles[1]);
    pos = map(angles[1],0,180,-180,0) + 5;
    shoulderStepper.moveTo(pos * shoulderStepsPerRev/360);
    
    Serial.print("Writing to wrist: ");
    float wristAngle = calculateWristAngle(pos, angles[2]);
    float wristTarget = wristAngle;
    float wristCurrent = wristServo.read();
    float wristDirection = (wristAngle > wristCurrent) ? 1 : -1;
    float wristStepsToGo = abs(wristAngle - wristCurrent);
    const int wristStepDelay = 10; // ms between servo steps
    
    Serial.println(wristAngle);
    //wristServo.write(wristAngle);
    
    // Calculate maximum number of steps needed by any motor
    long shoulderStepsToGo = shoulderStepper.distanceToGo() / shoulderStepper.speed();
    long baseStepsToGo = baseStepper.distanceToGo() / baseStepper.speed();
    long maxSteps = max(abs(baseStepsToGo), max(abs(shoulderStepsToGo), elbowStepsToGo));
    
    if (maxSteps == 0) {
      if (angles[3] == 0){
        Serial.println("CLOSING GRIPPER");
        closeGripper();
      }
      wristServo.write(wristAngle);
      Serial.println("Done with move");
      return; // No movement needed
    }
    
    // Calculate ratios to synchronize movements
    float baseRatio = (maxSteps > 0 && baseStepsToGo != 0) ? abs(baseStepsToGo) / (float)maxSteps : 0;
    float shoulderRatio = (maxSteps > 0 && shoulderStepsToGo != 0) ? abs(shoulderStepsToGo) / (float)maxSteps : 0;
    float elbowRatio = (maxSteps > 0 && elbowStepsToGo > 0) ? elbowStepsToGo / (float)maxSteps : 0;
    float wristRatio = (maxSteps > 0 && wristStepsToGo > 0) ? wristStepsToGo / (float)maxSteps : 0;
    
    // Counters for fractional steps
    unsigned long lastElbowUpdateTime = millis();
    unsigned long lastWristUpdateTime = lastElbowUpdateTime;
  
    // Main movement loop
    while (baseStepper.distanceToGo() != 0 || shoulderStepper.distanceToGo() != 0 || elbowCurrent != elbowTarget || wristCurrent != wristTarget) {
      unsigned long currentTime = millis();
      
      // Base stepper movement
      if (baseStepper.distanceToGo() != 0) {
        for(int i=0; i < baseRatio; i++) {
          baseStepper.run();
        }
      }

       // Elbow servo movement
      if (elbowCurrent != elbowTarget && currentTime - lastElbowUpdateTime >= elbowStepDelay) {

        if(abs(elbowCurrent - elbowTarget) < 3) {
          elbowCurrent = elbowTarget;
        }
        else {
         elbowCurrent += elbowDirection/2 * elbowRatio / 1.8;
        }
  
        elbowServo.write(elbowCurrent);
  
        lastElbowUpdateTime = currentTime;
      }
      
      // Shoulder stepper movement
      if (shoulderStepper.distanceToGo() != 0) {
        for(int i=0; i < shoulderRatio; i++) {
          shoulderStepper.run();
        }
      }
      
      // // Elbow servo movement
      //  if (elbowCurrent != elbowTarget && currentTime - lastElbowUpdateTime >= elbowStepDelay) {

      //   if(abs(elbowCurrent - elbowTarget) < 3) {
      //     elbowCurrent = elbowTarget;
      //   }
      //   else {
      //    elbowCurrent += elbowDirection/2 * elbowRatio;
      //   }
  
      //   elbowServo.write(elbowCurrent);
  
      //   lastElbowUpdateTime = currentTime;
      // }

      if (wristCurrent != wristTarget && currentTime - lastWristUpdateTime >= wristStepDelay) {

        if(abs(wristCurrent - wristTarget) < 3) {
          wristCurrent = wristTarget;
        }
        else {
         wristCurrent += wristDirection * wristRatio;
        }
  
        wristServo.write(wristCurrent);
  
        lastWristUpdateTime = currentTime;
      }

//      if (wristCurrent != wristAngle && currentTime - lastElbowUpdateTime >= elbowStepDelay) {
//        wristCounter += wristRatio;
//        if (wristCounter >= 1.0) {
//          wristCounter -= 1.0;
//          wristCurrent += wristDirection;
//          wristServo.write(wristCurrent);
//        }
//      }
      
      // Small delay to prevent hogging CPU
      delayMicroseconds(100);
    }

    wristServo.write(wristAngle);

    // Close the gripper if nneded
    if (angles[3] == 0){
      Serial.println("CLOSING GRIPPER");
      closeGripper();
    }
  
    Serial.println("Done with move");
}



void openGripper(){
  //wristServo.write(wristServo.read() - 10);
  gripperServo.write(160);
  wristOpen = 1;
}



void closeGripper(){
  //wristServo.write(wristServo.read() + 10);
  gripperServo.write(20);
  wristOpen = 0;
}



void calibrateShoulder() {
 
  // Serial.print("Arm cal pin: " + String(digitalRead(humerusLS_pin)));
  // Serial.print("Base cal pin: ");
  // Serial.println(digitalRead(base_cal_pin));

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

      shoulderStepper.setSpeed(shoulderStepper.maxSpeed());
      
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
  baseStepper.setSpeed(300);

  while(1){
    baseStepper.runSpeed();

    if (digitalRead(base_cal_pin) == 0) {
      // if limit switch is triggered, stop
      baseStepper.stop();
      Serial.println("Stopping base calibration");

      baseStepper.setCurrentPosition(0);  // Set the current position as zero
      Serial.print("Base calibration complete at position: ");
      Serial.println(baseStepper.currentPosition() * 360/baseStepsPerRev);

      // resents the base to its max speed
      baseStepper.setSpeed(baseStepper.maxSpeed());
      
      Serial.println("Exiting base calibration loop");
      return;
   }
 }
  setMotors(angles);
}



float calculateWristAngle(float shoulderAngle, float elbowAngle) {
  // Ensure the wrist remains vertical by compensating for the shoulder and elbow angles
  float prewristAngle = 90 - (shoulderAngle - 180 + elbowAngle);
  float wristAngle = map(prewristAngle, -180, 0, 170, -10) - 5;

  // Normalize the wrist angle to stay within 0-180 degrees for the servo
  if (wristAngle < 0) {
    wristAngle += 360;
  }
  wristAngle = fmod(wristAngle, 360); // Keep it within 0-360
  if (wristAngle > 180) {
    wristAngle -= 360; // Adjust for servo range
  }

  if (wristOpen == 1) {
    wristAngle -= wristCorrectionFactor;
  }
  // else{
  //   wristAngle += wristCorrectionFactor;
  // }


  return wristAngle;
}


void limit_ISR() {
  Serial.println("MADE IT TO ISR");
  shoulderStepper.stop();
  baseStepper.stop();
  Serial.println("STOPPED");
  while(1){}
}
