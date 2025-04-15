#include <Servo.h>
#include <AccelStepper.h>

// SERIAL 
const int BUFFER_SIZE = 64;
char buffer[BUFFER_SIZE];
float angles[5];  // Array to store the three angles
float pos = 0;

// PIN DEFINITIONS
const int elbowServoPin = 11;
const int wristServoPin = 13;
const int gripperServoPin = 12;
const int humerusLS_pin = 10; //Limit switch on the back of the humerus

const int shoulderStepsPerRev = 200 * 20 * 4;  // change this to fit the number of steps per revolution (steps * gear ratio * 4)
const int baseStepsPerRev = 800 * 4;  // change this to fit the number of steps per revolution (steps * gear ratio * 4)

//Initialize stepper and servo motors
AccelStepper baseStepper(AccelStepper::FULL4WIRE, 2, 3, 4, 5); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper shoulderStepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
Servo elbowServo;
Servo wristServo;
Servo gripperServo;

void setup() {
  Serial.begin(9600);
  //Serial.setTimeout(200);
 
  // Initialize servo pin and stepper speeds and accelerations 
  elbowServo.attach(elbowServoPin);  // attaches the servo on pin 9 to the Servo object
  elbowServo.write(130);
  wristServo.attach(wristServoPin);
  gripperServo.attach(gripperServoPin);
  baseStepper.setMaxSpeed(300);  // Steps per second (max value set by arduino clock speed)
  baseStepper.setAcceleration(500.0);  // Steps per secon^2 (6000 default)
  shoulderStepper.setMaxSpeed(50000);  // Steps per second (max value set by arduino clock speed)
  shoulderStepper.setAcceleration(2000.0);  // Steps per secon^2 (6000 default)

  shoulderStepper.setCurrentPosition(0);
  baseStepper.setCurrentPosition(0);

  Serial.print("Motor angle = ");
  Serial.print(baseStepper.currentPosition() * 360/baseStepsPerRev);
  Serial.print(", ");
  Serial.print(shoulderStepper.currentPosition() * 360/shoulderStepsPerRev);

  // Run calibration before limit switch becomes an interupt
  pinMode(humerusLS_pin, INPUT_PULLUP);
  //calibrateShoulder();
  //angles[1] = 90;

  // Set the hummerous outside limit switch to an interupt pin so it will have imediate control
  delay(2000);
  attachInterrupt(digitalPinToInterrupt(humerusLS_pin), limit_ISR, LOW);

  Serial.println("Arduino ready");
}


void loop() {
  if (Serial.available() > 0) {
    // Read incoming data until newline
    int bytesRead = Serial.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
    buffer[bytesRead] = '\0';  // Null terminate
   
    // Parse the three comma-separated values
    char* token = strtok(buffer, ",");
    for (int i = 0; i < 5 && token != NULL; i++) {
      angles[i] = atof(token);
      token = strtok(NULL, ",");
    }
  
    // Set motors to the given angles
    setMotors(angles);
   
  }
}

void printMotorAngles(float angles[5]) {

  Serial.print("Motor angle = ");
  Serial.print(angles[0]);
  Serial.print(", ");
  Serial.print(angles[1]);
  Serial.print(", ");
  Serial.println(angles[2]);
  Serial.print(", ");
  Serial.println(angles[3]);
  Serial.print(", ");
  Serial.println(angles[4]);

}




void setMotors(float angles[3]) {

  printMotorAngles(angles);

  Serial.print("Writing to Elbow: ");
  Serial.println(angles[2]);
  pos = map(angles[2], 0, 166, 0, 180);   // Linearly maps 0-166 onto 0-180 for the servo
  pos = pos + 6;  // adds 6 for the constant offset
  elbowServo.write(180 - pos);

  Serial.print("Writing to wrist: ");
  Serial.println(angles[3]);
  wristServo.write(angles[3]);

  Serial.println("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk");
  Serial.print(baseStepper.currentPosition() * 360/baseStepsPerRev);
  Serial.print(", ");
  Serial.print(shoulderStepper.currentPosition() * 360/shoulderStepsPerRev);
  Serial.println("pppppppppppppppppppppppppppppppppppppppp");

  Serial.print("Writing to Base: ");
  Serial.println(angles[0]);
  baseStepper.moveTo(angles[0] * baseStepsPerRev/360);

  Serial.print("Writing to Shoulder: ");
  Serial.println(angles[1]);
  //pos = map(angles[1],0,180,-180,0);
  pos = angles[1];
  shoulderStepper.moveTo(pos * shoulderStepsPerRev/360);

  while (baseStepper.distanceToGo() || shoulderStepper.distanceToGo()){
    
    if(baseStepper.distanceToGo()) {
      baseStepper.run();  
    }
    if(shoulderStepper.distanceToGo()) {
     shoulderStepper.run(); 
    }
  }

  if(angles[4] == 1){
    openGripper();
  }
  if (angles[4] == 0){
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

//  Serial.println("Enter 0 to continue:");

//  while(1) {
//    if (Serial.available() > 0) {
//      char receivedChar = Serial.read();  // Read the incoming character
//      
//      // If the character received is '0', exit the loop and continue
//      if (receivedChar == '0') {
//        Serial.println("Received 0, continuing...");
//        break;  
//      }
//    }
//  }

  // rotate stepper motor backwards at half speed
  shoulderStepper.setSpeed(100);

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
      shoulderStepper.runToNewPosition(-90.0 * shoulderStepsPerRev/360);


      while(shoulderStepper.isRunning()){}

      Serial.println(shoulderStepper.currentPosition() * 360/shoulderStepsPerRev);

 
      Serial.println("Calibration complete");
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


void limit_ISR() {
  Serial.println("MADE IT TO ISR");
  shoulderStepper.stop();
  while(1){
    Serial.println("STOPPED");
  }
}
