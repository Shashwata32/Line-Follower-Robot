#define MOTOR_SPEED 300   // PWM speed for the motors

// Right motor
int enableRightMotor = 22;   // GPIO pin for right motor enable
int rightMotorPin1 = 16;     // GPIO pin for right motor direction 1
int rightMotorPin2 = 17;     // GPIO pin for right motor direction 2

// Left motor
int enableLeftMotor = 23;    // GPIO pin for left motor enable
int leftMotorPin1 = 18;      // GPIO pin for left motor direction 1
int leftMotorPin2 = 19;      // GPIO pin for left motor direction 2
int leftmotorspeed;
int rightmotorspeed;

// IR sensor array pins (8 sensors)
int irSensors[8] = {14, 25, 39, 36, 35, 34, 27, 32};  // GPIO pins for 8 IR sensors

void setup() {
  // Set PWM frequency for motor control pins
  ledcSetup(0, 5000, 8); // channel 0, 5 kHz frequency, 8-bit resolution
  ledcSetup(1, 5000, 8); // channel 1, 5 kHz frequency, 8-bit resolution

  ledcAttachPin(enableRightMotor, 0); // Attach right motor enable pin to channel 0
  ledcAttachPin(enableLeftMotor, 1);  // Attach left motor enable pin to channel 1

  // Set motor control pins as outputs
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Set IR sensor pins as inputs
  for (int i = 0; i < 8; i++) {
    pinMode(irSensors[i], INPUT);
  }

  rotateMotor(0, 0);  // Initial stop (both motors off)
}

void loop() {
  int sensorValues[8];
  
  // Read all IR sensors
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = digitalRead(irSensors[i]);
  }

  // Count how many sensors on the left are HIGH
  int leftHighCount = 0;
  for (int i = 0; i < 4; i++) {
    if (sensorValues[i] == HIGH) {
      leftHighCount++;
    }
  }

  // Count how many sensors on the right are HIGH
  int rightHighCount = 0;
  for (int i = 4; i < 8; i++) {
    if (sensorValues[i] == HIGH && i!=6) {
      rightHighCount++;
    }
  }

  // Logic for line following based on sensor array
  
   if (leftHighCount > rightHighCount) {
    rotateMotor(MOTOR_SPEED / 0.8, -MOTOR_SPEED / 2.5);  // Turn left
    leftmotorspeed=-MOTOR_SPEED / 2.5;
    rightmotorspeed=MOTOR_SPEED / 0.8;
  }
  else if (rightHighCount > leftHighCount) {
    rotateMotor(-MOTOR_SPEED / 2.5, MOTOR_SPEED / 0.8);  // Turn right
    leftmotorspeed=MOTOR_SPEED / 0.8;
    rightmotorspeed=-MOTOR_SPEED / 2.5;
  }
  else if(rightHighCount == leftHighCount && rightHighCount == 0)
  {
        rotateMotor(rightmotorspeed, leftmotorspeed);  
  }
  
  else {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);  // Move straight if both sides have equal high sensors
  }
  
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Right motor direction control
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW); // Motor goes backwards
  }
  else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);  // Motor goes forwards
  }
  else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);  // Motor stops
  }

  // Left motor direction control
  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);  // Motor goes backwards
  }
  else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);   // Motor goes forwards
  }
  else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);   // Motor stops
  }

  // Set motor speed using PWM signals (0-255)
  ledcWrite(0, 200); // Write PWM signal to right motor
  ledcWrite(1, 200);  // Write PWM signal to left motor
}