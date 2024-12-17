#include <Wire.h>
#include <LiquidCrystal.h>
#include <MPU6050.h> // Include the MPU6050 library
#include <math.h>    // Include math library for angle calculation

// LCD pin definitions
const int rs = 12;   // Register select pin
const int en = 11;   // Enable pin
const int d4 = 6;    // Data pin 4
const int d5 = 4;    // Data pin 5
const int d6 = 8;    // Data pin 6
const int d7 = 7;    // Data pin 7

// Motor pin definitions
const int motorIn1 = 10;    // Motor input 1
const int motorIn2 = 13;    // Motor input 2
const int motorEnable = 9;  // Motor enable pin (PWM)

// Encoder pin definitions
const int encoderC1 = 2;    // Encoder Channel 1
const int encoderC2 = 3;    // Encoder Channel 2

// Variables for encoder tracking
volatile int encoderTicks = 0; // Count of encoder ticks
int lastEncoderStateC1 = LOW;  // Previous state of C1
unsigned long lastPrintTime = 0; // Last time encoder speed was printed

// Initialize the LCD object
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Initialize MPU6050 object
MPU6050 mpu;

float previousAngle = 0; // Track the previous angle for relative movement

void setup() {
  // Set motor pins as outputs
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorEnable, OUTPUT);

  // Set encoder pins as inputs
  pinMode(encoderC1, INPUT);
  pinMode(encoderC2, INPUT);

  // Initialize Serial communication
  Serial.begin(9600);

  // Initialize the LCD
  lcd.begin(16, 2);  // Initialize the LCD with 16 columns and 2 rows
  lcd.setCursor(0, 0);
  lcd.print("Motor Controller");
  delay(2000); // Show intro message for 2 seconds
  lcd.clear();

  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(encoderC1), encoderISR, CHANGE);

  // Start with the motor off
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
  analogWrite(motorEnable, 0); // Set PWM duty cycle to 0 (motor off)
  Serial.println("Motor is stopped.");

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  // Check MPU6050 connection
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 initialized successfully.");
}

void loop() {
  // Variables for accelerometer readings
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az); // Get accelerometer data

  // Convert raw accelerometer data to g's
  float x = ax / 16384.0;
  float y = ay / 16384.0;
  float z = az / 16384.0;

  // Calculate tilt angle based on accelerometer data
  float currentAngle = atan2(x, sqrt(y * y + z * z)) * 180 / M_PI;
  float angleDifference = currentAngle - previousAngle; // Calculate the relative angle movement
  previousAngle = currentAngle; // Update the previous angle

  Serial.print("Current angle: ");
  Serial.println(currentAngle);
  Serial.print("Angle difference: ");
  Serial.println(angleDifference);
  Serial.print("Encoder ticks: ");
  Serial.println(encoderTicks);

  // Display MPU value, encoder ticks, and angle difference on LCD
  lcd.setCursor(0, 0);
  lcd.print("MPU:");
  lcd.print(currentAngle, 1);
  lcd.print(" Enc:");
  lcd.print(encoderTicks);

  lcd.setCursor(0, 1);
  lcd.print("Diff:");
  lcd.print(angleDifference, 1);
  lcd.print("      "); // Clear extra characters

  // Move the motor based on the angle difference
  if (angleDifference > 0) {
    moveMotor(angleDifference, true); // Move forward for positive angle difference
  } else if (angleDifference < 0) {
    moveMotor(abs(angleDifference), false); // Move backward for negative angle difference
  }

  delay(30); // Short delay for stability
}

void moveMotor(float angle, bool forward) {
  int moveDuration = map(angle, 0, 360, 0, 2000); // Map angle to movement duration in milliseconds (adjust values as needed)

  if (forward) {
    digitalWrite(motorIn1, HIGH);
    digitalWrite(motorIn2, LOW);
    Serial.println("Motor spinning forward.");
  } else {
    digitalWrite(motorIn1, LOW);
    digitalWrite(motorIn2, HIGH);
    Serial.println("Motor spinning backward.");
  }

  analogWrite(motorEnable, 150); // Set motor speed (adjust value as needed)
  delay(moveDuration); // Move motor for the calculated duration based on the angle

  stopMotor(); // Stop the motor after movement
}

void stopMotor() {
  digitalWrite(motorIn1, LOW);
  digitalWrite(motorIn2, LOW);
  analogWrite(motorEnable, 0); // Disable motor
  Serial.println("Motor is stopped.");
}

// Interrupt Service Routine (ISR) for encoder
void encoderISR() {
  int c1State = digitalRead(encoderC1);
  int c2State = digitalRead(encoderC2);

  // Determine direction based on C1 and C2 states
  if (c1State != lastEncoderStateC1) {
    if (c2State != c1State) {
      encoderTicks++; // Forward direction
    } else {
      encoderTicks--; // Backward direction
    }
    lastEncoderStateC1 = c1State;
  }
}
