#include "pid-autotune.h"

// Define Hardware Pins
const int sensorPin = A0;
const int outputPin = 11;

// Global Objects
PID pid = PID();
pid_tuner tuner = pid_tuner(pid);

// Test variables
double currentSetpoint = 0;
unsigned long testStartTime = 0;
bool tuningFinished = false;

// Function to apply output to hardware
void outputFunc(double x) {
  analogWrite(outputPin, x);
}

void setup() {
  Serial.begin(115200);
  
  // 1. Initial Setup for Tuner
  tuner.setTargetValue(100);       // Target for tuning cycle
  tuner.setOutputRange(0, 255);    // Output constraints
  tuner.setCycles(10);             // 10 cycles for averaging
  
  // 2. Initialize PID constraints
  pid.setConstrains(0, 255);       // Prevent integral windup
  
  Serial.println(">>> PHASE 1: STARTING AUTOTUNE <<<");
  tuner.start();                   // Initialize tuning variables
}

void loop() {
  double input = analogRead(sensorPin);

  if (!tuningFinished) {
    // --- TUNING PHASE ---
    double output = tuner.update(input); // Run relay logic
    outputFunc(output);

    if (tuner.isDone()) { // Check state machine status
      tuningFinished = true;
      
      // Get results
      double* constants = tuner.getConstants(); // [Kp, Ki, Kd]
      Serial.println(">>> TUNING COMPLETE <<<");
      Serial.print("Kp: "); Serial.println(constants[0]);
      Serial.print("Ki: "); Serial.println(constants[1]);
      Serial.print("Kd: "); Serial.println(constants[2]);

      // Transition to PID Control
      pid.enable();               //
      currentSetpoint = 150;      // Set first test setpoint
      pid.setSetPoint(currentSetpoint); //
      testStartTime = millis();
      Serial.println(">>> PHASE 2: STEP RESPONSE TEST (Setpoint 150) <<<");
    }
  } else {
    // --- CONTROL PHASE ---
    double output = pid.compute(input); // Calculate PID response
    outputFunc(output);

    // Periodically change setpoints to observe behavior
    unsigned long elapsed = millis() - testStartTime;
    if (elapsed > 10000 && elapsed < 10100) {
      currentSetpoint = 200;
      pid.setSetPoint(currentSetpoint);
      Serial.println(">>> SETPOINT CHANGE: 200 <<<");
    }

    // Telemetry for Serial Plotter
    Serial.print("Setpoint:"); Serial.print(currentSetpoint);
    Serial.print(",");
    Serial.print("Input:"); Serial.print(input);
    Serial.print(",");
    Serial.print("Output:"); Serial.println(output);
  }
  
  delay(10); // Stability delay
}