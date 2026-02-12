#include "pid.h"
#include "pid-autotune.h"

// Hardware Configuration
const int sensorPin = A0;
const int outputPin = 11;
const double testSetpoint = 100.0;

PID myPid;
pid_tuner tuner(myPid);

// Structure to store results for the final report
struct TuningResult {
  const char* modeName;
  double kp;
  double ki;
  double kd;
};

TuningResult results[4];
int currentModeIndex = 0;
bool allTestsDone = false;

// Array of modes to iterate through
pid_tuner::mode_t modes[] = {
  pid_tuner::CLASSIC_PID,
  pid_tuner::PESSEN_INTEGRAL_RULE,
  pid_tuner::SOME_OVERSHOOT,
  pid_tuner::NO_OVERSHOOT
};

const char* modeNames[] = {
  "Classic PID",
  "Pessen Integral",
  "Some Overshoot",
  "No Overshoot"
};

void setup() {
  Serial.begin(115200);
  myPid.setConstrains(0, 255);
  
  // Setup Tuner defaults
  tuner.setTargetValue(testSetpoint);
  tuner.setOutputRange(0, 255);
  tuner.setCycles(8); // Number of oscillations per test
  
  Serial.println("Starting in 3 seconds... Prepare your hardware.");
  delay(3000);
  
  startNextTest();
}

void startNextTest() {
  if (currentModeIndex < 4) {
    Serial.print("\n>>> Now Testing: ");
    Serial.println(modeNames[currentModeIndex]);
    
    tuner.setTuningMode(modes[currentModeIndex]);
    tuner.start(); // Reset internal sums and peak tracking
  } else {
    printFinalReport();
    allTestsDone = true;
  }
}

void loop() {
  if (allTestsDone) return;

  double input = analogRead(sensorPin);
  double output = tuner.update(input); // Core tuning logic
  analogWrite(outputPin, output);

  // Check if current tuning cycle is complete
  if (tuner.isDone()) {
    // Store constants using the library's getter
    results[currentModeIndex].modeName = modeNames[currentModeIndex];
    results[currentModeIndex].kp = myPid.getKp();
    results[currentModeIndex].ki = myPid.getKi();
    results[currentModeIndex].kd = myPid.getKd();
    
    Serial.println("Done.");
    currentModeIndex++;
    
    // Give the system time to settle between tests
    analogWrite(outputPin, 0);
    Serial.println("Settling system (5 seconds)...");
    delay(5000);
    
    startNextTest();
  }
}

// Helper: Prints a string right-aligned within a specific width
void printRightAligned(String val, int width) {
  int spaces = width - val.length();
  for (int i = 0; i < spaces; i++) Serial.print(" ");
  Serial.print(val);
}

void printFinalReport() {
  Serial.println("\n============================================================");
  Serial.println("                PID TUNING SUMMARY REPORT                   ");
  Serial.println("============================================================");
  
  // Define wider column widths to prevent overlapping
  const int colMode = 22; // Mode Name
  const int colVal  = 12; // Constants (Kp, Ki, Kd)
  
  // 1. Print Headers
  Serial.print("Mode");
  // Calculate padding for "Mode" header to reach start of next column
  for(int i = 4; i < colMode; i++) Serial.print(" ");
  
  printRightAligned("Kp", colVal);
  printRightAligned("Ki", colVal);
  printRightAligned("Kd", colVal);
  Serial.println();
  
  Serial.println("------------------------------------------------------------");
  
  // 2. Print Data Rows
  for (int i = 0; i < 4; i++) {
    // Print Mode Name (Left Aligned)
    Serial.print(results[i].modeName);
    for(int j = strlen(results[i].modeName); j < colMode; j++) Serial.print(" ");
    
    // Print Constants (Right Aligned with 2 decimal places)
    printRightAligned(String(results[i].kp, 2), colVal);
    printRightAligned(String(results[i].ki, 2), colVal);
    printRightAligned(String(results[i].kd, 2), colVal);
    Serial.println();
  }
  
  Serial.println("============================================================");
  Serial.println("Test Complete. System Halted.");
}
