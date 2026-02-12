#include "pid-autotune.h"

// Hardware Simulation Variables
double virtualProcessValue = 20.0; // Starting temperature/position
double outputPower = 0;

// PID Objects
PID pid = PID();
pid_tuner tuner = pid_tuner(pid);

// Setup the Virtual Plant (The "System" we are controlling)
void simulateSystem(double output, double dt) {
    // A simple model: Input adds "heat", environment "cools" it down
    double gain = 0.5;      // How much the output affects the system
    double timeConstant = 2.0; // How slow the system is
    
    double change = (output * gain - (virtualProcessValue - 20.0)) * (dt / timeConstant);
    virtualProcessValue += change;
}

void applyOutput(double x) {
    outputPower = x;
    analogWrite(11, x); // Visualize on Pin 11 in simulator
}

void setup() {
    Serial.begin(115200);
    
    // Configure Tuner
    tuner.setTargetValue(100);    // We want to reach 100
    tuner.setCycles(8);           // Number of oscillations to watch
    tuner.setOutputRange(0, 255);
    
    // Start tuning
    tuner.start();
    Serial.println("Starting Autotune Relay...");
}

void loop() {
    static uint32_t lastTime = millis();
    uint32_t now = millis();
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // 1. Update the physical simulation
    simulateSystem(outputPower, dt);
    double currentInput = virtualProcessValue;

    // 2. Run the State Machine
    if (!tuner.isDone()) {
        // --- TUNING PHASE ---
        double relayOutput = tuner.update(currentInput);
        applyOutput(relayOutput);
        
        // Print progress
        static uint32_t printTimer = 0;
        if (now - printTimer > 200) {
            Serial.print("Tuning_Input:"); Serial.print(currentInput);
            Serial.print(",Output:"); Serial.println(outputPower);
            printTimer = now;
        }
    } 
    else {
        // --- CONTROL PHASE ---
        static bool firstRun = true;
        if (firstRun) {
            double* constants = tuner.getConstants();
            Serial.println(">>> TUNING DONE <<<");
            Serial.print("Final Kp: "); Serial.println(constants[0]);
            pid.enable();
            pid.setSetPoint(120); // Move to a new setpoint to test
            firstRun = false;
        }

        double pidOutput = pid.compute(currentInput);
        applyOutput(pidOutput);

        // Telemetry for Serial Plotter
        Serial.print("Setpoint:120,Input:");
        Serial.print(currentInput);
        Serial.print(",Output:");
        Serial.println(pidOutput);
    }

    delay(20); // Simulation step
}