#include "pid.h"
#include "pid-autotune.h"

PID myPid;
pid_tuner tuner(myPid);

const int sensorPin = A0;
const int outputPin = 11;
bool tuningMode = true;

double sp = 0;

void outputFunc(double x) {
  analogWrite(outputPin, x);
}

void setup() {
    Serial.begin(115200);
    myPid.setConstrains(0, 255);
    
    // Setup Tuner
    tuner.setTargetValue(120.0);
    tuner.setOutputRange(0, 255);
    tuner.setCycles(5);
    tuner.start();

    Serial.println("--- Starting Autotune ---");
    Serial.println("Wait for 'Tuning Done' before entering new setpoints.");
}

void loop() {
    double input = analogRead(sensorPin);

    if (tuningMode) {
        double output = tuner.update(input);
        outputFunc(output);
        
        if (tuner.isDone()) {
            tuningMode = false;
            myPid.enable();
            Serial.println("--- Tuning Done! ---");
            Serial.print("Final Constants -> Kp: "); Serial.print(myPid.getKp());
            Serial.print(" Ki: "); Serial.print(myPid.getKi());
            Serial.print(" Kd: "); Serial.println(myPid.getKd());
            Serial.println("Commands: 'D' for Disturbance, 'S[value]' for Setpoint (e.g. S150)");        }
    } else {
        // --- Normal PID Operation ---
        double output = myPid.compute(input);
        outputFunc(output);

        // --- Handle Serial Commands ---
        if (Serial.available() > 0) {
            char cmd = Serial.read(); // Read the first character
            
            if (cmd == 'D' || cmd == 'd') {
                // DISTURBANCE: Cut power for 2 seconds
                Serial.println("Disturbance: Cutting Power!");
                outputFunc(0);
                delay(2000); 
            } 
            else if (cmd == 'S' || cmd == 's') {
                // SETPOINT: Read the number following 'S'
                sp = (double)Serial.parseFloat(); 
                if (sp > 0) {
                    myPid.setSetPoint(sp);
                    Serial.print(">>> Setpoint changed to: ");
                    Serial.println(sp);
                }
            }
        }

        // --- Monitoring Output ---
        Serial.print("Setpoint:"); Serial.print(sp);
        Serial.print(",Input:"); Serial.print(input);
        Serial.print(",Output:"); Serial.println(output);
        delay(100); // Small delay to make the Serial Monitor readable
    }

    delay(20);
}
