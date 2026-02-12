#include "pid-autotune.h"

PID pid = PID();
pid_tuner tuner = pid_tuner(pid);

void outputFunc(double x) {
  analogWrite(11, x);
}

void setup() {
    Serial.begin(115200);

    tuner.setConstrains(0, 255);
    tuner.setTargetValue(100);
    tuner.setCycles(10);
    tuner.setTimeout(30000); // 30 seconds
    tuner.setTuningMode(pid_tuner::CLASSIC_PID);

    tuner.start();
}

void loop() {
    // Read sensor
    double input = analogRead(A0);
    double output = 0;

    if (!tuner.isDone()) {
        output = tuner.update(input);
    } else {
        output = pid.compute(input);
    }
    
    outputFunc(output);
}
