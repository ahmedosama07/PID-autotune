# PID-autotune [![arduino-library-badge](https://www.ardu-badge.com/badge/pid-autotune.svg?)](https://www.ardu-badge.com/pid-autotune)
A robust Arduino library for automatic PID tuning using the **Relay Method (Ziegler-Nichols)**.

## Features
- **4 Tuning Modes:** Optimize for speed, stability, or disturbance rejection.
- **Angular Logic:** Native support for 360-degree systems like gimbals or servos.
- **High Resolution:** Uses microsecond timing for high-speed control loops.
- **Failsafe Tuning:** Includes safety timeouts to prevent hardware damage during failed tuning cycles.

## Tuning Modes
The library supports several classic tuning rules via `pid_tuner::mode_t`:
| Mode | Strategy | Best For |
| :--- | :--- | :--- |
| `CLASSIC_PID` | Standard ZN tuning. Aggressive response. | General robotics |
| `PESSEN_INTEGRAL` | Emphasizes disturbance rejection. | Industrial processes |
| `SOME_OVERSHOOT` | Balanced response with moderate damping. | Motion control |
| `NO_OVERSHOOT` | Most conservative. Eliminates peaks. | Temperature control |

## Quick Start
```cpp
#include <pid-autotune.h>

PID myPid;
pid_tuner tuner(myPid);

void setup() {
  tuner.setTargetValue(100.0);
  tuner.setOutputRange(0, 255);
  tuner.start(); // Begins oscillation cycles
}

void loop() {
  double input = analogRead(A0);
  
  if (!tuner.isDone()) {
    analogWrite(11, tuner.update(input)); // Runs relay logic
  } else {
    analogWrite(11, myPid.compute(input)); // Runs tuned PID
  }
}
```
