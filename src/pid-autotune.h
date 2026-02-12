#ifndef PID_AUTOTUNE_H
#define PID_AUTOTUNE_H

#include "pid.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/**
 * @brief Class for PID autotuning.
 *
 * This class provides functionality for tuning the PID constants automatically
 * based on the system response. It implements several methods for autotuning
 * PID controllers, including classic PID, Pessen integral rule, some overshoot,
 * and no overshoot methods.
 */

class pid_tuner
{
public:
  /**
   * @brief Enum for tuning rules.
   */
  typedef enum
  {
    CLASSIC_PID,
    PESSEN_INTEGRAL_RULE,
    SOME_OVERSHOOT,
    NO_OVERSHOOT
  } mode_t;

  /**
   * @brief Enum for tuner state.
   */
  typedef enum
  {
    IDLE,
    TUNING,
    DONE,
    TIMEOUT_ERROR
  } state_t;

private:
  PID &_pid;                  // Reference to the PID object to be tuned
  mode_t _mode = CLASSIC_PID; // Tuning rule to apply
  state_t _state = IDLE;      // Current state of the machine

  double _target_input = 0;  // Setpoint for tuning
  double _hysteresis = 0.5;  // Noise margin to prevent chatter
  double _output_low = 0;    // Low output value during relay
  double _output_high = 255; // High output value during relay

  uint32_t _cycles_requested = 10; // Number of oscillation cycles
  uint32_t _cycle_count = 0;       // Current cycle counter
  uint32_t _timeout_ms = 30000;    // Safety timeout per half-cycle

  bool _output_state = false;  // Current relay output state
  uint32_t _t_last_switch = 0; // Timestamp of the last relay flip
  double _peak_max = -1e12;    // Peak high during current cycle
  double _peak_min = 1e12;     // Peak low during current cycle

  double _kp_sum = 0;               // Accumulated Kp for averaging
  double _ki_sum = 0;               // Accumulated Ki for averaging
  double _kd_sum = 0;               // Accumulated Kd for averaging
  uint32_t _samples = 0;            // Count of valid tuning samples
  double _constants[3] = {0, 0, 0}; // Member array for safe memory access

public:
// Constructors
#pragma region constructors
  /**
   * @brief Constructor for pid_tuner.
   * @param pid Reference to the PID controller instance.
   */
  pid_tuner(PID &pid);
#pragma endregion constructors

// Setters
#pragma region setters
  void setTargetValue(double target) { _target_input = target; }
  void setHysteresis(double hysteresis) { _hysteresis = hysteresis; }
  void setOutputRange(double low, double high)
  {
    _output_low = low;
    _output_high = high;
  }
  void setTuningMode(mode_t mode) { _mode = mode; }
  void setCycles(uint32_t cycles) { _cycles_requested = cycles; }
  void setTimeout(uint32_t ms) { _timeout_ms = ms; }
#pragma endregion setters

// Getters
#pragma region getters
  /**
   * @brief Get the tuned constants.
   * @return Pointer to an array containing {Kp, Ki, Kd}.
   */
  double *getConstants();

  state_t getState() const { return _state; }
  bool isDone() const { return _state == DONE; }
  bool hasFailed() const { return _state == TIMEOUT_ERROR; }
#pragma endregion getters

  /**
   * @brief Resets and starts the tuning process.
   */
  void start();

  /**
   * @brief Updates the tuning state machine. Call this in the main loop.
   * @param input Current feedback value from the sensor.
   * @return The relay output value to be applied to the actuator.
   */
  double update(double input);
};

#endif /* PID_AUTOTUNE_H */
