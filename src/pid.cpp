#include "pid.h"

#pragma region constructors
// Default constructor
PID::PID()
{
    _acc = 0;
    _isEnabled = true; // By default, PID controller is enabled
}

// Constructor with PID constants
PID::PID(double kp, double ki, double kd)
{
    _acc = 0;
    _isEnabled = true; // By default, PID controller is enabled
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// Constructor with angular flag
PID::PID(bool isAngular)
{
    _acc = 0;
    _isEnabled = true; // By default, PID controller is enabled
    _isAngular = isAngular;
}

// Constructor with angular flag and PID constants
PID::PID(bool isAngular, double kp, double ki, double kd)
{
    _acc = 0;
    _isEnabled = true; // By default, PID controller is enabled
    _isAngular = isAngular;
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
#pragma endregion constructors

#pragma region setters
// Set PID constants
void PID::setConstants(double kp, double ki, double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// Set setpoint value
void PID::setSetPoint(double setpoint)
{
    _setpoint = setpoint;
}

// Set output constraints
void PID::setConstrains(double lower, double upper)
{
    _lower = lower;
    _upper = upper;
}

// Enable PID controller
void PID::enable()
{
    _isEnabled = true;
}

// Disable PID controller
void PID::disable()
{
    _isEnabled = false;
}

// Reset PID controller state
void PID::reset()
{
    _acc = 0;
    _prev_time = 0;
}
#pragma endregion setters

// Compute PID output
double PID::compute(double feedback)
{
    if (!_isEnabled)
        return 0;

    uint32_t now = micros();
    if (_prev_time == 0)
    {
        _prev_time = now;
        _prev_feedback = feedback;
        return 0;
    }

    double dt = (now - _prev_time) / 1000000.0;
    if (dt <= 0)
        return _last_output;

    double error = _setpoint - feedback;

    if (_isAngular)
    {
        while (error > 180)
            error -= 360;
        while (error < -180)
            error += 360;
    }

    // Integral with basic clamping anti-windup
    _acc += error * dt;

    // Calculate the max accumulator value allowed based on K_i
    double max_i = (_ki > 0) ? (_upper / _ki) : 1e12;
    double min_i = (_ki > 0) ? (_lower / _ki) : -1e12;

    // Clamp the accumulator immediately to prevent windup
    _acc = constrain(_acc, min_i, max_i);

    double i_term = _ki * _acc;
    i_term = constrain(i_term, _lower, _upper);

    // Derivative on Measurement (avoids derivative kick on setpoint change)
    double d_input = feedback - _prev_feedback;
    if (_isAngular)
    {
        while (d_input > 180)
            d_input -= 360;
        while (d_input < -180)
            d_input += 360;
    }
    double derivative = d_input / dt;

    _prev_time = now;
    _prev_feedback = feedback;

    _last_output = (_kp * error) + i_term - (_kd * derivative);
    return constrain(_last_output, _lower, _upper);
}
