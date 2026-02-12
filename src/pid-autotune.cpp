#include "pid-autotune.h"

#pragma region constructors
pid_tuner::pid_tuner(PID &pid) : _pid(pid)
{
    _state = IDLE;
}
#pragma endregion constructors

#pragma region getters
double *pid_tuner::getConstants()
{
    _constants[0] = _pid.getKp();
    _constants[1] = _pid.getKi();
    _constants[2] = _pid.getKd();
    return _constants;
}
#pragma endregion getters

void pid_tuner::start()
{
    _cycle_count = 0;
    _samples = 0;
    _kp_sum = _ki_sum = _kd_sum = 0;
    _peak_max = -1e12;
    _peak_min = 1e12;
    _state = TUNING;
    _output_state = true;
    _t_last_switch = millis();
}

double pid_tuner::update(double input)
{
    if (_state != TUNING)
        return 0;

    uint32_t now = millis();

    // Safety check: Abort if system does not oscillate within timeout
    if (now - _t_last_switch > _timeout_ms)
    {
        _state = TIMEOUT_ERROR;
        return 0;
    }

    // Track peaks for amplitude calculation
    if (input > _peak_max)
        _peak_max = input;
    if (input < _peak_min)
        _peak_min = input;

    // Relay Logic with Hysteresis (Non-blocking)
    if (_output_state && (input > (_target_input + _hysteresis)))
    {
        // Switch to LOW
        _output_state = false;
    }
    else if (!_output_state && (input < (_target_input - _hysteresis)))
    {
        // Switch to HIGH - One full cycle complete
        uint32_t elapsed = now - _t_last_switch;
        double tu = elapsed / 1000.0; // Ultimate Period (Tu) in seconds
        _t_last_switch = now;
        _output_state = true;

        // Calculate amplitudes
        double a = (_peak_max - _peak_min) / 2.0;      // Input Amplitude
        double d = (_output_high - _output_low) / 2.0; // Output Amplitude

        // Ultimate Gain (Ku)
        if (a > 0)
        {
            double ku = (4.0 * d) / (PI * a);

            // Tuning rules
            double kp_c = 0.6, ti_c = 0.5, td_c = 0.125;
            if (_mode == PESSEN_INTEGRAL_RULE)
            {
                kp_c = 0.7;
                ti_c = 0.4;
                td_c = 0.15;
            }
            else if (_mode == SOME_OVERSHOOT)
            {
                kp_c = 0.33;
                ti_c = 0.5;
                td_c = 0.33;
            }
            else if (_mode == NO_OVERSHOOT)
            {
                kp_c = 0.2;
                ti_c = 0.5;
                td_c = 0.33;
            }

            // Skip the first cycle as it is often erratic (transient)
            if (_cycle_count > 0)
            {
                double current_kp = kp_c * ku;
                _kp_sum += current_kp;
                _ki_sum += current_kp / (ti_c * tu);
                _kd_sum += current_kp * (td_c * tu);
                _samples++;
            }
        }

        // Reset peaks for next cycle
        _peak_max = -1e12;
        _peak_min = 1e12;
        _cycle_count++;

        // Check if finished
        if (_cycle_count >= _cycles_requested)
        {
            if (_samples > 0)
            {
                _pid.setConstants(_kp_sum / _samples, _ki_sum / _samples, _kd_sum / _samples);
            }
            _state = DONE;
        }
    }

    return _output_state ? _output_high : _output_low;
}