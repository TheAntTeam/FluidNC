// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    This is a full featured TTL PWM spindle This does not include speed/power
    compensation. Use the Laser class for that.
*/
#include "PWMSpindle.h"

#include "../System.h"  // sys
#include "../GCode.h"   // gc_state.modal

// ======================= PWM ==============================
/*
    This gets called at startup or whenever a spindle setting changes
    If the spindle is running it will stop and need to be restarted with M3Snnnn
*/

namespace Spindles {
    void PWM::init() {
        is_reversable = _direction_pin.defined();

        if (_output_pin.defined()) {
            if (_output_pin.capabilities().has(Pin::Capabilities::PWM)) {
                auto outputNative = _output_pin.getNative(Pin::Capabilities::PWM);
                _output_pin.setAttr(Pin::Attr::PWM, _pwm_freq);
            } else {
                log_error(name() << " output pin " << _output_pin.name().c_str() << " cannot do PWM");
            }
        } else {
            log_error(name() << " output pin not defined");
        }

        _current_state    = SpindleState::Disable;
        _current_pwm_duty = 0;

        _enable_pin.setAttr(Pin::Attr::Output);
        _direction_pin.setAttr(Pin::Attr::Output);

        if (_speeds.size() == 0) {
            // The default speed map for a PWM spindle is linear from 0=0% to 10000=100%
            linearSpeeds(10000, 100.0f);
        }
        setupSpeeds(_output_pin.maxDuty());
        init_atc();

		if (_use_pwm_ramping) {
            if (maxSpeed() < 300 || _ramp_up_delay_ms < 500 || _ramp_down_delay_ms < 500) {  // Some reasonable values for ramping
                log_warn("PWM Ramping max speed < 300 or spinup_ms/spindown_ms < 500...disabling");
				_use_pwm_ramping = false;
            } else {
                log_warn("PWM Ramping init else");
                // TODO: Do we want to deal with a min speed?
                _ramp_up_dev_increment   = mapSpeed(SpindleState::Cw, maxSpeed()) / (_ramp_up_delay_ms / _ramp_interval);
                _ramp_down_dev_increment = mapSpeed(SpindleState::Cw, maxSpeed()) / (_ramp_down_delay_ms / _ramp_interval);
                log_info("PWM Ramping Maxspeed:" << maxSpeed() << " spinup incr:" << _ramp_up_dev_increment
                                                 << " spindown incr:" << _ramp_down_dev_increment);
            }
        }

        //log_info("Maxspeed:" << maxSpeed() << " mapped max:" << mapSpeed(maxSpeed()) << " ovr:" << sys.spindle_speed_ovr);
        config_message();
    }

    void IRAM_ATTR PWM::setSpeedfromISR(uint32_t dev_speed) {
        set_enable(gc_state.modal.spindle != SpindleState::Disable);
        set_output(dev_speed);
    }

    // XXX this is the same as OnOff::setState so it might be possible to combine them
    void PWM::setState(SpindleState state, SpindleSpeed speed) {
        if (sys.abort) {
            return;  // Block during abort.
        }

        if (!_output_pin.defined()) {
            log_config_error(name() << " spindle output_pin not defined");
        }

        uint32_t dev_speed = mapSpeed(state, speed);
        uint32_t dev_speed_0 = mapSpeed(state, 0);
        if (_use_pwm_ramping) {
            log_warn("PWM Ramping intercepted");
            set_enable(state != SpindleState::Disable);
            if (state != SpindleState::Disable) {
                if (_direction_pin.defined() && (_direction_pin.read() != (state == SpindleState::Cw))) {
                    ramp_speed(dev_speed_0);
                }
                set_direction(state == SpindleState::Cw);
                ramp_speed(dev_speed);
            } else {
                ramp_speed(dev_speed_0);  // Always want to ramp down on diable
            }
        } else {
            if (state != SpindleState::Disable) {  // Halt or set spindle direction and speed.
                // XXX this could wreak havoc if the direction is changed without first
                // spinning down.
                set_direction(state == SpindleState::Cw);
            }
        }

        // rate adjusted spindles (laser) in M4 set power via the stepper engine, not here

        // set_output must go first because of the way enable is used for level
        // converters on some boards.

        if (isRateAdjusted() && (state == SpindleState::Ccw)) {
            dev_speed = offSpeed();
            set_output(dev_speed);
        } else {
            set_output(dev_speed);
        }

        set_enable(state != SpindleState::Disable);
        spindleDelay(state, speed);
    }

    // prints the startup message of the spindle config
    void PWM::config_message() {
        log_info(name() << " Spindle Ena:" << _enable_pin.name() << " Out:" << _output_pin.name() << " Dir:" << _direction_pin.name()
                        << " Freq:" << _pwm_freq << "Hz Period:" << _output_pin.maxDuty() << atc_info()

        );
    }

    void IRAM_ATTR PWM::set_output(uint32_t duty) {
        // to prevent excessive calls to pwmSetDuty, make sure duty has changed
        if (duty == _current_pwm_duty) {
            return;
        }

        _current_pwm_duty = duty;
        _output_pin.setDuty(duty);
    }

    void PWM::deinit() {
        stop();
        _output_pin.setAttr(Pin::Attr::Input);
        _enable_pin.setAttr(Pin::Attr::Input);
        _direction_pin.setAttr(Pin::Attr::Input);
    }

    void PWM::ramp_speed(uint32_t target_duty) {
        // speed is given, but we need to work in dev_speed
        uint32_t next_duty   = _current_duty;  // this is the value that increments in this function
        bool     spinup      = (target_duty > _current_duty);

        log_warn("Ramp duty from:" << _current_duty << " to:" << target_duty);

        while ((spinup && next_duty < target_duty) || (!spinup && (next_duty > target_duty))) {
            if (spinup) {
                if (next_duty + _ramp_up_dev_increment < target_duty) {
                    next_duty += _ramp_up_dev_increment;
                } else {
                    next_duty = target_duty;
                }
            } else {
                if ((next_duty > _ramp_down_dev_increment) && (next_duty - _ramp_down_dev_increment > target_duty)) {  // is is safe to subtract?
                    next_duty -= _ramp_down_dev_increment;
                } else {
                    next_duty = target_duty;
                }
            }

            //log_warn("  next duty:" << next_duty);

            set_output(next_duty);
            _current_duty = next_duty;
            if (next_duty == target_duty) {
                return;
            }
            delay_ms(_ramp_interval);
        }
    }

    // Configuration registration
    namespace {
        SpindleFactory::InstanceBuilder<PWM> registration("PWM");
    }
}
