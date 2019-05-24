#include "CarActuators.h"

car::CarActuators::CarActuators(int steering_channel, int throttle_channel) {
    _steering_channel = steering_channel;
    _throttle_channel = throttle_channel;
    _i2c = std::make_unique<i2c::I2C>(1, 0x40, 50);
}

void car::CarActuators::steer(int pwm_value) {
    _i2c->set_PWM(_steering_channel, pwm_value);
}

void car::CarActuators::throttle(int pwm_value) {
    _i2c->set_PWM(_throttle_channel, pwm_value);
}

double car::CarActuators::normalize_pwm(int absolute_pwm, int min_pwm, int max_pwm) {
	return (absolute_pwm - min_pwm) / (double)(max_pwm - min_pwm);
}

double car::CarActuators::normalize_throttle(int absolute_pwm) {
    return this->normalize_pwm(absolute_pwm, car::THROTTLE_MIN, car::THROTTLE_MAX);
}

double car::CarActuators::normalize_steering(int absolute_pwm) {
    return this->normalize_pwm(absolute_pwm, car::STEERING_RIGHTMOST, car::STEERING_LEFTMOST);
}

int car::CarActuators::absolute_pwm(double relative_pwm, int min_pwm, int max_pwm) {
	return (int)(min_pwm + relative_pwm * (max_pwm - min_pwm));
}

int car::CarActuators::absolute_steering(double relative_pwm) {
    return this->absolute_pwm(relative_pwm, car::STEERING_RIGHTMOST, car::STEERING_LEFTMOST);
}

int car::CarActuators::absolute_throttle(double relative_pwm) {
    return this->absolute_pwm(relative_pwm, car::THROTTLE_MIN, car::THROTTLE_MAX);
}
