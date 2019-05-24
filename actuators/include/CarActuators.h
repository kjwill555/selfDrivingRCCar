#pragma once

#include <memory>
#include "I2C.h"

namespace car {
	// Car servo PWM limits
	const int THROTTLE_MAX = 365;
	const int THROTTLE_STOP = 333;
	const int THROTTLE_MIN = 280;
	const int STEERING_LEFTMOST = 390;
	const int STEERING_CENTER = 350;
	const int STEERING_RIGHTMOST = 300;

	/*
	 * Class for controlling the car
	*/
	class CarActuators {
	public:
		/*
		 * Constructor.
		 * @param The channel for the steering servo
		 * @param The channel for the throttle servo
		*/
		CarActuators(int, int);
		/*
		 * Set the position of the steering servo
		 * @param The PWM to send to the servo
		*/
		void steer(int);
		/*
		 * Set the throttle level
		 * @param The PWM to send to the servo
		*/
		void throttle(int);
		/*
		 * Normalize a PWM in [0, 1] for the steering servo
		 * @param The PWM to normalize
		*/
		double normalize_steering(int);
		/*
		 * Normalize a PWM in [0, 1] for the throttle servo
		 * @param The PWM to normalize
		*/
		double normalize_throttle(int);
		/*
		 * Convert a normalized steering PWM back to the absolute value
		 * @param The normalized PWM
		*/
		int absolute_steering(double);
		/*
		 * Convert a normalized throttle PWM back to the absolute value
		 * @param The normalized PWM
		*/
		int absolute_throttle(double);

	private:
		std::unique_ptr<i2c::I2C> _i2c;
		int _steering_channel;
		int _throttle_channel;

		/*
		 * Normalize a PWM value
		 * @param The PWM value to normalize
		 * @param The minimum PWM value
		 * @param The maximum PWM value
		*/
		double normalize_pwm(int, int, int);
		/*
		 * Convert a PWM value to its absolute value
		 * @param The normalized PWM value
		 * @param The minimum PWM value
		 * @param The maximum PWM value
		*/
		int absolute_pwm(double, int, int);
	};
}
