#include <sys/ioctl.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <ros/ros.h>

#include "I2C.h"


i2c::I2C::I2C(int bus_num, uint8_t device_address, int frequency) {
	_bus_num = bus_num;
	_device_address = device_address;
	snprintf(_bus_path, sizeof(_bus_path), "/dev/i2c-%d", bus_num);
	if (!this->open_fd()) {
		ROS_ERROR("Could not open file descriptor '%s'", _bus_path);
		return;
	}

	this->set_all_PWM(0, 0);
	this->reset();
	usleep(5000); // Wait for oscillator
	uint8_t mode1;
	this->read_byte(i2c::MODE1, mode1);
	mode1 &= ~i2c::SLEEP; // Wake up (reset sleep)
	this->write_byte(i2c::MODE1, mode1);
	usleep(5000); // Wait for oscillator
	this->set_PWM_freq(frequency);
}

i2c::I2C::~I2C() {
	close(_fd);
}

bool i2c::I2C::open_fd() {
	if ((_fd = open(_bus_path, O_RDWR)) < 0) {
		ROS_ERROR("Couldn't open device at '%s'", _bus_path);
		return false;
	}

	// Treat the file descriptor as an I2C slave
	if (ioctl(_fd, I2C_SLAVE, _device_address) < 0) {
		ROS_ERROR("Failed to set device address of '%s' to 0x%x",
			   _bus_path, _device_address);
		close(_fd);
		_fd = -1;
		return false;
	}

	return true;
}

bool i2c::I2C::read_byte(uint8_t register_address, uint8_t& byte_out) {
	if (_fd != -1) {
		uint8_t buffer[i2c::BUFFER_SIZE];
		buffer[0] = register_address;
		// Tell the device which register we want to read from
		if (write(_fd, buffer, i2c::BUFFER_SIZE) != i2c::BUFFER_SIZE) {
			ROS_ERROR("I2C slave at address 0x%x failed to move to register 0x%x",
				_device_address, register_address);
			return false;
		}

		// Read from the specified register
		if (read(_fd, _data_buffer, i2c::BUFFER_SIZE) != i2c::BUFFER_SIZE) {
			ROS_ERROR("Could not read from I2C slave at address 0x%x, register 0x%x",
				_device_address, register_address);
			return false;
		}

		byte_out = _data_buffer[0];
		return true;
	}

	ROS_ERROR("I2C Read failed: device not available");
	return false;
}

bool i2c::I2C::write_byte(uint8_t register_address, uint8_t data) {
	if (_fd != -1) {
		uint8_t buffer[2];
		buffer[0] = register_address;
		buffer[1] = data;
		if (write(_fd, buffer, sizeof(buffer)) != sizeof(buffer)) {
			ROS_ERROR("Failed to write to I2C slave at address 0x%x, register 0x%x",
				_device_address, register_address);
			return false;
		}

		return true;
	}

	ROS_ERROR("I2C Write failed: device not available");
	return false;
}

void i2c::I2C::set_all_PWM(int on, int off) {
	this->write_byte(i2c::ALLLED_ON_L, on & 0xFF);
	this->write_byte(i2c::ALLLED_ON_H, on >> 8);
	this->write_byte(i2c::ALLLED_OFF_L, off & 0xFF);
	this->write_byte(i2c::ALLLED_OFF_H, off >> 8);
}

void i2c::I2C::set_PWM_freq(int freq) {
	double prescale_est = i2c::CLOCK_FREQ;
	prescale_est /= 4096.0; // 12-bit
	prescale_est /= (double)freq;
	prescale_est -= 1.0;
	int prescale_final = (int)floor(prescale_est + 0.5);

	uint8_t old_mode;
	this->read_byte(i2c::MODE1, old_mode);
	uint8_t new_mode = (old_mode & 0x7F) | 0x10; // Sleep
	this->write_byte(i2c::MODE1, new_mode); // Go to sleep
	this->write_byte(i2c::PRE_SCALE, prescale_final);
	this->write_byte(i2c::MODE1, old_mode);
	usleep(5000); // Wait for oscillator
	this->write_byte(i2c::MODE1, old_mode | 0x80);
}

void i2c::I2C::set_PWM(uint8_t led_channel, int value) {
	this->set_PWM(led_channel, 0, value);
}

void i2c::I2C::set_PWM(uint8_t led_channel, int on_value, int off_value) {
	this->write_byte(i2c::LED0_ON_L + i2c::LED_MULTIPLYER * led_channel, on_value & 0xFF);
	this->write_byte(i2c::LED0_ON_H + i2c::LED_MULTIPLYER * led_channel, on_value >> 8);
	this->write_byte(i2c::LED0_OFF_L + i2c::LED_MULTIPLYER * led_channel, off_value & 0xFF);
	this->write_byte(i2c::LED0_OFF_H + i2c::LED_MULTIPLYER * led_channel, off_value >> 8);
}

void i2c::I2C::reset() {
	this->write_byte(i2c::MODE2, i2c::OUTDRV);
	this->write_byte(i2c::MODE1, i2c::ALLCALL);
}
