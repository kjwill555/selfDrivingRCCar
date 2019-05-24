#pragma once

#include <inttypes.h>

namespace i2c {
	// Register Definitions
	const uint8_t MODE1 = 0x00; //Mode register  1
	const uint8_t MODE2 = 0x01;	//Mode register  2
	const uint8_t SUBADR1 = 0x02; //I2C-bus subaddress 1
	const uint8_t SUBADR2 = 0x03; //I2C-bus subaddress 2
	const uint8_t SUBADR3 = 0x04; //I2C-bus subaddress 3
	const uint8_t ALLCALLADR = 0x05; //LED All Call I2C-bus address
	const uint8_t LED0 = 0x6; //LED0 start register
	const uint8_t LED0_ON_L = 0x6; //LED0 output and brightness control byte 0
	const uint8_t LED0_ON_H = 0x7; //LED0 output and brightness control byte 1
	const uint8_t LED0_OFF_L = 0x8; //LED0 output and brightness control byte 2
	const uint8_t LED0_OFF_H = 0x9; //LED0 output and brightness control byte 3
	const int LED_MULTIPLYER = 4; // For the other 15 channels
	const uint8_t ALLLED_ON_L = 0xFA; // Load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
	const uint8_t ALLLED_ON_H = 0xFB; // Load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
	const uint8_t ALLLED_OFF_L = 0xFC; // Load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
	const uint8_t ALLLED_OFF_H = 0xFD; // Load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)

   // Commands
	const uint8_t RESTART = 0x80;
	const uint8_t SLEEP = 0x10;
	const uint8_t ALLCALL = 0x01;
	const uint8_t INVRT = 0x10;
	const uint8_t OUTDRV = 0x04;

	// Clock
	const uint8_t PRE_SCALE = 0xFE;		// Prescaler for output frequency
	const double CLOCK_FREQ = 25000000.0; // 25MHz default oscillator clock

	// Buffer size
	const int BUFFER_SIZE = 1;

	/*
	 * Class for commanding an I2C slave
	*/
	class I2C {
	public:
		/*
		 * Constructor
		 *
		 * @param The bus number of the device (/dev/i2c-X)
		 * @param The manufacturer address of the device
		 * @param The frequency to set on the device
		*/
		I2C(int, uint8_t, int);
		/*
		 * Destructor. Closes the underlying file descriptor
		*/
		~I2C();
		/*
		* Set the PWM of one register
		*
		* @param The register to set
		* @param The desired PWM
		*/
		void set_PWM(uint8_t, int);

	private:
		int _device_address;
		int _bus_num;
		char _bus_path[64];
		int _fd;
		uint8_t _data_buffer[BUFFER_SIZE];

		/*
		 * Open the file descriptor for the I2C device
		 *
		 * @returns Whether the file was successfully opened 
		*/
		bool open_fd();
		/*
		 * Read one byte from a specified register
		 *
		 * @param The register address
		 * @param Byte reference to store the read byte in
		*/
		bool read_byte(uint8_t, uint8_t&);
		/**
		* Writes a byte to the specified register
		*
		* @param The address of the register to write to
		* @param The byte to write
		*
		* @return Whether the write succeeded
		*/
		bool write_byte(uint8_t, uint8_t);
		/*
		 * Set the PWM for all of the registers
		 *
		 * @param ON value
		 * @param OFF value
		*/
		void set_all_PWM(int, int);
		/*
		 * Set the driver frequency
		 *
		 * @param The desired frequency
		*/
		void set_PWM_freq(int);
		/*
		 * Set the PWM of one register
		 *
		 * @param The register to set
		 * @param ON value
		 * @param OFF value
		*/
		void set_PWM(uint8_t, int, int);
		/*
		 * Reset the device
		*/
		void reset();
	};

}