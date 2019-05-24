#include "actuators/Normalized_PWM.h"
#include "CarActuators.h"

#include <ros/ros.h>
#include <memory>
#include <algorithm>
#include <geometry_msgs/Twist.h>

/*
 * The actuators node
*/
class Actuators {
public:
	/*
	 * Constructor. Sets up the node but does not run it.
	 * @param argc from main()
	 * @param argv from main()
	*/
	Actuators(int argc, char **argv) {
        // Initialize the node
		ros::init(argc, argv, "actuators");
		_car = std::make_unique<car::CarActuators>(0, 1);
		// Calibrate the throttle
		_car->throttle(car::THROTTLE_STOP);
		_throttle = car::THROTTLE_STOP;
		// Set the steering to the neutral position
		_car->steer(car::STEERING_CENTER);
		_steering = car::STEERING_CENTER;

		_nh = std::make_unique<ros::NodeHandle>();
		// Subscribe to Twist messages from teleop node(s) (training mode)
		_twist_sub = std::make_unique<ros::Subscriber>(_nh->subscribe("twist", 20, &Actuators::twist, this));
		// Subscriber to Normalized_PWM messages from the autopilot (auto mode)
		_norm_sub = std::make_unique<ros::Subscriber>(_nh->subscribe("norm_PWM", 20, &Actuators::norm_PWM, this));
		// Publish the normalized PWM values converted from the Twist messages for the datastore node (training mode)
		_norm_pub = std::make_unique<ros::Publisher>(_nh->advertise<actuators::Normalized_PWM>("normalized_PWM", 20));
		ROS_INFO("Car ready");
	}

    /*
     * Destructor. Stops the car and sets the steering to neutral
    */
    ~Actuators() {
        _car->throttle(car::THROTTLE_STOP);
        _car->steer(car::STEERING_CENTER);
    }

	/*
	 * Run the node until Ctrl-C is received, or ROS shuts down
	*/
	void run() {
		ros::spin();
	}

private:
    int _steering;
    int _throttle;
    std::unique_ptr<car::CarActuators> _car;
    std::unique_ptr<ros::NodeHandle> _nh;
    std::unique_ptr<ros::Subscriber> _twist_sub;
    std::unique_ptr<ros::Subscriber> _norm_sub;
    std::unique_ptr<ros::Publisher> _norm_pub;

	/*
	 * Truncate the servo values to be within limits
	*/
	void truncate_pwms() {
		_steering = std::max(std::min(_steering, car::STEERING_LEFTMOST), car::STEERING_RIGHTMOST);
		_throttle = std::max(std::min(_throttle, car::THROTTLE_MAX), car::THROTTLE_MIN);
	}

	/*
	 * Callback for receiving a Twist message
	 * @param The received message
	*/
    void twist(const geometry_msgs::Twist::ConstPtr& msg) {
        int throttle_scale{1};
        if (msg->linear.y != 0) {
			/*
			 * For the throttle servo, (318, 348) is a dead zone
			 * Use a scale of 15 when:
			 *     -Speeding up forwards or backwards from a stop
			 *     -Speeding up forwards at the lower end of the dead zone
			 *	   -Slowing down at the upper end of the dead zone
			*/
            if ((_throttle > 318 && _throttle < 348) ||
                (_throttle == 318 && msg->linear.y == 1) ||
                (_throttle == 348 && msg->linear.y == -1)) {
                throttle_scale = 15;
            }

            _throttle += msg->linear.y * throttle_scale;
        } else if (msg->angular.y == 0) {
			// Zero values for throttle and steering mean stop
            _throttle = 333;
        }

		// Use a scale of 15 for the steering
        _steering += msg->angular.y * 15;

		this->truncate_pwms();

		// Publish the normalized PWM values for the datastore node
        actuators::Normalized_PWM norm_msg;
        norm_msg.steering = _car->normalize_steering(_steering);
        norm_msg.throttle = _car->normalize_throttle(_throttle);
        _norm_pub->publish(norm_msg);

        _car->throttle(_throttle);
        _car->steer(_steering);
    }

	/*
	 * Callback for receiving a Normalized_PWM message
	 * @param The received message
	*/
    void norm_PWM(const actuators::Normalized_PWM::ConstPtr& msg) {
        _steering = _car->absolute_steering(msg->steering);
        _throttle = _car->absolute_throttle(msg->throttle);

		this->truncate_pwms();

        _car->steer(_steering);
        _car->throttle(_throttle);
    }
};

int main(int argc, char **argv) {
    Actuators car(argc, argv);
    car.run();

    return 0;
}
