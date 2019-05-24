#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <actuators/Normalized_PWM.h>
#include <fstream>
#include <sstream>

/*
 * Node for collecting training data in bucket format
*/
class DataStore {
public:
    /*
     * Constructor. Initializes the node but doesn't start it
     * @param argc from main()
     * @param argv from main()
    */
    DataStore(int argc, char **argv) {
        // Initialize the node
        ros::init(argc, argv, "datastore");
        _nh = std::make_unique<ros::NodeHandle>();
        // Subcribe to the normalized PWMs output by the actuators node
        _actuators_sub = std::make_unique<ros::Subscriber>(_nh->subscribe("normalized_PWM", 20, &DataStore::normalized_pwm, this));
        // Create a stream to write the record files
        _file_writer = std::make_unique<std::ofstream>();
        _record_number = 0;
        _steering_pwm = 0.0;
        _throttle_pwm = 0.0;
        _dir = argv[1];
        // Subscribe to the images topic
        _image_node = std::make_unique<image_transport::ImageTransport>(*_nh);
        _image_sub = std::make_unique<image_transport::Subscriber>(_image_node->subscribe("camera/image", 20, &DataStore::image_recvd, this));
    }

    /*
     * Run the node
    */
    void run() {
        ros::spin();
    }

private:
    // ROS subscriber callbacks are guaranteed to be thread-safe
    // Thus, mutexes for these variables are not needed
    double _steering_pwm;
    double _throttle_pwm;
    int _record_number;
    char *_dir;
    std::unique_ptr<ros::NodeHandle> _nh;
    std::unique_ptr<ros::Subscriber> _actuators_sub;
    std::unique_ptr<image_transport::ImageTransport> _image_node;
    std::unique_ptr<image_transport::Subscriber> _image_sub;
    std::unique_ptr<std::ofstream> _file_writer;

    /*
     * Callback for receiving new servo values from the actuators node
     * @param The received message
    */
    void normalized_pwm(const actuators::Normalized_PWM::ConstPtr& msg) {
        _steering_pwm = msg->steering;
        _throttle_pwm = msg->throttle;
    }

    /*
     * Callback for receiving an image from the pi camera
     * @param The received message
    */
    void image_recvd(const sensor_msgs::ImageConstPtr& msg) {
        // Format the file names
        std::stringstream image_name, record_name;
        image_name << _dir << "image_" << _record_number << ".jpg";
        record_name << _dir << "record_" << _record_number << ".json";

        // Convert the image to grayscale and save it
        cv::Mat grayscale;
        cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, grayscale, cv::COLOR_BGR2GRAY);
        cv::imwrite(image_name.str(), grayscale);

        // Write the record file and save it
        _file_writer->open(record_name.str());
        *_file_writer << "{\"steering\": " << _steering_pwm << ", \"throttle\": " << _throttle_pwm << ", \"image\": \"image_" << _record_number << ".jpg\"}\n";
        _file_writer->close();
        // Clear the stream to be reused next callback
        _file_writer->clear();

        _record_number++;
    }
};

int main(int argc, char **argv) {
    DataStore datastore(argc, argv);
    datastore.run();

    return 0;
}
