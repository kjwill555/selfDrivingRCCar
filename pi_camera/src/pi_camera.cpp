#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/*
 * Node for continuous capture on the Raspberry Pi Camera
*/
class PiCamera {
public:
    /*
     * Constructor. Opens the camera and warms it up
     * @param argc from main()
     * @param argv from main()
    */
    PiCamera(int argc, char** argv) {
        // Initialize the node
        ros::init(argc, argv, "pi_camera");
        // Create the publisher node
        _nh = std::make_unique<ros::NodeHandle>();
        _image_node = std::make_unique<image_transport::ImageTransport>(*_nh);
        _pub = std::make_unique<image_transport::Publisher>(_image_node->advertise("camera/image", 20));
        // Open the camera
        _cap = std::make_unique<cv::VideoCapture>(0);
        if (!_cap->isOpened()) {
            ROS_ERROR("Could not open camera");
            return;
        }

        // Set camera settings
        _cap->set(cv::CAP_PROP_FPS, 20);
        _cap->set(cv::CAP_PROP_FRAME_HEIGHT, 120);
        _cap->set(cv::CAP_PROP_FRAME_WIDTH, 160);
        
        // Warm up the camera
        cv::Mat frame;
        ROS_INFO("Warming up camera...");
        *_cap >> frame;
        usleep(3000000);
    }

    /*
     * Run the node
    */
    void run() {
        cv::Mat frame;
        sensor_msgs::ImagePtr msg;
        // Start the publish loop
        ros::Rate loop_rate(20);
        while (_nh->ok()) {
            *_cap >> frame;
            // Publish the image if its nonempty
            if(!frame.empty()) {
                // Create an image message
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                _pub->publish(msg);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    std::unique_ptr<ros::NodeHandle> _nh;
    std::unique_ptr<image_transport::ImageTransport> _image_node;
    std::unique_ptr<image_transport::Publisher> _pub;
    std::unique_ptr<cv::VideoCapture> _cap;
};

int main(int argc, char** argv) {
    PiCamera camera(argc, argv);
    camera.run();

    return 0;
}
