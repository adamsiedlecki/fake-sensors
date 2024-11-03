#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/opencv.hpp"

class CameraConfig {
public:
    const char* PARAM_WIDTH = "width";
    const char* PARAM_HEIGHT = "height";
    const char* PARAM_MP4 = "mp4";
    const char* PARAM_FPS = "fps";

    int DEFAULT_WIDTH = 1920;
    int DEFAULT_HEIGHT = 1080;
    std::string DEFAULT_MP4 = "me.mp4";
    int DEFAULT_FPS = 30;

    const char* DESCRIPTION_WIDTH = "width of image";
    const char* DESCRIPTION_HEIGHT = "height of image";
    const char* DESCRIPTION_MP4 = "path to mp4 file that is a mock for real camera";
    const char* DESCRIPTION_FPS = "frames per second";

public:
    int width;
    int height;
    std::string mp4_path;
    int fps;
public:
    int get_duration_between_frames_ms();
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    void print_config(rclcpp::Node *node);
};

void CameraConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_WIDTH;
    node->declare_parameter(PARAM_WIDTH, DEFAULT_WIDTH, descriptor);

    descriptor.description = DESCRIPTION_HEIGHT;
    node->declare_parameter(PARAM_HEIGHT, DEFAULT_HEIGHT, descriptor);

    descriptor.description = DESCRIPTION_MP4;
    node->declare_parameter(PARAM_MP4, DEFAULT_MP4, descriptor);

    descriptor.description = DESCRIPTION_FPS;
    node->declare_parameter(PARAM_FPS, DEFAULT_FPS, descriptor);
}

void CameraConfig::update_parameters(rclcpp::Node *node)
{
    this->width = node->get_parameter(PARAM_WIDTH).as_int();
    this->height = node->get_parameter(PARAM_HEIGHT).as_int();
    this->mp4_path = node->get_parameter(PARAM_MP4).as_string();
    this->fps = node->get_parameter(PARAM_FPS).as_int();
}

void CameraConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_WIDTH, this->width);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_HEIGHT, this->height);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_MP4, this->mp4_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_FPS, this->fps);
}

int CameraConfig::get_duration_between_frames_ms() {
    return 1000/fps;
}

class FakeCamera: public rclcpp::Node {
public:
    FakeCamera();
    void publish_frame();
private:
    CameraConfig camera_config;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    std::vector<unsigned char> get_next_frame();

private:
    cv::VideoCapture video_capture;
    bool is_video_from_file;
};

FakeCamera::FakeCamera(): Node("fake_camera") {
    camera_config.declare_parameters(this);
    camera_config.update_parameters(this);
    camera_config.print_config(this);
    image_publisher = this->create_publisher<sensor_msgs::msg::Image>("/camera",10);

    if (camera_config.mp4_path.empty()) {
        video_capture.open(0);
        is_video_from_file = false;
    } else {
        video_capture.open(camera_config.mp4_path);
        is_video_from_file = true;
    }

    timer = this->create_wall_timer(
        std::chrono::milliseconds(camera_config.get_duration_between_frames_ms()),
            std::bind(&FakeCamera::publish_frame, this)
    );
}

std::vector<unsigned char> FakeCamera::get_next_frame() {
    cv::Mat frame;
    if (!video_capture.read(frame)) {
        if (is_video_from_file) {
            RCLCPP_INFO(this->get_logger(), "Resetting video back to first frame.");
            video_capture.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (!video_capture.read(frame)) {
                RCLCPP_INFO(this->get_logger(), "Cannot read any frame from file. Changing to default camera.");
                video_capture.open(0);

                    int width = static_cast<int>(video_capture.get(cv::CAP_PROP_FRAME_WIDTH));
                    int height = static_cast<int>(video_capture.get(cv::CAP_PROP_FRAME_HEIGHT));
                    RCLCPP_INFO(this->get_logger(), "Real camera width: %d height: %d", width, height);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Cannot read frame from real device camera.");
        }
    }
    return std::vector<unsigned char>(frame.data, frame.data + frame.total() * frame.elemSize());
}

void FakeCamera::publish_frame()
{
    auto msg = sensor_msgs::msg::Image();
    
    msg.header.stamp = this->now();
    msg.header.frame_id = "image_frame";

    msg.width = camera_config.width;
    msg.height = camera_config.height;
    msg.step = camera_config.width * 3; // rgb = 3 bytes

    msg.data = get_next_frame();

    // nie mam pewności do poniższych
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    

    image_publisher->publish(msg);

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeCamera>());
    rclcpp::shutdown();
    return 0;
}