//=================================================================================================
//                                          INCLUDES
//=================================================================================================
#include <chrono>
#include <memory>
#include <cmath>
#include <utility>
#include <fstream>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


//=================================================================================================
//                                        LIDAR CONFIG 
//=================================================================================================
class LidarConfig
{
public:
    const char* PARAM_MIN_RANGE = "min_range";
    const char* PARAM_MAX_RANGE = "max_range";
    const char* PARAM_MIN_ANGLE = "min_angle";
    const char* PARAM_MAX_ANGLE = "max_angle";
    const char* PARAM_SAMPLE_COUNT = "sample_count";
    const char* PARAM_SAMPLING_FREQUENCY = "sampling_frequency";
    const char* PARAM_CSV_LIDAR_MOCK_FILE = "csv_lidar_mock_file";
    const char* PARAM_CSV_STD_DEV = "csv_std_dev";

    const double DEFAULT_MIN_RANGE = 0.2;
    const double DEFAULT_MAX_RANGE = 20.0;
    const double DEFAULT_MIN_ANGLE = -M_PI;
    const double DEFAULT_MAX_ANGLE = M_PI;
    const int DEFAULT_SAMPLE_COUNT = 360;
    const double DEFAULT_SAMPLING_FREQUENCY = 10.0;
    const char* DEFAULT_CSV_LIDAR_MOCK_FILE = "lidar_mock.csv";
    const double DEFAULT_CSV_STD_DEV = 1;

    const char* DESCRIPTION_MIN_RANGE = "minimum range value [m]";
    const char* DESCRIPTION_MAX_RANGE = "maximum range value [m]";
    const char* DESCRIPTION_MIN_ANGLE = "start angle of the scan [rad]";
    const char* DESCRIPTION_MAX_ANGLE = "end angle of the scan [rad]";
    const char* DESCRIPTION_SAMPLE_COUNT = "Number of samples per full laser scan";
    const char* DESCRIPTION_SAMPLING_FREQUENCY = "Number of full Scans per second.";
    const char* DESCRIPTION_CSV = "Csv file with mock data for fake lidar. One row = one full scan.";
    const char* DESCRIPTION_CSV_STD_DEV = "Standard deviation for gauss noise";

public:
    std::pair<double,double> range;
    std::pair<double,double> angle;
    int sample_count;
    double sampling_frequency;
    std::string csv_path;
    std::vector<std::vector<float>> csv_data;
    double std_dev;

public:
    void declare_parameters(rclcpp::Node *node);
    void update_parameters(rclcpp::Node *node);
    void print_config(rclcpp::Node *node);
    void load_csv_data(rclcpp::Node *node);


    double get_scan_step() const;
    int get_scan_period_ms() const;
    std::vector<float> get_next_fake_scan();
    double get_noise();
    LidarConfig();

private:
    unsigned long next_fake_scan = 0;

    std::random_device rd;
    std::mt19937 generator;
    std::normal_distribution<double> distribution;
};

LidarConfig::LidarConfig(): generator(rd()) {}

double LidarConfig::get_noise() {
    return distribution(generator);
}

int LidarConfig::get_scan_period_ms() const
{
    return std::lround(1000/sampling_frequency);
}

double LidarConfig::get_scan_step() const
{
    return (angle.second-angle.first)/sample_count;
}    

void LidarConfig::declare_parameters(rclcpp::Node *node)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

    descriptor.description = DESCRIPTION_MIN_RANGE;
    node->declare_parameter(PARAM_MIN_RANGE, DEFAULT_MIN_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MAX_RANGE;
    node->declare_parameter(PARAM_MAX_RANGE, DEFAULT_MAX_RANGE, descriptor);
    descriptor.description = DESCRIPTION_MIN_ANGLE;
    node->declare_parameter(PARAM_MIN_ANGLE, DEFAULT_MIN_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_MAX_ANGLE;
    node->declare_parameter(PARAM_MAX_ANGLE, DEFAULT_MAX_ANGLE, descriptor);
    descriptor.description = DESCRIPTION_SAMPLE_COUNT;
    node->declare_parameter(PARAM_SAMPLE_COUNT, DEFAULT_SAMPLE_COUNT, descriptor);
    descriptor.description = DESCRIPTION_SAMPLING_FREQUENCY;
    node->declare_parameter(PARAM_SAMPLING_FREQUENCY, DEFAULT_SAMPLING_FREQUENCY, descriptor);
    descriptor.description = DESCRIPTION_CSV;
    node->declare_parameter(PARAM_CSV_LIDAR_MOCK_FILE, DEFAULT_CSV_LIDAR_MOCK_FILE, descriptor);
        descriptor.description = DESCRIPTION_CSV_STD_DEV;
    node->declare_parameter(PARAM_CSV_STD_DEV, DEFAULT_CSV_STD_DEV, descriptor);
}

void LidarConfig::update_parameters(rclcpp::Node *node)
{
    this->range.first = node->get_parameter(PARAM_MIN_RANGE).as_double();
    this->range.second = node->get_parameter(PARAM_MAX_RANGE).as_double();
    this->angle.first = node->get_parameter(PARAM_MIN_ANGLE).as_double();
    this->angle.second = node->get_parameter(PARAM_MAX_ANGLE).as_double();
    this->sampling_frequency = node->get_parameter(PARAM_SAMPLING_FREQUENCY).as_double();
    this->csv_path = node->get_parameter(PARAM_CSV_LIDAR_MOCK_FILE).as_string();
    this->std_dev = node->get_parameter(PARAM_CSV_STD_DEV).as_double();

    this->load_csv_data(node);
    this->sample_count = csv_data.at(0).size(); //  ilość odczytów na pełny skan powinna odpowiadać ilości kolumn w pliku CSV

    std::normal_distribution<double> distribution(0.0, this->std_dev);
}

void LidarConfig::print_config(rclcpp::Node *node)
{
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_RANGE, this->range.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_RANGE, this->range.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MIN_ANGLE, this->angle.first);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_MAX_ANGLE, this->angle.second);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %d", PARAM_SAMPLE_COUNT, this->sample_count);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_SAMPLING_FREQUENCY,this->sampling_frequency);
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %s", PARAM_CSV_LIDAR_MOCK_FILE,this->csv_path.c_str());
    RCLCPP_INFO(node->get_logger(), "Parameter %s = %f", PARAM_CSV_STD_DEV,this->std_dev);
    RCLCPP_INFO(node->get_logger(), "Scan step = %f", this->get_scan_step());
}

std::vector<float> LidarConfig::get_next_fake_scan() {
    if (next_fake_scan >= csv_data.size()) {
        next_fake_scan = 0;
    }
    std::vector<float> scan = csv_data.at(next_fake_scan);
    next_fake_scan++;
    return scan;
}

void LidarConfig::load_csv_data(rclcpp::Node *node) {
    std::ifstream file(this->csv_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Could not open file under path: %s", csv_path.c_str());
        return;
    }
    std::string line;
    int rowCounter = 0;
    while (std::getline(file, line)) {
        rowCounter++;
        std::vector<float> scan_data;
        std::stringstream ss(line);
        std::string value;
        bool isDataErrorInRow = false;
        float valueOutOfScope;
        while (std::getline(ss, value, ',') && !isDataErrorInRow) {
            float input = std::stof(value);
            if(input < this->range.first || input > range.second) {
                isDataErrorInRow = true;
                valueOutOfScope = input;
                continue;
            }
            scan_data.push_back(input);
        }
        if (isDataErrorInRow) {
            RCLCPP_ERROR(node->get_logger(), "There is error in csv data in row: %d - value %f is out of scope.", rowCounter, valueOutOfScope);
        } else {
            this->csv_data.push_back(scan_data);
        }
    }
    file.close();
}

//=================================================================================================
//                                         FAKE LIDAR NODE 
//=================================================================================================
class FakeLidar: public rclcpp::Node
{
public:
    FakeLidar();

private:
    LidarConfig _config;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _scan_publisher;
    rclcpp::TimerBase::SharedPtr _scan_timer;

private:
    void _publish_fake_scan();
};

FakeLidar::FakeLidar(): Node("fake_lidar")
{
    _config.declare_parameters(this);
    _config.update_parameters(this);
    _config.print_config(this);
    _scan_publisher = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan",10);
    _scan_timer = this->create_wall_timer(
        std::chrono::milliseconds(_config.get_scan_period_ms()),
        std::bind(&FakeLidar::_publish_fake_scan, this)
    );
}
    
void FakeLidar::_publish_fake_scan()
{
    auto msg = sensor_msgs::msg::LaserScan();
    
    msg.header.stamp = this->now();
    msg.header.frame_id = "laser_frame";

    msg.angle_min = _config.angle.first;
    msg.angle_max = _config.angle.second;
    msg.range_min = _config.range.first;
    msg.range_max = _config.range.second;
    msg.angle_increment = _config.get_scan_step();
    msg.time_increment = 0;
    msg.scan_time = _config.get_scan_period_ms()/1000.0;

    std::vector<float> ranges(_config.sample_count);
    std::vector<float> fake_scan = _config.get_next_fake_scan();
    for(int i=0; i<_config.sample_count; i++)
    {
        double noise = 0;
        if (_config.std_dev != 0) {
            noise = _config.get_noise();
        }
        ranges[i] = fake_scan.at(i) + noise;
    }
    msg.ranges = ranges;
    _scan_publisher->publish(msg);

}

//=================================================================================================
//                                          MAIN 
//=================================================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<FakeLidar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
