#include "ydlidar_ros2/ydlidar.hpp"

using namespace chrono_literals;

namespace ydlidar
{

Ydlidar::Ydlidar() : Node("ydlidar")
{
    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1000);
    init();
    timer_ = this->create_wall_timer(33ms, std::bind(&Ydlidar::timer_callback, this));
}
Ydlidar::~Ydlidar()
{   
}

void Ydlidar::init()
{
    printf("__   ______  _     ___ ____    _    ____  \n");
    printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
    printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
    printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
    printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
    printf("\n");
    fflush(stdout);

    //Declare parameters
    this->declare_parameter<std::string>("port", "/dev/ydlidar");
    this->declare_parameter<std::string>("frame_id", "laser_frame");
    this->declare_parameter<std::string>("ignore_array", "");
    this->declare_parameter<int>("baudrate", baudrate);
    this->declare_parameter<int>("sample_rate", sample_rate);
    this->declare_parameter<bool>("resolution_fixed", true);
    this->declare_parameter<bool>("auto_reconnect", true);
    this->declare_parameter<bool>("reversion", true);
    this->declare_parameter<bool>("intensity", true);
    this->declare_parameter<bool>("low_exposure", false);
    this->declare_parameter<double>("angle_max", 180.0);
    this->declare_parameter<double>("angle_min", -180.0);
    this->declare_parameter<double>("range_max", 64.0);
    this->declare_parameter<double>("range_min", 0.01);
    this->declare_parameter<double>("frequency", 10.0);

    // Get parameters from parameter server out of launch file
    this->get_parameter<std::string>("port", port);
    this->get_parameter<std::string>("frame_id", frame_id);
    this->get_parameter<std::string>("ignore_array", list);
    this->get_parameter<int>("baudrate", baudrate);
    this->get_parameter<int>("sample_rate", sample_rate);
    this->get_parameter<bool>("resolution_fixed", resolution_fixed);
    this->get_parameter<bool>("auto_reconnect", auto_reconnect);
    this->get_parameter<bool>("reversion", reversion);
    this->get_parameter<bool>("intensity", intensities);
    this->get_parameter<bool>("low_exposure", low_exposure);
    this->get_parameter<double>("angle_max", angle_max);
    this->get_parameter<double>("angle_min", angle_min);
    this->get_parameter<double>("range_max", range_max);
    this->get_parameter<double>("range_min", range_min);
    this->get_parameter<double>("frequency", frequency);

    //Convert list to ignora_array
    ignore_array = split(list ,',');
    if(ignore_array.size()%2){
        RCLCPP_ERROR_STREAM(this->get_logger(), "ignore array is odd need be even");
    }
    for(uint16_t i =0 ; i < ignore_array.size();i++){
        if(ignore_array[i] < -180 && ignore_array[i] > 180){
            RCLCPP_ERROR_STREAM(this->get_logger(), "ignore array should be between 0 and 360");
        }
    }

    if(frequency<3){
    frequency = 7.0; 
    }
    if(frequency>15.7){
        frequency = 15.7;
    }
    if(angle_max < angle_min){
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }
    laser.setSerialPort(port);
    laser.setSerialBaudrate(baudrate);
    laser.setIntensities(intensities);
    laser.setMaxRange(range_max);
    laser.setMinRange(range_min);
    laser.setMaxAngle(angle_max);
    laser.setMinAngle(angle_min);
    laser.setReversion(reversion);
    laser.setFixedResolution(resolution_fixed);
    laser.setAutoReconnect(auto_reconnect);
    laser.setExposure(low_exposure);
    laser.setScanFrequency(frequency);
    laser.setIgnoreArray(ignore_array);
    laser.setSampleRate(sample_rate);
    ret = laser.initialize();
}

void Ydlidar::timer_callback()
{
    if(rclcpp::ok())
    {
        bool hardError;
        LaserScan scan;

        RCLCPP_INFO(this->get_logger(), std::to_string(laser.getSerialBaudrate()));
        if(laser.doProcessSimple(scan, hardError))
        {
            sensor_msgs::msg::LaserScan scan_msg;
            rclcpp::Time start_scan_time;
            scan_msg.header.stamp.sec = scan.system_time_stamp/1000000000ul;
            scan_msg.header.stamp.nanosec = scan.system_time_stamp%1000000000ul;
            scan_msg.header.frame_id = frame_id;
            scan_msg.angle_min =(scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            scan_msg.angle_increment = (scan.config.ang_increment);
            scan_msg.scan_time = 1/frequency;
            scan_msg.time_increment = 1/ (((angle_max - angle_min)/scan.config.ang_increment) *frequency);
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);
            int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.ang_increment + 1;
            scan_msg.ranges = scan.ranges;
            scan_msg.intensities = scan.intensities;
            scan_pub->publish(scan_msg);
        }
    }  

}

std::vector<float> Ydlidar::split(const std::string &s, char delim)
{
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) 
    {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}


// class Ydlidar : public rclcpp::Node
// {
// public:
//     Ydlidar() : Node("ydlidar")
//     {
//         scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1000);
//         init();
//         timer_ = this->create_wall_timer(50ms, std::bind(&Ydlidar::timer_callback, this));
//     }
//     void init()
//     {
//         printf("__   ______  _     ___ ____    _    ____  \n");
//         printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
//         printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
//         printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
//         printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
//         printf("\n");
//         fflush(stdout);

//         //Declare parameters
//         this->declare_parameter<std::string>("port", "/dev/ydlidar");
//         this->declare_parameter<std::string>("frame_id", "laser_frame");
//         this->declare_parameter<std::string>("ignore_array", "");
//         this->declare_parameter<int>("baudrate", baudrate);
//         this->declare_parameter<int>("sample_rate", sample_rate);
//         this->declare_parameter<bool>("resolution_fixed", true);
//         this->declare_parameter<bool>("auto_reconnect", true);
//         this->declare_parameter<bool>("reversion", true);
//         this->declare_parameter<bool>("isSingleChannel", isSingleChannel);
//         this->declare_parameter<bool>("isTOFlidar", isTOFLidar);
//         this->declare_parameter<double>("angle_max", 180.0);
//         this->declare_parameter<double>("angle_min", -180.0);
//         this->declare_parameter<double>("range_max", 64.0);
//         this->declare_parameter<double>("range_min", 0.01);
//         this->declare_parameter<double>("frequency", 10.0);

//         // Get parameters from parameter server out of launch file
//         this->get_parameter<std::string>("port", port);
//         this->get_parameter<std::string>("frame_id", frame_id);
//         this->get_parameter<std::string>("ignore_array", list);
//         this->get_parameter<int>("baudrate", baudrate);
//         this->get_parameter<int>("sample_rate", sample_rate);
//         this->get_parameter<bool>("resolution_fixed", resolution_fixed);
//         this->get_parameter<bool>("auto_reconnect", auto_reconnect);
//         this->get_parameter<bool>("reversion", reversion);
//         this->get_parameter<bool>("isSingleChannel", isSingleChannel);
//         this->get_parameter<bool>("isTOFlidar", isTOFLidar);
//         this->get_parameter<double>("angle_max", angle_max);
//         this->get_parameter<double>("angle_min", angle_min);
//         this->get_parameter<double>("range_max", range_max);
//         this->get_parameter<double>("range_min", range_min);
//         this->get_parameter<double>("frequency", frequency);

//         //Convert list to ignora_array
//         ignore_array = split(list ,',');
//         if(ignore_array.size()%2){
//             RCLCPP_ERROR_STREAM(this->get_logger(), "ignore array is odd need be even");
//         }
//         for(uint16_t i =0 ; i < ignore_array.size();i++){
//             if(ignore_array[i] < -180 && ignore_array[i] > 180){
//                 RCLCPP_ERROR_STREAM(this->get_logger(), "ignore array should be between 0 and 360");
//             }
//         }

//         if(frequency<3){
//         frequency = 7.0; 
//         }
//         if(frequency>15.7){
//             frequency = 15.7;
//         }
//         if(angle_max < angle_min){
//             double temp = angle_max;
//             angle_max = angle_min;
//             angle_min = temp;
//         }
//         laser.setSerialPort(port);
//         laser.setSerialBaudrate(baudrate);
//         laser.setMaxRange(range_max);
//         laser.setMinRange(range_min);
//         laser.setMaxAngle(angle_max);
//         laser.setMinAngle(angle_min);
//         laser.setReversion(reversion);
//         laser.setFixedResolution(resolution_fixed);
//         laser.setAutoReconnect(auto_reconnect);
//         laser.setScanFrequency(frequency);
//         laser.setIgnoreArray(ignore_array);
//         laser.setSampleRate(sample_rate);
//         laser.setInverted(inverted);
//         laser.setSingleChannel(isSingleChannel);
//         laser.setLidarType(isTOFLidar ? TYPE_TOF : TYPE_TRIANGLE);
//         ret = laser.initialize();
//         if (ret) {
//             ret = laser.turnOn();
//             if (!ret) {
//                 RCLCPP_ERROR(this->get_logger(), "Failed to start scan mode!!!");
//             }
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Error initializing YDLIDAR Comms and Status!!!");
//         }
//     }

//     void timer_callback()
//     {
//         if(ret&&rclcpp::ok())
//         {
//             bool hardError;
//             LaserScan scan;
//             if(laser.doProcessSimple(scan, hardError))
//             {
//                 sensor_msgs::msg::LaserScan scan_msg;
//                 rclcpp::Time start_scan_time;
//                 scan_msg.header.stamp.sec = scan.stamp/1000000000ul;
//                 scan_msg.header.stamp.nanosec = scan.stamp%1000000000ul;
//                 scan_msg.header.frame_id = frame_id;
//                 scan_msg.angle_min =(scan.config.min_angle);
//                 scan_msg.angle_max = (scan.config.max_angle);
//                 scan_msg.angle_increment = (scan.config.angle_increment);
//                 scan_msg.scan_time = scan.config.scan_time;
//                 scan_msg.time_increment = scan.config.time_increment;
//                 scan_msg.range_min = (scan.config.min_range);
//                 scan_msg.range_max = (scan.config.max_range);
//                 int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
//                 scan_msg.ranges.resize(size);
//                 scan_msg.intensities.resize(size);
//                 for(int i=0; i < scan.points.size(); i++) {
//                     int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
//                     if(index >=0 && index < size) {
//                         scan_msg.ranges[index] = scan.points[i].range;
//                         scan_msg.intensities[index] = scan.points[i].intensity;
//                     }
//                 }
//             scan_pub->publish(scan_msg);
//             }  
//         }
//     }
// private:
//     std::vector<float> split(const std::string &s, char delim)
//     {
//         std::vector<float> elems;
//         std::stringstream ss(s);
//         std::string number;
//         while(std::getline(ss, number, delim)) 
//         {
//             elems.push_back(atof(number.c_str()));
//         }
//         return elems;
//     }   


//     std::string port, frame_id, list;
//     int baudrate=230400;
//     bool reversion, resolution_fixed, auto_reconnect, ret;
//     double angle_max, angle_min,range_max, range_min, frequency;
//     result_t op_result;
//     std::vector<float> ignore_array;  
//     int sample_rate = 5;
//     bool inverted = true;
//     bool isSingleChannel = false;
//     bool isTOFLidar = false;
//     CYdLidar laser;
//     rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

} // namespace ydlidar