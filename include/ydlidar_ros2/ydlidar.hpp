#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "CYdLidar.h"

namespace ydlidar
{
class Ydlidar : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Ydlidar object
     * 
     */
    Ydlidar();
    /**
     * @brief Destroy the Ydlidar object
     * 
     */
    ~Ydlidar();

    /**
     * @brief Initialisation function to declare and initialise parameters and class variables
     * 
     */
    void init();

    /**
     * @brief Function called at a rate of 20Hz to publish the laser scan data
     * 
     */
    void timer_callback();

private:

    /**
     * @brief Helper function to split the std::string parameter into array form
     * 
     * @param [input] s the input string 
     * @param [input] delim delimiter used for separation 
     * @return [output] std::vector<float> the output array
     */
    std::vector<float> split(const std::string &s, char delim);

    std::string port, frame_id, list, model;
    int baudrate=230400;
    bool reversion, resolution_fixed, auto_reconnect, ret, low_exposure, intensities;
    double angle_max, angle_min,range_max, range_min, frequency;
    result_t op_result;
    std::vector<float> ignore_array;  
    int sample_rate = 4;
    bool inverted = true;
    CYdLidar laser;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};

}