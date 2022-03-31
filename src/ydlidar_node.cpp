#include "ydlidar_ros2/ydlidar.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ydlidar>());
    rclcpp::shutdown();
    return 0;
}