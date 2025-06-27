#include "ultrasonic_sensor/ultrasonic_sensor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<UltrasonicSensorNode>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();
    
  rclcpp::shutdown();
}
