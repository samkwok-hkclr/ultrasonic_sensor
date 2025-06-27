#ifndef ULTRASONIC_SENSOR_NODE_HPP__
#define ULTRASONIC_SENSOR_NODE_HPP__

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "canopen_interfaces/msg/co_data.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class UltrasonicSensorNode : public rclcpp::Node
{
  using Temperature = sensor_msgs::msg::Temperature;
  using Range = sensor_msgs::msg::Range;
  using COData = canopen_interfaces::msg::COData;

public:
  explicit UltrasonicSensorNode(const rclcpp::NodeOptions& options);
  ~UltrasonicSensorNode();

  void ctrl_ultrasonic_sensor(bool enable);

  bool is_valid_rpdo_index(uint16_t index) const;

  void pub_cb(void);

  void rpdo_cb(const COData::SharedPtr msg);

private:
  bool simulation_;

  std::mutex mutex_;

  float field_of_view_;
  float min_range_;
  float max_range_;
  float range_;

  float temp_;

  rclcpp::CallbackGroup::SharedPtr pub_cbg_;
  rclcpp::CallbackGroup::SharedPtr rpdo_cbg_;

  rclcpp::TimerBase::SharedPtr pub_timer_;

  rclcpp::Publisher<Range>::SharedPtr range_pub_;
  rclcpp::Publisher<Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<COData>::SharedPtr tpdo_pub_;

  rclcpp::Subscription<COData>::SharedPtr rpdo_sub_;

  std::unordered_set<uint16_t> VALID_RPDO_INDEXES;

  // rclcpp::Service<???>::SharedPtr ???_srv_;

};

#endif