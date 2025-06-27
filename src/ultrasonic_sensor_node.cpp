#include "ultrasonic_sensor/ultrasonic_sensor_node.hpp"

UltrasonicSensorNode::UltrasonicSensorNode(const rclcpp::NodeOptions& options)
  : Node("ultrasonic_sensor_node", options)
{
  declare_parameter<bool>("sim", true);
  get_parameter<bool>("sim", simulation_);

  declare_parameter<float>("field_of_view", 0.0);
  get_parameter<float>("field_of_view", field_of_view_);

  declare_parameter<float>("min_range", 0.0);
  get_parameter<float>("min_range", min_range_);

  declare_parameter<float>("max_range", 0.0);
  get_parameter<float>("max_range", max_range_);

  std::string co_dev_ns;

  declare_parameter<std::string>("co_dev_ns", "canopen_module");
  get_parameter<std::string>("co_dev_ns", co_dev_ns);

  VALID_RPDO_INDEXES = {0x6010, 0x6011, 0x6012};

  pub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpdo_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions rpdo_options;
  rpdo_options.callback_group = rpdo_cbg_;

  pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&UltrasonicSensorNode::pub_cb, this), 
    pub_cbg_);

  range_pub_ = create_publisher<Range>("ultrasonic_range", 10); 
  temp_pub_ = create_publisher<Temperature>("ultrasonic_sensor_internal_temperature", 10); 
  tpdo_pub_ = create_publisher<COData>("/" + co_dev_ns + "/tpdo", 10); 

  rpdo_sub_ = create_subscription<COData>(
    "/" + co_dev_ns + "/rpdo", 
    1000,
    std::bind(&UltrasonicSensorNode::rpdo_cb, this, _1),
    rpdo_options);

  ctrl_ultrasonic_sensor(true);

  RCLCPP_INFO(get_logger(), "Ultrasonic Sensor Node is up.");
}

UltrasonicSensorNode::~UltrasonicSensorNode()
{
  if (rclcpp::ok())
  {
    ctrl_ultrasonic_sensor(false);
  }
}

void UltrasonicSensorNode::ctrl_ultrasonic_sensor(bool enable)
{
  COData msg;

  msg.index = 0x601F;
  msg.subindex = 0x0;
  msg.data = enable ? 0x1 : 0x0;

  tpdo_pub_->publish(msg);

  RCLCPP_WARN(get_logger(), "Ultrasonic Sensor is %s", enable ? "enabled" : "disabled");
}

bool UltrasonicSensorNode::is_valid_rpdo_index(uint16_t index) const
{
  return VALID_RPDO_INDEXES.find(index) != VALID_RPDO_INDEXES.end();
}

void UltrasonicSensorNode::rpdo_cb(const COData::SharedPtr msg)
{
  if (!is_valid_rpdo_index(msg->index)) 
    return;

  const std::lock_guard<std::mutex> lock(mutex_);

  switch (msg->index)
  {
    case 0x6010:
      range_ = static_cast<float>(msg->data) / 10.0f;
    break;
    case 0x6011:
      temp_ = static_cast<float>(msg->data);
    break;
    case 0x6012:
      // time of flight
    break;
  }
}

void UltrasonicSensorNode::pub_cb(void)
{
  Range range_msg;
  range_msg.header.stamp = get_clock()->now();
  
  Temperature temp_msg;
  temp_msg.header.stamp = get_clock()->now();

  if (simulation_)
  {
    range_msg.field_of_view = 0.007;
    range_msg.min_range = 0.07;
    range_msg.max_range = 0.77;
    range_msg.range = 0.7;

    temp_msg.temperature = 25.52;
  }
  else
  {
    const std::lock_guard<std::mutex> lock(mutex_);

    range_msg.field_of_view = field_of_view_;
    range_msg.min_range = min_range_;
    range_msg.max_range = max_range_;
    range_msg.range = range_;

    temp_msg.temperature = temp_;
  }

  range_pub_->publish(range_msg);
  temp_pub_->publish(temp_msg);
}