#ifndef MYCOBOT_320PI_INTERFACE_H  
#define MYCOBOT_320PI_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include <string>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace mycobot_320pi_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Mycobot_320pi_Interface : public hardware_interface::SystemInterface
{
public:
  Mycobot_320pi_Interface();
  Mycobot_320pi_Interface(const rclcpp::Node::SharedPtr &node);
  virtual ~Mycobot_320pi_Interface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  void query_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<double> position_commands_;
  std::vector<double> prev_position_commands_;
  std::vector<double> position_states_;

  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr robot_command_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr query_subscriber_;
  
  // Optional: Publisher for actual gripper hardware commands
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gripper_hardware_publisher_;
  
  bool warning_printed_;
  rclcpp::Time start_time_; 
};
}  

#endif