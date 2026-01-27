#ifndef MYCOBOT320PI_INTERFACE_H  
#define MYCOBOT320PI_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include <string>
#include <mutex>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"

namespace mycobot_320pi_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Mycobot320piInterface : public hardware_interface::SystemInterface
{
public:
  Mycobot320piInterface();
  Mycobot320piInterface(const rclcpp::Node::SharedPtr &node);
  virtual ~Mycobot320piInterface();

  // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
  virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // Implementing hardware_interface::SystemInterface
  virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  void query_callback_arm(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg);
  void query_callback_gripper(const std_msgs::msg::Float32::ConstSharedPtr &msg);
  void CalibrationArm(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<double> position_commands_;
  std::vector<double> prev_position_commands_;
  std::vector<double> position_states_;
  
  // Mutex for thread-safe access
  std::mutex state_mutex_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr robot_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_command_publisher_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr query_arm_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr query_gripper_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr calibration_arm_sub_;
  
  bool warning_openloop_;
  bool warning_closeloop_;
  rclcpp::Time start_time_; 

  // Calibration parameters
  double arm_joint_1_radians_to_degrees_{0.0};
  double arm_joint_2_radians_to_degrees_{0.0};
  double arm_joint_3_radians_to_degrees_{0.0};
  double arm_joint_4_radians_to_degrees_{0.0};
  double arm_joint_5_radians_to_degrees_{0.0};
  double arm_joint_6_radians_to_degrees_{0.0};
  double arm_joint_1_degrees_to_radians_{0.0};
  double arm_joint_2_degrees_to_radians_{0.0};
  double arm_joint_3_degrees_to_radians_{0.0};
  double arm_joint_4_degrees_to_radians_{0.0};
  double arm_joint_5_degrees_to_radians_{0.0};
  double arm_joint_6_degrees_to_radians_{0.0};

  double gripper_radians_to_degrees_{0.0};
  double gripper_degrees_to_radians_{0.0};

  std::vector<double> prev_arm_calibrations_ = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  bool calibration_initialized_;

  rclcpp::Logger logger_{rclcpp::get_logger("Mycobot320piInterface")};

};

}  // namespace mycobot_320pi_controller

#endif  // MYCOBOT320PI_INTERFACE_H