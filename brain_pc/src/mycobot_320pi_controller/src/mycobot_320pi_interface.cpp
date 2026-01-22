#include "mycobot_320pi_controller/mycobot_320pi_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iomanip>  
#include <sstream> 
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <algorithm>

namespace mycobot_320pi_controller
{

double degreesToRadians(double degrees) {
  const double pi = 3.14159265358979323846;
  return degrees * (pi / 180.0);
}

Mycobot_320pi_Interface::Mycobot_320pi_Interface() 
{
    node_ = std::make_shared<rclcpp::Node>("Mycobot_320pi_Interface");
    warning_printed_ = false;
}

Mycobot_320pi_Interface::Mycobot_320pi_Interface(const rclcpp::Node::SharedPtr &node)
    : node_(node), warning_printed_(false)
{
}

Mycobot_320pi_Interface::~Mycobot_320pi_Interface()
{
}

CallbackReturn Mycobot_320pi_Interface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    robot_command_publisher_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>("/robot_cmd_pub_", rclcpp::QoS(10));

    query_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/query_response", rclcpp::QoS(10), 
        std::bind(&Mycobot_320pi_Interface::query_callback, this, std::placeholders::_1));

    // Optional: Create publisher for gripper hardware commands
    gripper_hardware_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/gripper_hardware_command", rclcpp::QoS(10));

    position_commands_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size());
    prev_position_commands_.resize(info_.joints.size());
    start_time_ = node_->get_clock()->now();

    RCLCPP_INFO(node_->get_logger(), "Robot 2030A Interface initialized successfully!");
    RCLCPP_INFO(node_->get_logger(), "Number of joints: %zu", info_.joints.size());
    
    // Log all joint names for debugging
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Joint %zu: %s", i, info_.joints[i].name.c_str());
    }
    
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Mycobot_320pi_Interface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide position interface for all joints
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Mycobot_320pi_Interface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide position interface for all joints
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn Mycobot_320pi_Interface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(node_->get_logger(), "Starting robot hardware ...");

  // Reset commands and states
  std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
  std::fill(prev_position_commands_.begin(), prev_position_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);

  RCLCPP_INFO(node_->get_logger(), "Hardware started, ready to take commands");
  RCLCPP_INFO(node_->get_logger(), "Number of joints: %zu", position_commands_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn Mycobot_320pi_Interface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(node_->get_logger(), "Stopping robot hardware ...");
  RCLCPP_INFO(node_->get_logger(), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Mycobot_320pi_Interface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  rclcpp::spin_some(node_);

  if ((query_subscriber_->get_publisher_count() == 0) &&  
    ((node_->get_clock()->now() - start_time_).seconds() > 30.0))
  {
    if (!warning_printed_)
    {
        RCLCPP_WARN(node_->get_logger(), "\033[1;31m--------------------------- System is not Closeloop !!! ---------------------------\033[0m");
        warning_printed_ = true;
    }
    
    // For simulation or when no feedback, set states equal to commands
    for (size_t i = 0; i < position_states_.size(); i++)
    {
        position_states_[i] = position_commands_[i];
    }
    
    return hardware_interface::return_type::OK;
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Mycobot_320pi_Interface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
    // Create message for arm joints (first 6 joints)
    auto int_msg = std_msgs::msg::Int32MultiArray();
    int_msg.data.clear();

    // Send commands for arm joints (first 6)
    size_t num_arm_joints = std::min(position_commands_.size(), (size_t)6);
    
    for (size_t i = 0; i < num_arm_joints; ++i)
    {
        int angle = static_cast<int>((((position_commands_.at(i) - (M_PI / 2)) * 180) / M_PI) + 90);
        int rounded_angle = std::round(angle * 100.0f) / 100.0f;

        if (std::fabs(rounded_angle) < std::numeric_limits<int>::epsilon())
        {
            rounded_angle = 0.0f; 
        }

        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << rounded_angle;
        int formatted_angle = std::stof(stream.str());

        // Multiply the value by 1000
        formatted_angle *= 1000;

        int_msg.data.push_back(formatted_angle);
    }

    // Publish the arm command
    robot_command_publisher_->publish(int_msg);

    if (position_commands_.size() > 6)
    {
        for (size_t i = 6; i < position_commands_.size(); ++i)
        {
            // Send gripper commands to hardware
            auto gripper_msg = std_msgs::msg::Float32MultiArray();
            gripper_msg.data = {static_cast<float>(position_commands_[i])};
            gripper_hardware_publisher_->publish(gripper_msg);
            
            RCLCPP_DEBUG(node_->get_logger(), "Gripper joint %zu command: %.3f", 
                        i, position_commands_[i]);
        }
    }

    prev_position_commands_ = position_commands_;

    return hardware_interface::return_type::OK;
}

void Mycobot_320pi_Interface::query_callback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg)
{
    if (msg->data.size() < 6)
    {
        RCLCPP_ERROR(node_->get_logger(), "Received message with insufficient data size.");
        return;
    }

    // Update arm joint states (first 6 joints)
    for (size_t i = 0; i < 6 && i < position_states_.size(); ++i)
    {
      position_states_.at(i) = degreesToRadians(msg->data.at(i));
    }
    
}

}  

PLUGINLIB_EXPORT_CLASS(mycobot_320pi_controller::Mycobot_320pi_Interface, hardware_interface::SystemInterface)