#include "mycobot_320pi_controller/mycobot_320pi_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iomanip>  
#include <sstream> 
#include <cmath>
#include <algorithm>

namespace mycobot_320pi_controller
{

Mycobot320piInterface::Mycobot320piInterface() 
{
    node_ = std::make_shared<rclcpp::Node>("Mycobot320piInterface");
    warning_openloop_ = false;
    warning_closeloop_ = false;
}

Mycobot320piInterface::Mycobot320piInterface(const rclcpp::Node::SharedPtr &node)
    : node_(node), 
      warning_openloop_(false),
      warning_closeloop_(false)
{
}

Mycobot320piInterface::~Mycobot320piInterface()
{
}

CallbackReturn Mycobot320piInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    robot_command_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/arm_hardware_command", rclcpp::QoS(10));
    
    gripper_command_publisher_ = node_->create_publisher<std_msgs::msg::Float32>(
        "/gripper_hardware_command", rclcpp::QoS(10));

    query_arm_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/query_response_arm", rclcpp::QoS(10), 
        std::bind(&Mycobot320piInterface::query_callback_arm, this, std::placeholders::_1));
    
    query_gripper_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/query_response_gripper", rclcpp::QoS(10), 
        std::bind(&Mycobot320piInterface::query_callback_gripper, this, std::placeholders::_1));    
    
    calibration_arm_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/arm_calibrations", rclcpp::QoS(10), 
        std::bind(&Mycobot320piInterface::CalibrationArm, this, std::placeholders::_1));    

    position_commands_.resize(info_.joints.size());
    position_states_.resize(info_.joints.size());
    prev_position_commands_.resize(info_.joints.size());
    start_time_ = node_->get_clock()->now();

    RCLCPP_INFO(node_->get_logger(), "Mycobot320pi Interface initialized successfully!");
    RCLCPP_INFO(node_->get_logger(), "Number of joints: %zu", info_.joints.size());
    
    // Log all joint names for debugging
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Joint %zu: %s", i, info_.joints[i].name.c_str());
    }
    
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Mycobot320piInterface::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> Mycobot320piInterface::export_command_interfaces()
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

CallbackReturn Mycobot320piInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    RCLCPP_INFO(node_->get_logger(), "Starting robot hardware ...");

    // Reset commands and states
    std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
    std::fill(prev_position_commands_.begin(), prev_position_commands_.end(), 0.0);
    std::fill(position_states_.begin(), position_states_.end(), 0.0);

    // Reset warning flags
    warning_openloop_ = false;
    warning_closeloop_ = false;
    start_time_ = node_->get_clock()->now();

    RCLCPP_INFO(node_->get_logger(), "Hardware started, ready to take commands");
    RCLCPP_INFO(node_->get_logger(), "Number of joints: %zu", position_commands_.size());
    return CallbackReturn::SUCCESS;
}

CallbackReturn Mycobot320piInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
    RCLCPP_INFO(node_->get_logger(), "Stopping robot hardware ...");
    RCLCPP_INFO(node_->get_logger(), "Hardware stopped");
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Mycobot320piInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
    rclcpp::spin_some(node_);

    // Check if feedback is available
    bool arm_feedback_available = (query_arm_subscriber_->get_publisher_count() > 0);
    bool gripper_feedback_available = (query_gripper_subscriber_->get_publisher_count() > 0);
    
    // Check time elapsed since start
    double time_elapsed = (node_->get_clock()->now() - start_time_).seconds();
    
    // Open-loop warning (no feedback available)
    if (!arm_feedback_available && time_elapsed > 2.0)
    {
        if (!warning_openloop_)
        {
            RCLCPP_WARN(node_->get_logger(), 
                "\033[1;31m--------------------------- System is not in Closed Loop !!! ---------------------------\033[0m");
            RCLCPP_WARN(node_->get_logger(), 
                "No arm feedback received. Operating in open-loop mode.");
            warning_openloop_ = true;
        }
    }
    
    // Closed-loop mode (feedback available)
    if (arm_feedback_available && !warning_closeloop_ && time_elapsed > 2.0)
    {
        RCLCPP_INFO(node_->get_logger(), 
            "\033[1;32m--------------------------- System is in Closed Loop ---------------------------\033[0m");
        RCLCPP_INFO(node_->get_logger(), 
            "Arm feedback is available. Operating in closed-loop mode.");
        warning_closeloop_ = true;
    }
    
    // For simulation or when no feedback, set states equal to commands
    if (!arm_feedback_available && time_elapsed > 2.0)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < position_states_.size(); i++)
        {
            position_states_[i] = position_commands_[i];
        }
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Mycobot320piInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    
    // Initialize prev_position_commands_ on first call or size mismatch
    if (prev_position_commands_.size() != position_commands_.size())
    {
        prev_position_commands_ = position_commands_;
        // Force publishing on first call to send initial positions
    }
    
    // Check if commands have changed SIGNIFICANTLY
    bool significant_change = false;
    bool all_zero = true;
    
    // Use a larger threshold to filter out tiny movements
    const double SIGNIFICANT_CHANGE_THRESHOLD = 0.0001;  // 0.001 rad = ~0.057°
    const double ZERO_THRESHOLD = 0.0001;                // 0.001 rad = ~0.057°
    
    
        
    // DEBUG: Log changes
    for (size_t i = 0; i < position_commands_.size(); i++)
    {
        double diff = std::abs(position_commands_[i] - prev_position_commands_[i]);
        
        
        // Check if this command is non-zero (using larger threshold)
        if (std::abs(position_commands_[i]) > ZERO_THRESHOLD)
        {
            all_zero = false;
        }
    }
    
    
    // Create message for arm joints (first 6 joints)
    auto arm_msg = std_msgs::msg::Float32MultiArray();
    arm_msg.data.clear();

    // Send commands for arm joints (first 6)
    size_t num_arm_joints = std::min(position_commands_.size(), (size_t)6);
    
    // Array of calibration factors for each joint
    const std::array<double, 6> calibration_factors = {
        (arm_joint_1_radians_to_degrees_ != 0.0) ? arm_joint_1_radians_to_degrees_ : 1.0,
        (arm_joint_2_radians_to_degrees_ != 0.0) ? arm_joint_2_radians_to_degrees_ : 1.0,
        (arm_joint_3_radians_to_degrees_ != 0.0) ? arm_joint_3_radians_to_degrees_ : 1.0,
        (arm_joint_4_radians_to_degrees_ != 0.0) ? arm_joint_4_radians_to_degrees_ : 1.0,
        (arm_joint_5_radians_to_degrees_ != 0.0) ? arm_joint_5_radians_to_degrees_ : 1.0,
        (arm_joint_6_radians_to_degrees_ != 0.0) ? arm_joint_6_radians_to_degrees_ : 1.0
    };
    
    for (size_t i = 0; i < num_arm_joints; ++i)
    {
        // Check if this joint's command is significant enough to send
        double current_cmd = position_commands_[i];
        double prev_cmd = prev_position_commands_[i];
        
        
        // Apply calibration conversion (radians to degrees)
        double angle_degrees = current_cmd * calibration_factors[i];
        
        // Format with one decimal place
        float rounded_angle = std::round(static_cast<float>(angle_degrees) * 100.0f) / 100.0f;

        if (std::fabs(rounded_angle) < std::numeric_limits<float>::epsilon())
        {
            rounded_angle = 0.0f; 
        }

        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << rounded_angle;
        float formatted_angle = std::stof(stream.str());

        arm_msg.data.push_back(formatted_angle);
     
    }

    // Publish the arm command
    if (num_arm_joints > 0)
    {
        robot_command_publisher_->publish(arm_msg);
    }

    // Send gripper commands
    if (position_commands_.size() > 6)
    {
        // Note: Assuming only one gripper joint at index 6
        std_msgs::msg::Float32 gripper_msg;
        double gripper_calibration = (gripper_radians_to_degrees_ != 0.0) ? 
                                     gripper_radians_to_degrees_ : 1.0;
        
        // Check if gripper changed significantly
        double gripper_cmd = position_commands_[6];
        double prev_gripper_cmd = (prev_position_commands_.size() > 6) ? prev_position_commands_[6] : 0.0;
        
        if (std::abs(gripper_cmd - prev_gripper_cmd) <= SIGNIFICANT_CHANGE_THRESHOLD)
        {
            gripper_cmd = prev_gripper_cmd;
        }
        
        gripper_msg.data = static_cast<float>(gripper_cmd * gripper_calibration);

        gripper_command_publisher_->publish(gripper_msg);
        
    }

    prev_position_commands_ = position_commands_;

    return hardware_interface::return_type::OK;
}


void Mycobot320piInterface::query_callback_arm(const std_msgs::msg::Float32MultiArray::ConstSharedPtr &msg)
{
    if (msg->data.size() < 6)
    {
        RCLCPP_ERROR(node_->get_logger(), 
            "Received arm message with insufficient data size: %zu (expected >=6)", 
            msg->data.size());
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Array of calibration factors for each joint
    std::array<double, 6> calibration_factors = {
        (arm_joint_1_degrees_to_radians_ != 0.0) ? arm_joint_1_degrees_to_radians_ : 1.0,
        (arm_joint_2_degrees_to_radians_ != 0.0) ? arm_joint_2_degrees_to_radians_ : 1.0,
        (arm_joint_3_degrees_to_radians_ != 0.0) ? arm_joint_3_degrees_to_radians_ : 1.0,
        (arm_joint_4_degrees_to_radians_ != 0.0) ? arm_joint_4_degrees_to_radians_ : 1.0,
        (arm_joint_5_degrees_to_radians_ != 0.0) ? arm_joint_5_degrees_to_radians_ : 1.0,
        (arm_joint_6_degrees_to_radians_ != 0.0) ? arm_joint_6_degrees_to_radians_ : 1.0
    };
    
    // Update arm joint states (first 6 joints) and apply calibration
    for (size_t i = 0; i < 6 && i < position_states_.size(); ++i)
    {
        double degrees = static_cast<double>(msg->data.at(i));
        position_states_[i] = degrees * calibration_factors[i];
        
    }
    
}


void Mycobot320piInterface::query_callback_gripper(
  const std_msgs::msg::Float32::ConstSharedPtr &msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Update gripper state (joint index 6)
    if (position_states_.size() > 6)
    {
        double degrees = static_cast<double>(msg->data);
        double gripper_calibration = (gripper_degrees_to_radians_ != 0.0) ? 
                                     gripper_degrees_to_radians_ : 0.01745329252;
        position_states_[6] = degrees * gripper_calibration;

        RCLCPP_DEBUG(node_->get_logger(),
            "Updated gripper state: %.3f deg -> %.3f rad (calibration factor: %.6f)",
            degrees,
            position_states_[6],
            gripper_calibration
        );
    }
}

void Mycobot320piInterface::CalibrationArm(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!msg) {
        RCLCPP_WARN(logger_, "CalibrationArm received null msg");
        return;
    }

    if (msg->data.size() < 14) {
        RCLCPP_WARN(logger_, "CalibrationArm data size %zu < 14 (expected 14 parameters)", 
                   msg->data.size());
        return;
    }

    std::vector<double> new_arm_calibs {
        static_cast<double>(msg->data[0]),
        static_cast<double>(msg->data[1]),
        static_cast<double>(msg->data[2]),
        static_cast<double>(msg->data[3]),
        static_cast<double>(msg->data[4]),
        static_cast<double>(msg->data[5]),
        static_cast<double>(msg->data[6]),
        static_cast<double>(msg->data[7]),
        static_cast<double>(msg->data[8]),
        static_cast<double>(msg->data[9]),
        static_cast<double>(msg->data[10]),
        static_cast<double>(msg->data[11]),
        static_cast<double>(msg->data[12]),
        static_cast<double>(msg->data[13])
    };

    if (new_arm_calibs == prev_arm_calibrations_)
        return;

    // Update calibration parameters
    arm_joint_1_radians_to_degrees_    = new_arm_calibs[0];
    arm_joint_2_radians_to_degrees_    = new_arm_calibs[1];
    arm_joint_3_radians_to_degrees_    = new_arm_calibs[2];
    arm_joint_4_radians_to_degrees_    = new_arm_calibs[3];
    arm_joint_5_radians_to_degrees_    = new_arm_calibs[4];
    arm_joint_6_radians_to_degrees_    = new_arm_calibs[5];

    arm_joint_1_degrees_to_radians_    = new_arm_calibs[6];
    arm_joint_2_degrees_to_radians_    = new_arm_calibs[7];
    arm_joint_3_degrees_to_radians_    = new_arm_calibs[8];
    arm_joint_4_degrees_to_radians_    = new_arm_calibs[9];
    arm_joint_5_degrees_to_radians_    = new_arm_calibs[10];
    arm_joint_6_degrees_to_radians_    = new_arm_calibs[11];

    gripper_radians_to_degrees_    = new_arm_calibs[12];
    gripper_degrees_to_radians_    = new_arm_calibs[13];
    

    prev_arm_calibrations_ = new_arm_calibs;
}

}  // namespace mycobot_320pi_controller

PLUGINLIB_EXPORT_CLASS(mycobot_320pi_controller::Mycobot320piInterface, hardware_interface::SystemInterface)