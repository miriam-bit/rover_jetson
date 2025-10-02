#ifndef PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_
#define PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <can_msgs/msg/frame.hpp>


// include- msgs header file
// Example: #include "pix_driver_msgs/BrakeCommand.h"
// #include pix_driver_msgs/protocols["name"].h
#include <pix_rover_driver_msgs/msg/reference.hpp>



// include- Parse header file
// Example: #include "brake_command_101.hpp"
// #include protocols["name"].cpp
#include <pix_rover_driver/reference.hpp>


namespace pix_rover_driver
{
namespace control_command
{

/**
 * @brief param structure of control command node
 * @param base_frame_id frame id of vehicle
 * @param loop_rate loop rate of publishers in hz
 * @param command_timeout_ms timeout threshold of control command msg from control converter in ms
 */
struct Param
{
  std::string base_frame_id;
  double loop_rate;
  int command_timeout_ms;
};

class ControlCommand : public rclcpp::Node
{
private:
  // parameters of node
  Param param_;

  // subscribers
  // example rclcpp::Subscription<A2vBrakeCtrl>::SharedPtr a2v_brake_ctrl_sub_;
  rclcpp::Subscription<pix_rover_driver_msgs::msg::Reference>::SharedPtr reference_sub_;


  // msgs
  // example A2vBrakeCtrl::ConstSharedPtr brake_ctrl_ptr_;
  pix_rover_driver_msgs::msg::Reference::ConstSharedPtr reference_ptr_;


  // control command structures
  // example A2vdrivectrl130 a2v_drivectrl_130_entity_;
  Reference reference_entity_;


  // msg received timestamp
  // example rclcpp::Time drive_command_received_time_;
  rclcpp::Time reference_received_time_;


  // state control
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_ctrl_sub_;
  bool is_engage_;

  // publishers to can card driver
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;

  // publishing can msgs
  // example can_msgs::msg::Frame::ConstSharedPtr brake_ctrl_can_ptr_;
  can_msgs::msg::Frame::ConstSharedPtr reference_can_ptr_;


  // timer
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ControlCommand();
  // calback functions
  // example
  // void callbackBrakeCtrl(const A2vBrakeCtrl::ConstSharedPtr & msg);
  void callbackReference(const pix_rover_driver_msgs::msg::Reference::ConstSharedPtr & msg);


  void callbackEngage(const std_msgs::msg::Bool::ConstSharedPtr & msg);
  void timerCallback();

};

} // control_command
} // pix_rover_driver
#endif // PIX_HOOKE_DRIVER__CONTROL_COMMAND_HPP_