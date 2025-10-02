#include <pix_rover_driver/report_parser.hpp>

namespace pix_rover_driver
{
namespace report_parser
{
ReportParser::ReportParser() : Node("report_parser")
{
  // ros params
  param_.base_frame_id = declare_parameter("base_frame_id", "base_link");
  param_.report_timeout_ms = declare_parameter("report_timeout_ms", 1000);
  param_.loop_rate = declare_parameter("loop_rate", 50.0);

  // // initialize msg received time
  /** example
  brake_command_received_time_ = this->now();
  **/
  imu_gyro_xy_received_time_ = this->now();
imu_accel_xy_received_time_ = this->now();
imu_z_received_time_ = this->now();
lin_vel_xy_received_time_ = this->now();


  is_publish_ = true;

  using std::placeholders::_1;

  /* subscriber */
  {
    // from pix driver autoware interface
    can_frame_sub_ = create_subscription<can_msgs::msg::Frame>(
      "input/can_rx", 1, std::bind(&ReportParser::callbackCan, this, _1));
    // is publish
    is_publish_sub_ = create_subscription<std_msgs::msg::Bool>(
      "input/is_publish", 1, std::bind(&ReportParser::callbackIsPublish, this, _1));
  }

  /* publisher */
  {
    /** example
    brake_sta_fb_pub_ =
      create_publisher<V2aBrakeStaFb>("/pix_hooke/v2a_brakestafb", rclcpp::QoS{1});
    **/
    imu_gyro_xy_pub_ = create_publisher<pix_rover_driver_msgs::msg::ImuGyroXy>("/pix_rover/imu_gyro_xy", rclcpp::QoS{1});
imu_accel_xy_pub_ = create_publisher<pix_rover_driver_msgs::msg::ImuAccelXy>("/pix_rover/imu_accel_xy", rclcpp::QoS{1});
imu_z_pub_ = create_publisher<pix_rover_driver_msgs::msg::ImuZ>("/pix_rover/imu_z", rclcpp::QoS{1});
lin_vel_xy_pub_ = create_publisher<pix_rover_driver_msgs::msg::LinVelXy>("/pix_rover/lin_vel_xy", rclcpp::QoS{1});
 
  }
  {
    // timer
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Rate(param_.loop_rate).period(),
      std::bind(&ReportParser::timerCallback, this));
  }
}

// calback is publish
void ReportParser::callbackIsPublish(const std_msgs::msg::Bool::ConstSharedPtr & msg)
{
  is_publish_ = msg->data;
}

// callback can
void ReportParser::callbackCan(const can_msgs::msg::Frame::ConstSharedPtr & msg)
{
  std_msgs::msg::Header header;
  header.frame_id = param_.base_frame_id;
  header.stamp = msg->header.stamp;

  /** example
  V2aBrakeStaFb brake_sta_fb_msg;
  **/
  pix_rover_driver_msgs::msg::ImuGyroXy imu_gyro_xy_msg;
pix_rover_driver_msgs::msg::ImuAccelXy imu_accel_xy_msg;
pix_rover_driver_msgs::msg::ImuZ imu_z_msg;
pix_rover_driver_msgs::msg::LinVelXy lin_vel_xy_msg;


  uint8_t byte_temp[8];
  switch (msg->id)
  {
  /** example
  case V2adrivestafb530::ID:
    drive_sta_fb_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    v2a_drivestafb_530_entity_.update_bytes(byte_temp);
    v2a_drivestafb_530_entity_.Parse();

    drive_sta_fb_msg.header = header;
    drive_sta_fb_msg.vcu_chassis_driver_en_sta =
      v2a_drivestafb_530_entity_.vcu_chassisdriverensta;
    drive_sta_fb_msg.vcu_chassis_diver_slopover =
      v2a_drivestafb_530_entity_.vcu_chassisdiverslopover;
    drive_sta_fb_msg.vcu_chassis_driver_mode_sta =
      v2a_drivestafb_530_entity_.vcu_chassisdrivermodesta;
    drive_sta_fb_msg.vcu_chassis_gear_fb = v2a_drivestafb_530_entity_.vcu_chassisgearfb;
    drive_sta_fb_msg.vcu_chassis_speed_fb = v2a_drivestafb_530_entity_.vcu_chassisspeedfb;
    drive_sta_fb_msg.vcu_chassis_throttle_padl_fb =
      v2a_drivestafb_530_entity_.vcu_chassisthrottlepaldfb;
    drive_sta_fb_msg.vcu_chassis_accceleration_fb =
      v2a_drivestafb_530_entity_.vcu_chassisaccelerationfb;
    drive_sta_fb_ptr_ = std::make_shared<V2aDriveStaFb>(drive_sta_fb_msg);
    break;
  **/
  

    case ImuGyroXy::ID:
    imu_gyro_xy_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    imu_gyro_xy_entity_.update_bytes(byte_temp);
    imu_gyro_xy_entity_.Parse();

    imu_gyro_xy_msg.header = header;
    imu_gyro_xy_msg.gyro_x = imu_gyro_xy_entity_.gyro_x_;
imu_gyro_xy_msg.gyro_y = imu_gyro_xy_entity_.gyro_y_;

    imu_gyro_xy_ptr_ = std::make_shared<pix_rover_driver_msgs::msg::ImuGyroXy>(imu_gyro_xy_msg);
    break;
    

    case ImuAccelXy::ID:
    imu_accel_xy_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    imu_accel_xy_entity_.update_bytes(byte_temp);
    imu_accel_xy_entity_.Parse();

    imu_accel_xy_msg.header = header;
    imu_accel_xy_msg.accel_x = imu_accel_xy_entity_.accel_x_;
imu_accel_xy_msg.accel_y = imu_accel_xy_entity_.accel_y_;

    imu_accel_xy_ptr_ = std::make_shared<pix_rover_driver_msgs::msg::ImuAccelXy>(imu_accel_xy_msg);
    break;
    

    case ImuZ::ID:
    imu_z_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    imu_z_entity_.update_bytes(byte_temp);
    imu_z_entity_.Parse();

    imu_z_msg.header = header;
    imu_z_msg.gyro_z = imu_z_entity_.gyro_z_;
imu_z_msg.accel_z = imu_z_entity_.accel_z_;

    imu_z_ptr_ = std::make_shared<pix_rover_driver_msgs::msg::ImuZ>(imu_z_msg);
    break;
    

    case LinVelXy::ID:
    lin_vel_xy_received_time_ = this->now();
    
    for(uint i=0;i<8;i++)
    {
    byte_temp[i] = msg->data[i];
    }
    lin_vel_xy_entity_.update_bytes(byte_temp);
    lin_vel_xy_entity_.Parse();

    lin_vel_xy_msg.header = header;
    lin_vel_xy_msg.lin_vel_x = lin_vel_xy_entity_.lin_vel_x_;
lin_vel_xy_msg.lin_vel_y = lin_vel_xy_entity_.lin_vel_y_;

    lin_vel_xy_ptr_ = std::make_shared<pix_rover_driver_msgs::msg::LinVelXy>(lin_vel_xy_msg);
    break;
    

  default:
    break;
  }
}

void ReportParser::timerCallback()
{
  if (!is_publish_) return;

  const rclcpp::Time current_time = this->now();
  
  /** example
  // drive sta fb report
  const double drive_sta_fb_report_delta_time_ms =
    (current_time - drive_sta_fb_received_time_).seconds() * 1000.0;
  if(drive_sta_fb_report_delta_time_ms>param_.report_timeout_ms || drive_sta_fb_ptr_==nullptr)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "drive stat fb report timeout = %f ms.", drive_sta_fb_report_delta_time_ms);
  }else{
    drive_sta_fb_pub_->publish(*drive_sta_fb_ptr_);
  }
  **/
  
    const double imu_gyro_xy_report_delta_time_ms =
    (current_time - imu_gyro_xy_received_time_).seconds() * 1000.0;
    if(imu_gyro_xy_report_delta_time_ms>param_.report_timeout_ms || imu_gyro_xy_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "imu_gyro_xy report timeout = %f ms.", imu_gyro_xy_report_delta_time_ms);
    }else{
        imu_gyro_xy_pub_->publish(*imu_gyro_xy_ptr_);
    }
    
    const double imu_accel_xy_report_delta_time_ms =
    (current_time - imu_accel_xy_received_time_).seconds() * 1000.0;
    if(imu_accel_xy_report_delta_time_ms>param_.report_timeout_ms || imu_accel_xy_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "imu_accel_xy report timeout = %f ms.", imu_accel_xy_report_delta_time_ms);
    }else{
        imu_accel_xy_pub_->publish(*imu_accel_xy_ptr_);
    }
    
    const double imu_z_report_delta_time_ms =
    (current_time - imu_z_received_time_).seconds() * 1000.0;
    if(imu_z_report_delta_time_ms>param_.report_timeout_ms || imu_z_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "imu_z report timeout = %f ms.", imu_z_report_delta_time_ms);
    }else{
        imu_z_pub_->publish(*imu_z_ptr_);
    }
    
    const double lin_vel_xy_report_delta_time_ms =
    (current_time - lin_vel_xy_received_time_).seconds() * 1000.0;
    if(lin_vel_xy_report_delta_time_ms>param_.report_timeout_ms || lin_vel_xy_ptr_==nullptr)
    {
        RCLCPP_ERROR_THROTTLE(
        get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "lin_vel_xy report timeout = %f ms.", lin_vel_xy_report_delta_time_ms);
    }else{
        lin_vel_xy_pub_->publish(*lin_vel_xy_ptr_);
    }
    
  
}

} // namespace report_parser
} // namespace pix_rover_driver
