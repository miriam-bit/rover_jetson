#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <pix_rover_driver_msgs/msg/reference.hpp>
#include <cmath>
#include <cstdint>
#include <algorithm>

enum SmoothedAnalog_StatusTypeDef {
    SA_OK,
    SA_ERROR
};

struct Cartesian2D {
    float x;
    float y;
};

static inline float clamp(float val, float min_val, float max_val) {
    return std::min(std::max(val, min_val), max_val);
}

static inline SmoothedAnalog_StatusTypeDef driveMotors( const Cartesian2D& analog, float max_output_forward,float max_output_backward, int16_t& l_wheel, int16_t& r_wheel) 
{
    static const float tolerance = 0.1f;
    static const float max_input_forward = 1.0f;  
    static const float max_input_backward = 1.0f;  
    static const float max_turn_reg = 0.5f;
    float x = analog.x;
    float y = analog.y;
    float temp_A = 0.0f;
    float temp_B = 0.0f;

    if ((y > 0.0f) && (x >= 0.0f)) {
        temp_A = y / max_input_forward * max_output_forward;
        temp_B = temp_A - (x / max_input_forward * max_output_forward * max_turn_reg);
    } else if ((y > 0.0f) && (x < 0.0f)) {
        temp_A = (y / max_input_forward * max_output_forward) +
                 (x / max_input_backward * max_output_forward * max_turn_reg);
        temp_B = y / max_input_forward * max_output_forward;
    } else if ((y < 0.0f) && (x >= 0.0f)) {
        temp_A = y / max_input_backward * max_output_backward;
        temp_B = temp_A + (x / max_input_forward * max_output_backward * max_turn_reg);
    } else if ((y < 0.0f) && (x < 0.0f)) {
        temp_A = (y / max_input_backward * max_output_backward) -
                 (x / max_input_backward * max_output_backward * max_turn_reg);
        temp_B = y / max_input_backward * max_output_backward;
    } else if ((std::fabs(y) < tolerance) && (std::fabs(x) >= tolerance)) {
        temp_A = x / max_input_forward * max_output_forward;
        temp_B = -temp_A;
    } else {
        temp_A = 0.0f;
        temp_B = 0.0f;
    }

    temp_A = clamp(std::round(temp_A), -max_output_backward, max_output_backward);
    temp_B = clamp(std::round(temp_B), -max_output_backward, max_output_backward);

    l_wheel = static_cast<int16_t>(temp_B);
    r_wheel = static_cast<int16_t>(temp_A);

    return SA_OK;
}



class DriveNode : public rclcpp::Node {
public:
    DriveNode() : Node("reference_node") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&DriveNode::joyCallback, this, std::placeholders::_1));

        reference_pub_ = this->create_publisher<pix_rover_driver_msgs::msg::Reference>("/pix_rover/reference", 10);

        RCLCPP_INFO(this->get_logger(), "ReferenceNode started with ros2_socketcan publishing on /pix_rover/reference");
    }

private:

    /* -------------------------------------------------------------------------- *
    *  deadzone_symmetric_internal                                               *
    * -------------------------------------------------------------------------- */
    [[nodiscard]] float deadzone_symmetric_internal(float raw_value,
                                                    float deadzone) noexcept
    {

    static const float AXIS_LIMIT_POS_F32 =  1.0F;
    float adjusted {raw_value};
    
    if (deadzone > 0.0F)
    {
        if (std::fabs(raw_value) <= deadzone)
        {
            adjusted = 0.0F;
        }
        else
        {
            const float sign      = (raw_value < 0.0F) ? -1.0F : 1.0F;
            const float magnitude = (std::fabs(raw_value) - deadzone) /
                                    (AXIS_LIMIT_POS_F32 - deadzone);
            adjusted             = sign * magnitude;
        }
        
    }
    
        return adjusted;
    }
  
    float deadzone_symmetric(float raw_value, float deadzone) noexcept
    {
        return deadzone_symmetric_internal(raw_value, deadzone);
    }
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        static const float deadzone_x = 0.05;
        static const float deadzone_y = 0.05;
 
        if (msg->axes.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Joy message has insufficient axes");
            return;
        }

        Cartesian2D input{deadzone_symmetric(msg->axes[3], deadzone_x), deadzone_symmetric(msg->axes[1], deadzone_y )};
        int16_t l_motor = 0, r_motor = 0;

        if (driveMotors(input, 167.0f, 167.0f, l_motor, r_motor) == SA_OK) {
            RCLCPP_INFO(this->get_logger(), "Motori -> L: %d, R: %d", l_motor, r_motor);

            // Convert int8_t to uint16_t with offset
            int16_t value_left  = static_cast<uint16_t>(static_cast<int16_t>(l_motor)) ;
            int16_t value_right = static_cast<uint16_t>(static_cast<int16_t>(r_motor)) ;

            pix_rover_driver_msgs::msg::Reference reference_msg;
            reference_msg.header.stamp = this->get_clock()->now();
            reference_msg.header.frame_id = "base_link";     
            reference_msg.front_left = value_left;
            reference_msg.rear_left = value_left;
            reference_msg.front_right= value_right;
            reference_msg.rear_right = value_right;


            reference_pub_->publish(reference_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<pix_rover_driver_msgs::msg::Reference>::SharedPtr reference_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveNode>());
    rclcpp::shutdown();
    return 0;
}
