#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"
#include <geometry_msgs/msg/twist.hpp>
// #include <modbus.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class kint_control : public rclcpp::Node
{
  public:
    kint_control()
    : Node("kint_control_node")
    {
        CmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&kint_control::CmdVelCb, this, _1));
        PLCPublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("plc_data", 10);
    }
  public:
    void CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    double pwm_to_analog(double pwm_value, double max_pwm_value, double max_analog_value);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr CmdVelSub;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr PLCPublisher;

    double K=163.3;  // twist factor
    double w=0.9; // distance between center of two wheel

    double left_pwm, right_pwm, left_plc, right_plc, dx, dy, dr;
};

double kint_control::pwm_to_analog(double pwm_value, double max_pwm_value, double max_analog_value) 
{
    return (pwm_value / max_pwm_value) * max_analog_value;
}

void kint_control::CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    dx = msg->linear.x;
    dy = msg->linear.y;
    dr = msg->angular.z;

    left_pwm = K*(( 1.0 * dx ) - (dr * w /2)); // pwm value 0 - 255
    right_pwm = -K*(( 1.0 * dx ) + (dr * w /2)); // pwm value 0 -255

    left_plc = pwm_to_analog(left_pwm, 255, 880);  // plc mx analog value 880
    right_plc = pwm_to_analog(right_pwm, 255, 880); // plc mx analog value 880

    // ctx_plc = modbus_new_rtu("/dev/ttyplc", 115200, 'N', 8, 1);



}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kint_control>());
  rclcpp::shutdown();
  return 0;
}