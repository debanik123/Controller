#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <modbus/modbus.h>
#include <cmath>

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
    ~kint_control();
  public:
    void CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    double pwm_to_analog(double pwm_value, double max_pwm_value, double max_analog_value);
    void plc_modbus(double left_plc, double right_plc);
    double wrapping(float v);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr CmdVelSub;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr PLCPublisher;

    // double K=300;  // twist factor
    // double r = 0.207; // wheel radius 
    // double K = 60.0/(2 * 3.141592 * wheel_radius);
    // double w = 0.9; // distance between center of two wheel

    // Wheelbase and wheel radius parameters
    double wheelbase = 0.9;  // Replace with your robot's wheelbase in meters
    double wheel_radius = 0.207;  // Replace with your robot's wheel radius in meters

    double left_pwm, right_pwm, left_plc, right_plc, dx, dy, dr;
    modbus_t *ctx = NULL;
    
    
};

double kint_control::pwm_to_analog(double pwm_value, double max_pwm_value, double max_analog_value) 
{
  if (pwm_value <= 61.0) 
  {
    pwm_value = 61.0;
  }
  return (pwm_value / max_pwm_value) * max_analog_value;
}

void kint_control::plc_modbus(double left_plc, double right_plc)
{
    modbus_t *ctx_plc = NULL;
    uint16_t motor_write_reg[2] = {};

    int rc;

    ctx_plc = modbus_new_rtu("/dev/ttyplc", 115200, 'N', 8, 1);
    int status1 = modbus_connect(ctx_plc);

    if (status1 == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "PLC Connection failed");
        modbus_close(ctx_plc);
        modbus_free(ctx_plc);
        return;
    }

    else
    {
      motor_write_reg[0] = left_plc;
      motor_write_reg[1] = right_plc;

      rc = modbus_set_slave(ctx_plc, 1);
      rc = modbus_write_registers(ctx_plc, 4096, 2, motor_write_reg);

      if (rc == -1)
      {
          RCLCPP_ERROR(this->get_logger(), "Failed to write data Plc for motor %s", modbus_strerror(errno));
      }
      modbus_close(ctx_plc);
      modbus_free(ctx_plc);
    }
}

void kint_control::CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // Calculate left and right wheel velocities (in m/s)
    double left_wheel_vel = linear_x - (angular_z * wheelbase / 2.0);
    double right_wheel_vel = linear_x + (angular_z * wheelbase / 2.0);

    // Convert wheel velocities to RPM (assuming linear relationship)
    int left_motor_rpm = static_cast<int>(left_wheel_vel / (2 * 3.141592 * wheel_radius) * 60);
    int right_motor_rpm = static_cast<int>(right_wheel_vel / (2 * 3.141592 * wheel_radius) * 60);

    // Ensure RPM values are within the valid range (-255 to 255)
    left_motor_rpm = std::max(-255, std::min(255, left_motor_rpm));
    right_motor_rpm = std::max(-255, std::min(255, right_motor_rpm));

    // Print the calculated RPM values (replace with your motor control logic)
    RCLCPP_INFO(this->get_logger(), "Left Motor RPM: %d, Right Motor RPM: %d", left_motor_rpm, right_motor_rpm);

    left_plc = pwm_to_analog(left_motor_rpm, 255, 880);  // plc mx analog value 880
    right_plc = pwm_to_analog(right_motor_rpm, 255, 880); // plc mx analog value 880

    // std::cout<<"left_pwm --- > "<<left_pwm<<" right_pwm ----> "<<right_pwm<<std::endl;

    // plc_modbus(left_plc, right_plc);

    // std_msgs::msg::Int16MultiArray message;
    // message.data = {left_pwm, right_pwm, left_plc, right_plc};

    // PLCPublisher->publish(message);

}

double kint_control::wrapping(float v)
{
  if(std::abs(v) > 255.0)
  {
    v = 255.0;
  }
  return v;
}

kint_control::~kint_control()
{

    modbus_close(ctx);
    modbus_free(ctx);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kint_control>());
  rclcpp::shutdown();
  return 0;
}