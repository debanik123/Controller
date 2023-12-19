#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <modbus/modbus.h>
#include <cmath>
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class kint_control : public rclcpp::Node
{
  public:
    kint_control()
    : Node("kint_control_node")
    {
      declare_parameter("timer_period_s", 1);
      auto timer_period_s = std::chrono::seconds(get_parameter("timer_period_s").as_int());
      CmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&kint_control::CmdVelCb, this, _1));
      PLCPublisher = this->create_publisher<std_msgs::msg::Int16MultiArray>("plc_data", 10);

      start_followme_loop_client = create_client<std_srvs::srv::Trigger>("start_followme_loop");
      stop_followme_loop_client = create_client<std_srvs::srv::Trigger>("stop_followme_loop");

      timer_ = create_wall_timer(timer_period_s, std::bind(&kint_control::timer_callback, this));

      ctx_plc = modbus_new_rtu("/dev/ttyUSB0", 115200, 'N', 8, 1);
      modbus_connect(ctx_plc);

    }
  public:
    ~kint_control();
  public:
    void CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    double pwm_to_analog(double pwm_value, double max_pwm_value, double max_analog_value);
    void plc_modbus(double left_plc, double right_plc);
    double wrapping(float v);
    float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr CmdVelSub;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr PLCPublisher;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_followme_loop_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_followme_loop_client;

    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    // double K=300;  // twist factor
    // double r = 0.207; // wheel radius 
    // double K = 60.0/(2 * 3.141592 * wheel_radius);
    // double w = 0.9; // distance between center of two wheel

    // Wheelbase and wheel radius parameters
    double wheelbase = 0.9;  // Replace with your robot's wheelbase in meters
    double wheel_radius = 0.207;  // Replace with your robot's wheel radius in meters

    double left_pwm, right_pwm, left_plc, right_plc, dx, dy, dr;
    modbus_t *ctx = NULL;

    const double min_rpm_threshold = 0.0;
    double Sqrt(double x, double y);

    const double diff_lr_plc_threshold =8.0;
    double linear_x, angular_z;
    modbus_t *ctx_plc = NULL;
    int status1;

    
    
    
};
void kint_control::timer_callback()
{
  // if (!start_followme_loop_client->wait_for_service(1s) || !stop_followme_loop_client->wait_for_service(1s))
  // {
  //     RCLCPP_ERROR(get_logger(), "Failed to connect to the image save service");
  //     return;
  // }

  static uint8_t prev_toggle = 0;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // uint8_t toggle[1];
  uint16_t tab_reg[64];
  // modbus_read_bits(ctx_plc, 1025, 1, toggle);
  modbus_set_slave(ctx_plc, 1);
  int rc = modbus_read_registers(ctx_plc, 4098, 1, tab_reg);
  std::cout <<"\n "<< "Reg Count " <<rc << std::endl;
  // for (int i=0; i < rc; i++) 
  // {
  std::cout<<"reg data --> "<<tab_reg[0]<<std::endl;
  // }



  // if(toggle[0] == 1 && prev_toggle == 0)
  // {
  //   auto future1 = start_followme_loop_client->async_send_request(request);
  // }
  // if(toggle[0] == 0 && prev_toggle == 1)
  // {
  //   auto future2 = stop_followme_loop_client->async_send_request(request);
  // }

  // prev_toggle = toggle[0];

}
double kint_control::Sqrt(double x, double y)
{
  double h = hypot(x, y);
  return h;
}

double kint_control::pwm_to_analog(double pwm_value, double max_pwm_value, double max_analog_value) 
{
  if (pwm_value <= 61.0) 
  {
    pwm_value = 61.0;
  }
  return (pwm_value / max_pwm_value) * max_analog_value;
}

float kint_control::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void kint_control::plc_modbus(double left_plc, double right_plc)
{ 
    uint16_t motor_write_reg[1] = {};
    // int rc; 
    
    modbus_set_slave(ctx_plc, 1);

    double diff_lr_plc = left_plc - right_plc;
    RCLCPP_INFO(this->get_logger(), "diff_lr_plc: %f", diff_lr_plc);
    if (diff_lr_plc > diff_lr_plc_threshold) 
    {
      modbus_write_bit(ctx_plc, 2048, 0);
      modbus_write_bit(ctx_plc, 2049, 1);
      modbus_write_bit(ctx_plc, 2050, 0);
      modbus_write_bit(ctx_plc, 2051, 1);
      RCLCPP_INFO(this->get_logger(), "Turn Right");
      right_plc = right_plc;
      left_plc *= 1.35;
      if(left_plc>875)
      {
        left_plc = 875;
      }
    }
    else if (diff_lr_plc < -diff_lr_plc_threshold) 
    {
      modbus_write_bit(ctx_plc, 2048, 0);
      modbus_write_bit(ctx_plc, 2049, 1);
      modbus_write_bit(ctx_plc, 2050, 0);
      modbus_write_bit(ctx_plc, 2051, 1);
      RCLCPP_INFO(this->get_logger(), "Turn Left");
      right_plc *= 1.35;
      left_plc = left_plc;
      if(right_plc>875)
      {
        right_plc = 875;
      }
    }

    else if(linear_x <= 0.1)
    {
      modbus_write_bit(ctx_plc, 2048, 1);
      modbus_write_bit(ctx_plc, 2049, 0);
      modbus_write_bit(ctx_plc, 2050, 1);
      modbus_write_bit(ctx_plc, 2051, 0);
    }
    else
    {
      
      modbus_write_bit(ctx_plc, 2048, 0);
      modbus_write_bit(ctx_plc, 2049, 1);
      modbus_write_bit(ctx_plc, 2050, 0);
      modbus_write_bit(ctx_plc, 2051, 1);

      RCLCPP_INFO(this->get_logger(), "Moving straight");
      right_plc = right_plc*1.35;
      left_plc = left_plc*1.35;
      if(left_plc>875)
      {
        left_plc = 875;
      }
      if(right_plc>875)
      {
        right_plc = 875;
      }

    }

    
    motor_write_reg[0] = right_plc;
    motor_write_reg[1] = left_plc;
    int rc = modbus_write_registers(ctx_plc, 4096, 2, motor_write_reg);

    // modbus_close(ctx_plc);
    // modbus_free(ctx_plc);

    if (rc == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write data Plc for motor %s");
    }
    
    
}

void kint_control::CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    linear_x = msg->linear.x;
    angular_z = msg->angular.z;

    if(linear_x == 0.0 && angular_z == 0.0)
    {
      left_plc = 0.0;
      right_plc = 0.0;
    }

    else
    {
      // Calculate left and right wheel velocities (in m/s)
      double left_wheel_vel = linear_x - (angular_z * wheelbase / 2.0);
      double right_wheel_vel = linear_x + (angular_z * wheelbase / 2.0);

      // Convert wheel velocities to RPM (assuming linear relationship)
      int left_motor_rpm = static_cast<int>(left_wheel_vel / (2 * 3.141592 * wheel_radius) * 60);
      int right_motor_rpm = static_cast<int>(right_wheel_vel / (2 * 3.141592 * wheel_radius) * 60);

      left_plc = mapFloat(left_motor_rpm, -255, 255, 220, 880);
      right_plc = mapFloat(right_motor_rpm, -255, 255, 220, 880);

      


      RCLCPP_INFO(this->get_logger(), "Left Motor RPM: %d, Right Motor RPM: %d", left_motor_rpm, right_motor_rpm);
      RCLCPP_INFO(this->get_logger(), "Left Motor PLC: %f, Right Motor PLC: %f", left_plc, right_plc);
      // left_plc = pwm_to_analog(left_motor_rpm, 255, 880);  // plc mx analog value 880
      // right_plc = pwm_to_analog(right_motor_rpm, 255, 880); // plc mx analog value 880
      // std::cout<<"left_pwm --- > "<<left_pwm<<" right_pwm ----> "<<right_pwm<<std::endl;

    }
    

    
    plc_modbus(left_plc, right_plc);

    std_msgs::msg::Int16MultiArray message;
    message.data = {left_pwm, right_pwm, left_plc, right_plc};

    PLCPublisher->publish(message);

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
  modbus_close(ctx_plc);
  modbus_free(ctx_plc);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kint_control>());
  rclcpp::shutdown();
  return 0;
}