# pip3 install pymodbus==2.2.0rc1
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from pymodbus.client.sync import ModbusSerialClient
import math


class KintControl(Node):

    def __init__(self):
        super().__init__('kint_control_node')
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.plc_publisher = self.create_publisher(
            Int16MultiArray, 'plc_data', 10)

        # Wheelbase and wheel radius parameters
        self.wheelbase = 0.9  # Replace with your robot's wheelbase in meters
        self.wheel_radius = 0.207  # Replace with your robot's wheel radius in meters

        self.left_pwm = 0
        self.right_pwm = 0
        self.left_plc = 0
        self.right_plc = 0

        self.ctx_plc = ModbusSerialClient(method='ascii', timeout=0.2, port='/dev/ttyUSB0')
        if self.ctx_plc.connect():
            print("PLC Connection OKKKKk")
        if not self.ctx_plc.connect():
            # self.get_logger().error("PLC Connection failed")
            print("PLC Connection failed")
            self.ctx_plc.close()
            return

    def pwm_to_analog(self, pwm_value, max_pwm_value, max_analog_value):
        if pwm_value <= 61.0:
            pwm_value = 61.0
        return (pwm_value / max_pwm_value) * max_analog_value

    def map_float(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def int_to_ascii(self,value):
        """Convert an integer to ASCII."""
        try:
            ascii_value = chr(value)
            return ascii_value
        except ValueError:
            print("Error: Cannot convert integer to ASCII.")
            return None

    def plc_modbus(self, left_plc, right_plc):

        try:
            # left_ascii = chr(int(left_plc))
            # right_ascii = chr(int(right_plc))

            # motor_write_reg = [ord(left_ascii), ord(right_ascii)]
            self.ctx_plc.write_registers(4096, motor_write_reg[0])
            print(motor_write_reg[0])
        except Exception as e:
            print("Failed to write data to PLC for motor")
        finally:
            self.ctx_plc.close()

    def rpm_to_voltage(self, rpm, max_rpm, min_voltage, max_voltage):
        normalized_rpm = min(max(0, rpm), max_rpm)  # Ensure RPM is within valid range
        normalized_voltage = ((normalized_rpm / max_rpm) * (max_voltage - min_voltage)) + min_voltage
        return normalized_voltage


    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate left and right wheel velocities (in m/s)
        left_wheel_vel = linear_x - (angular_z * self.wheelbase / 2.0)
        right_wheel_vel = linear_x + (angular_z * self.wheelbase / 2.0)

        # Convert wheel velocities to RPM (assuming linear relationship)
        left_motor_rpm = int(left_wheel_vel / (2 * math.pi * self.wheel_radius) * 60)
        right_motor_rpm = int(right_wheel_vel / (2 * math.pi * self.wheel_radius) * 60)

        max_rpm = 255
        # Set the voltage range (1 to 5 volts)
        min_voltage = 1
        max_voltage = 5

        left_motor_voltage = self.rpm_to_voltage(left_motor_rpm, max_rpm, min_voltage, max_voltage)
        right_motor_voltage = self.rpm_to_voltage(right_motor_rpm, max_rpm, min_voltage, max_voltage)

        print("left_wheel_vel ---> ", left_wheel_vel, "right_wheel_vel ---> ", right_wheel_vel)
        print("left_motor_rpm ---> ", left_motor_rpm, "right_motor_rpm ---> ", right_motor_rpm)
        print("left_motor_voltage ---> ", left_motor_voltage, "right_motor_voltage ---> ", right_motor_voltage)

        # Ensure RPM values are within the valid range (-255 to 255)
        left_motor_rpm = max(-255, min(255, left_motor_rpm))
        right_motor_rpm = max(-255, min(255, right_motor_rpm))

        # Print the calculated RPM values (replace with your motor control logic)
        # self.get_logger().info("Left Motor RPM: %d, Right Motor RPM: %d", left_motor_rpm, right_motor_rpm)

        self.left_plc = self.map_float(left_motor_rpm, -255, 255, 220, 880)
        self.right_plc = self.map_float(right_motor_rpm, -255, 255, 220, 880)

        # self.get_logger().info("Left Motor PLC: %f, Right Motor PLC: %f", self.left_plc, self.right_plc)

        # self.plc_modbus(self.left_plc, self.right_plc)

        # message = Int16MultiArray(data=[self.left_pwm, self.right_pwm, self.left_plc, self.right_plc])
        # self.plc_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    kint_control_node = KintControl()
    rclpy.spin(kint_control_node)
    kint_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
