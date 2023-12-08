import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist
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

        self.left_pwm = 0.0
        self.right_pwm = 0.0
        self.left_plc = 0.0
        self.right_plc = 0.0


        # Modbus connection parameters
        self.modbus_port = '/dev/ttyUSB0'
        self.modbus_baudrate = 115200
        self.modbus_stopbits = 1
        self.modbus_bytesize = 8
        self.modbus_parity = 'N'
        self.modbus_slave_address = 1
        self.modbus_client = ModbusSerialClient(
            method='rtu',
            port=self.modbus_port,
            baudrate=self.modbus_baudrate,
            stopbits=self.modbus_stopbits,
            bytesize=self.modbus_bytesize,
            parity=self.modbus_parity
        )

        
        self.diff_lr_plc_threshold = 7.0

    def destroy(self):
        self.modbus_client.close()
        super().destroy_node()

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        print("fuckkkkkkkkkkkkkkkkkkkkkkkkkkkk")
        if linear_x == 0.0 and angular_z == 0.0:
            self.left_plc = 0.0
            self.right_plc = 0.0
        else:
            left_wheel_vel = linear_x - (angular_z * self.wheelbase / 2.0)
            right_wheel_vel = linear_x + (angular_z * self.wheelbase / 2.0)

            left_motor_rpm = int(left_wheel_vel / (2 * math.pi * self.wheel_radius) * 60)
            right_motor_rpm = int(right_wheel_vel / (2 * math.pi * self.wheel_radius) * 60)

            self.left_plc = self.map_float(left_motor_rpm, -255, 255, 220, 880)
            self.right_plc = self.map_float(right_motor_rpm, -255, 255, 220, 880)

            diff_lr_plc = self.left_plc - self.right_plc

            if diff_lr_plc > self.diff_lr_plc_threshold:
                self.right_plc = self.right_plc
                self.left_plc *= 1.35
                if self.left_plc > 875:
                    self.left_plc = 875
            elif diff_lr_plc < -self.diff_lr_plc_threshold:
                self.right_plc *= 1.35
                self.left_plc = self.left_plc
                if self.right_plc > 875:
                    self.right_plc = 875
            else:
                self.right_plc = self.right_plc * 1.5
                self.left_plc = self.left_plc * 1.5
                if self.left_plc > 875:
                    self.left_plc = 875
                if self.right_plc > 875:
                    self.right_plc = 875

            self.get_logger().info(f'Left Motor RPM: {left_motor_rpm}, Right Motor RPM: {right_motor_rpm}')
            self.get_logger().info(f'Left Motor PLC: {self.left_plc}, Right Motor PLC: {self.right_plc}')


        self.plc_modbus(self.left_plc, self.right_plc)

        message = Int16MultiArray()
        message.data = [self.left_pwm, self.right_pwm, self.left_plc, self.right_plc]
        self.plc_publisher.publish(message)

    def plc_modbus(self, left_plc, right_plc):
        if not self.modbus_client.connect():
            self.get_logger().error('PLC Connection failed')
            return

        try:
            self.modbus_client.set_slave(self.modbus_slave_address)
            motor_write_reg = [int(right_plc), int(left_plc)]
            self.modbus_client.write_registers(4096, motor_write_reg)
        except Exception as e:
            self.get_logger().error('Failed to write data to PLC: %s', str(e))
        finally:
            self.modbus_client.close()

    @staticmethod
    def map_float(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main(args=None):
    rclpy.init(args=args)
    kint_control = KintControl()
    try:
        rclpy.spin(kint_control)
    except KeyboardInterrupt:
        pass

    kint_control.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
