# pip3 install pymodbus==2.2.0rc1
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
# import modbus_tk.modbus_rtu as modbus_rtu
from pymodbus.client.sync import ModbusSerialClient
import math

class KintControl(Node):

    def __init__(self):
        super().__init__('kint_control_node')
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.plc_publisher = self.create_publisher(Int16MultiArray, 'plc_data', 10)

        # Wheelbase and wheel radius parameters
        self.wheelbase = 0.9  # Replace with your robot's wheelbase in meters
        self.wheel_radius = 0.207  # Replace with your robot's wheel radius in meters
        self.left_pwm = 0.0
        self.right_pwm = 0.0
        self.left_plc = 0.0
        self.right_plc = 0.0
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.diff_lr_plc_threshold = 8.0

        self.ctx_plc = ModbusSerialClient(method="rtu", port='/dev/ttyUSB0', baudrate=115200, stopbits=1, bytesize=8, parity='N')
        if self.ctx_plc.connect():
            print("PLC Connection OKKKKk")

            

        if not self.ctx_plc.connect():
            # self.get_logger().error("PLC Connection failed")
            print("PLC Connection failed")
            self.ctx_plc.close()
            return

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

        if self.linear_x == 0.0 and self.angular_z == 0.0:
            self.left_plc = 0.0
            self.right_plc = 0.0
        else:
            left_wheel_vel = self.linear_x - (self.angular_z * self.wheelbase / 2.0)
            right_wheel_vel = self.linear_x + (self.angular_z * self.wheelbase / 2.0)

            left_motor_rpm = int(left_wheel_vel / (2 * math.pi * self.wheel_radius) * 60)
            right_motor_rpm = int(right_wheel_vel / (2 * math.pi * self.wheel_radius) * 60)

            self.left_plc = self.map_float(left_motor_rpm, -255, 255, 220, 880)
            self.right_plc = self.map_float(right_motor_rpm, -255, 255, 220, 880)

            self.get_logger().info('Left Motor RPM: %d, Right Motor RPM: %d' % (left_motor_rpm, right_motor_rpm))
            self.get_logger().info('Left Motor PLC: %f, Right Motor PLC: %f' % (self.left_plc, self.right_plc))


        self.plc_modbus(self.left_plc, self.right_plc)


    def plc_modbus(self, left_plc, right_plc):
        try:
            motor_write_reg = [0, 0]
            motor_write_reg[0] = 450
            motor_write_reg[1] = 450
            self.ctx_plc.write_registers(4096, motor_write_reg)
            # print(rest.registers)
            print("done")
        
        except Exception as e:
            print("Failed to write data Plc for motor")

        # finally:
        #     self.ctx_plc.close()
    #     ctx_plc = None
    #     motor_write_reg = [0, 0]  # Assuming motor_write_reg is an array of 2 elements
    #     try:
    #         ctx_plc = ModbusRtu("/dev/ttyUSB0", 115200, 'N', 8, 1)
    #         ctx_plc.connect()

    #         diff_lr_plc = left_plc - right_plc
    #         self.get_logger().info("diff_lr_plc: %f", diff_lr_plc)

    #         if diff_lr_plc > self.diff_lr_plc_threshold:
    #             ctx_plc.write_bit(2048, False)
    #             ctx_plc.write_bit(2049, True)
    #             ctx_plc.write_bit(2050, False)
    #             ctx_plc.write_bit(2051, True)
    #             self.get_logger().info("Turn Right")
    #             right_plc = right_plc
    #             left_plc *= 1.35
    #             if left_plc > 875:
    #                 left_plc = 875
                    
    #         elif diff_lr_plc < -self.diff_lr_plc_threshold:
    #             ctx_plc.write_bit(2048, False)
    #             ctx_plc.write_bit(2049, True)
    #             ctx_plc.write_bit(2050, False)
    #             ctx_plc.write_bit(2051, True)
    #             self.get_logger().info("Turn Left")
    #             right_plc *= 1.35
    #             left_plc = left_plc
    #             if right_plc > 875:
    #                 right_plc = 875

    #         elif self.linear_x <= 0.1:
    #             ctx_plc.write_bit(2048, True)
    #             ctx_plc.write_bit(2049, False)
    #             ctx_plc.write_bit(2050, True)
    #             ctx_plc.write_bit(2051, False)
    #             self.get_logger().info("Moving reverse")

    #         else:
    #             ctx_plc.write_bit(2048, False)
    #             ctx_plc.write_bit(2049, True)
    #             ctx_plc.write_bit(2050, False)
    #             ctx_plc.write_bit(2051, True)
    #             self.get_logger().info("Moving straight")
    #             right_plc = right_plc * 1.35
    #             left_plc = left_plc * 1.35
    #             if left_plc > 875:
    #                 left_plc = 875
    #             if right_plc > 875:
    #                 right_plc = 875

    #         motor_write_reg[0] = right_plc
    #         motor_write_reg[1] = left_plc
    #         ctx_plc.write_registers(4096, motor_write_reg)

    #     except Exception as e:
    #         self.get_logger().error("Failed to write data Plc for motor %s", str(e))

    #     finally:
    #         if ctx_plc:
    #             ctx_plc.close()

    def map_float(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def main(args=None):
    rclpy.init(args=args)
    kint_control_node = KintControl()
    kint_control_node.ctx_plc.close()
    rclpy.spin(kint_control_node)
    kint_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
