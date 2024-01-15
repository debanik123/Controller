import rclpy
from geometry_msgs.msg import Twist

class DifferentialDriveController:
    def __init__(self):
        self.node = rclpy.create_node('differential_drive_controller')
        self.cmd_vel_subscription = self.node.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.wheelbase = 0.5  # Replace 0.5 with your actual wheelbase value

    def cmd_vel_callback(self, msg):
        # Assuming a differential drive robot with left and right wheel velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Use the provided formula to calculate left and right wheel velocities
        left_wheel_vel = linear_x - (angular_z * self.wheelbase / 2.0)
        right_wheel_vel = linear_x + (angular_z * self.wheelbase / 2.0)

        # Perform control logic or send wheel velocities to your robot

        # For example, print the wheel velocities
        print(f"Left wheel velocity: {left_wheel_vel}, Right wheel velocity: {right_wheel_vel}")

def main(args=None):
    rclpy.init(args=args)
    controller = DifferentialDriveController()
    rclpy.spin(controller.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
