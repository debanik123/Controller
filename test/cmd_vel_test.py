import rclpy
from geometry_msgs.msg import Twist
import math

def calculate_wheel_velocities(linear_x, angular_z, wheelbase, wheel_radius):
    left_wheel_vel = linear_x - (angular_z * wheelbase / 2.0)
    right_wheel_vel = linear_x + (angular_z * wheelbase / 2.0)

    # Convert wheel velocities to RPM (assuming linear relationship)
    left_motor_rpm = int(left_wheel_vel / (2 * math.pi * wheel_radius) * 60)
    right_motor_rpm = int(right_wheel_vel / (2 * math.pi * wheel_radius) * 60)

    return left_motor_rpm, right_motor_rpm, left_wheel_vel, right_wheel_vel

def map_to_range(value, from_min, from_max, to_min, to_max):
    # Map a value from one range to another
    return int((value - from_min) / (from_max - from_min) * (to_max - to_min) + to_min)

def cmd_vel_callback(msg):
    # Your callback function to process Twist messages
    linear_x = msg.linear.x
    angular_z = msg.angular.z
    print(f"Received cmd_vel message: Linear={linear_x}, Angular={angular_z}")

    # Assuming you have these parameters defined somewhere
    wheelbase = 0.9  # Replace with your actual value
    wheel_radius = 0.207  # Replace with your actual value

    left_motor_rpm, right_motor_rpm, left_wheel_vel, right_wheel_vel = calculate_wheel_velocities(linear_x, angular_z, wheelbase, wheel_radius)

    # Map the RPM values to the desired range (220 to 880)
    # mapped_left_motor_rpm = map_to_range(left_motor_rpm, 0, 255, 220, 880)
    # mapped_right_motor_rpm = map_to_range(right_motor_rpm, 0, 255, 220, 880)

    # Map the wheel velocities to a desired range (e.g., -1.0 to 1.0)
    mapped_left_wheel_vel = map_to_range(left_wheel_vel, 0.0, 1.0, 220, 440)
    mapped_right_wheel_vel = map_to_range(right_wheel_vel, 0.0, 1.0, 220, 440)

    # print(f"Original Left Motor RPM: {left_motor_rpm}, Mapped Left Motor RPM: {mapped_left_motor_rpm}")
    # print(f"Original Right Motor RPM: {right_motor_rpm}, Mapped Right Motor RPM: {mapped_right_motor_rpm}")
    print(f"Original Left Wheel Velocity: {left_wheel_vel}, Mapped Left Wheel Velocity: {mapped_left_wheel_vel}")
    print(f"Original Right Wheel Velocity: {right_wheel_vel}, Mapped Right Wheel Velocity: {mapped_right_wheel_vel}")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('cmd_vel_subscriber')
    cmd_vel_subscriber = node.create_subscription(Twist, 'cmd_vel', cmd_vel_callback, 10)
    print('Cmd_vel subscriber node has been initialized.')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Cmd_vel subscriber node stopped by the user.')
    finally:
        # Cleanup
        cmd_vel_subscriber.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
