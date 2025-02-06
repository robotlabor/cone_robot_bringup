import rclpy
from rclpy.node import Node
from rclpy.client import Client
from geometry_msgs.msg import Twist
from mavros_msgs.msg import RCIn
from odrive_can.srv import AxisState

class RCToCmdVel(Node):
    def __init__(self):
        super().__init__('rc_to_cmd_vel_node')
        self.subscription = self.create_subscription(RCIn, '/mavros/rc/in', self.rc_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service clients for ODrive axes
        self.odrive_services = [
            self.create_client(AxisState, '/odrive_axis1/request_axis_state'),
            self.create_client(AxisState, '/odrive_axis2/request_axis_state'),
            self.create_client(AxisState, '/odrive_axis3/request_axis_state'),
            self.create_client(AxisState, '/odrive_axis4/request_axis_state')
        ]

        self.channel_mapping = {'linear': 1, 'angular': 0, 'button': 4}  # Map RC channels
        self.max_speed = 3.0  # Maximum linear velocity (m/s)
        self.max_angular = 2.0  # Maximum angular velocity (rad/s)

        self.button_previous_state = False  # Previous state of the button
        self.current_axis_state = 1  # Initial axis state

    def rc_callback(self, msg: RCIn):
        try:
            # Extract channels (adjust indices as needed)
            linear_pwm = msg.channels[self.channel_mapping['linear']]
            angular_pwm = msg.channels[self.channel_mapping['angular']]
            button_pwm = msg.channels[self.channel_mapping['button']]

            # Normalize PWM values (assumes 1000-2000 PWM range)
            linear_speed = self.pwm_to_velocity(linear_pwm, self.max_speed)
            angular_speed = self.pwm_to_velocity(angular_pwm, self.max_angular)

            # Publish Twist message
            twist = Twist()
            twist.linear.x = -linear_speed
            twist.angular.z = angular_speed
            self.publisher.publish(twist)

            # Handle button toggle for ODrive axis state
            button_state = self.is_button_pressed(button_pwm)
            if button_state and not self.button_previous_state:  # Button toggled
                self.toggle_odrive_axes()
            self.button_previous_state = button_state

        except IndexError:
            self.get_logger().warn("Received RC input with fewer channels than expected.")

    @staticmethod
    def pwm_to_velocity(pwm_value, max_value):
        # Normalize PWM (assuming 1500 is center, 1000-2000 is range)
        normalized = (pwm_value - 1500) / 500.0  # Scale -1 to 1
        return max_value * normalized

    @staticmethod
    def is_button_pressed(pwm_value):
        # Define thresholds for button pressed/released states
        return pwm_value > 1500  # Adjust threshold as needed

    def toggle_odrive_axes(self):
        # Toggle axis states between 1 and 8
        self.current_axis_state = 8 if self.current_axis_state == 1 else 1
        self.get_logger().info(f"Toggling ODrive axes to state: {self.current_axis_state}")

        # Send service requests for all axes
        for i, client in enumerate(self.odrive_services):
            if client.wait_for_service(timeout_sec=1.0):
                request = AxisState.Request()
                request.axis_requested_state = self.current_axis_state
                future = client.call_async(request)
                future.add_done_callback(lambda future: self.handle_service_response(future, i + 1))
            else:
                self.get_logger().error(f"Service for ODrive axis{i+1} not available")

    def handle_service_response(self, future, axis_id):
        try:
            response = future.result()
            self.get_logger().info(f"ODrive axis{axis_id} state set successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to set state for ODrive axis{axis_id}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RCToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
