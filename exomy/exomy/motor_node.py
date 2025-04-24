#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from exomy_msgs.msg import MotorCommands
from .motors import Motors

class MotorNode(Node):
    """Convert Motor Commands"""

    def __init__(self):
        """Init Node."""
        self.node_name = 'motor_node'
        super().__init__(self.node_name)

        # Initialize parameters
        self.parameters = {}
        self.init_params()

        # Create Subscription
        self.subscription = self.create_subscription(
            MotorCommands,
            'motor_commands',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create motor instances
        self.motors = Motors(self.parameters)

        # Create publisher for motor_commands_with_timestamp topic
        self.timestamped_motor_commands_publisher = self.create_publisher(MotorCommands, 'motor_commands_with_timestamp', 10)


        # Create watchdog timer
        self.watchdog_timer = self.create_timer(50, self.watchdog)
        #self.motors.stopMotors()

        self.get_logger().info('\t{} STARTED.'.format(self.node_name.upper()))

    def __del__(self):
        self.motors.stopMotors()

    def init_params(self):
        """Initialize Parameters."""

        # Pin numbers for motors on PWM board
        self.declare_parameter('pin_drive_fl', 15)
        self.parameters['pin_drive_fl'] = self.get_parameter('pin_drive_fl').value
        self.declare_parameter('pin_steer_fl', 13)
        self.parameters['pin_steer_fl'] = self.get_parameter('pin_steer_fl').value

        self.declare_parameter('pin_drive_fr', 8)
        self.parameters['pin_drive_fr'] = self.get_parameter('pin_drive_fr').value
        self.declare_parameter('pin_steer_fr', 11)
        self.parameters['pin_steer_fr'] = self.get_parameter('pin_steer_fr').value

        self.declare_parameter('pin_drive_cl', 14)
        self.parameters['pin_drive_cl'] = self.get_parameter('pin_drive_cl').value
        self.declare_parameter('pin_steer_cl', 12)
        self.parameters['pin_steer_cl'] = self.get_parameter('pin_steer_cl').value

        self.declare_parameter('pin_drive_cr', 9)
        self.parameters['pin_drive_cr'] = self.get_parameter('pin_drive_cr').value
        self.declare_parameter('pin_steer_cr', 10)
        self.parameters['pin_steer_cr'] = self.get_parameter('pin_steer_cr').value

        self.declare_parameter('pin_drive_rl', 7)
        self.parameters['pin_drive_rl'] = self.get_parameter('pin_drive_rl').value
        self.declare_parameter('pin_steer_rl', 5)
        self.parameters['pin_steer_rl'] = self.get_parameter('pin_steer_rl').value

        self.declare_parameter('pin_drive_rr', 6)
        self.parameters['pin_drive_rr'] = self.get_parameter('pin_drive_rr').value
        self.declare_parameter('pin_steer_rr', 4)
        self.parameters['pin_steer_rr'] = self.get_parameter('pin_steer_rr').value

        ## PWM values
        # Those values highly depend on the motors and pwm board used
        # Check out the wiki for further explanation
        self.declare_parameter('steer_pwm_neutral_fl', 335)
        self.parameters['steer_pwm_neutral_fl'] = self.get_parameter('steer_pwm_neutral_fl').value
        self.declare_parameter('steer_pwm_neutral_fr', 342)
        self.parameters['steer_pwm_neutral_fr'] = self.get_parameter('steer_pwm_neutral_fr').value
        self.declare_parameter('steer_pwm_neutral_cl', 330)
        self.parameters['steer_pwm_neutral_cl'] = self.get_parameter('steer_pwm_neutral_cl').value
        self.declare_parameter('steer_pwm_neutral_cr', 350)
        self.parameters['steer_pwm_neutral_cr'] = self.get_parameter('steer_pwm_neutral_cr').value
        self.declare_parameter('steer_pwm_neutral_rl', 316)
        self.parameters['steer_pwm_neutral_rl'] = self.get_parameter('steer_pwm_neutral_rl').value
        self.declare_parameter('steer_pwm_neutral_rr', 316)
        self.parameters['steer_pwm_neutral_rr'] = self.get_parameter('steer_pwm_neutral_rr').value

        # PWW range around steering neutral position
        self.declare_parameter('steer_pwm_range', 190)
        self.parameters['steer_pwm_range'] = self.get_parameter('steer_pwm_range').value

        # PWM value for the stop position of driving motors
        self.declare_parameter('drive_pwm_neutral', 307)
        self.parameters['drive_pwm_neutral'] = self.get_parameter('drive_pwm_neutral').value

        # PWW range around driving neutral position
        self.declare_parameter('drive_pwm_range', 41)
        self.parameters['drive_pwm_range'] = self.get_parameter('drive_pwm_range').value

        self.declare_parameter('drive_pwm_range_fr', 6)
        self.parameters['drive_pwm_range_fr'] = self.get_parameter('drive_pwm_range_fr').value
        self.declare_parameter('drive_pwm_range_cr', 6)
        self.parameters['drive_pwm_range_cr'] = self.get_parameter('drive_pwm_range_cr').value
        self.declare_parameter('drive_pwm_range_rr', 6)
        self.parameters['drive_pwm_range_rr'] = self.get_parameter('drive_pwm_range_rr').value

        self.declare_parameter('drive_pwm_range_fl', 5)
        self.parameters['drive_pwm_range_fl'] = self.get_parameter('drive_pwm_range_fl').value
        self.declare_parameter('drive_pwm_range_cl', 5)
        self.parameters['drive_pwm_range_cl'] = self.get_parameter('drive_pwm_range_cl').value
        self.declare_parameter('drive_pwm_range_rl', 5)
        self.parameters['drive_pwm_range_rl'] = self.get_parameter('drive_pwm_range_rl').value

    def callback(self, cmds):
        self.motors.setSteering(cmds.motor_angles)
        self.motors.setDriving(cmds.motor_speeds)
        # Add timestamp to the header of the received MotorCommands message
        cmds.header.stamp = self.get_clock().now().to_msg()

        # Publish the MotorCommands message with timestamp
        self.timestamped_motor_commands_publisher.publish(cmds)

        self.watchdog_timer.cancel()
        # If this timer runs longer than the duration specified,
        # then watchdog() is called stopping the driving motors.
        # Preventing the robot to go on driving if connection is lost.
        self.watchdog_timer = self.create_timer(50, self.watchdog)

    def watchdog(self):
        self.get_logger().info('Watchdog fired. Stopping driving motors.')
        self.motors.stopMotors()


def main(args=None):
    rclpy.init(args=args)

    try:
        motor_node = MotorNode()
        try:
            rclpy.spin(motor_node)
        finally:
            motor_node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
