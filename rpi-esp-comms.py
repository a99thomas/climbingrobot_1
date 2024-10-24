import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import ParameterDescriptor
import serial
import time

class ESP32Controller(Node):
    def __init__(self):
        super().__init__('esp32_controller')

        # Serial connection setup
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)  # Allow the ESP32 to reset

        # ROS 2 Subscribers and Publishers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_command_callback,
            10
        )
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Setup parameters for PID gains (for motors 3 and 7)
        self.declare_parameter('motor3_pid', [1.0, 0.0, 0.0], ParameterDescriptor(description='PID for Motor 3'))
        self.declare_parameter('motor7_pid', [1.0, 0.0, 0.0], ParameterDescriptor(description='PID for Motor 7'))

        # Timer to periodically send joint states
        self.create_timer(0.1, self.publish_joint_states)

    def joint_command_callback(self, msg):
        """Send joint commands to ESP32 over serial."""
        if len(msg.data) != 8:
            self.get_logger().error('Expected 8 joint commands, got {}'.format(len(msg.data)))
            return

        # Format the joint commands and send them to ESP32
        command = ','.join([str(val) for val in msg.data]) + '\n'
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent joint commands: {command.strip()}')

    def publish_joint_states(self):
        """Receive joint states from ESP32 and publish as JointState message."""
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            values = [float(v) for v in line.split(',')]
            if len(values) != 8:
                self.get_logger().warn(f'Received unexpected data: {line}')
                return

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [
                'joint_1', 'joint_2', 'motor_3', 'joint_4', 
                'joint_5', 'joint_6', 'motor_7', 'joint_8'
            ]
            joint_state_msg.position = values
            self.joint_state_pub.publish(joint_state_msg)
            self.get_logger().info(f'Published joint states: {values}')
        except Exception as e:
            self.get_logger().error(f'Error reading from serial: {str(e)}')

    def set_pid_values(self):
        """Send updated PID values to ESP32 via serial."""
        pid3 = self.get_parameter('motor3_pid').value
        pid7 = self.get_parameter('motor7_pid').value

        # Format and send PID commands
        pid_command_3 = f'PID3,{pid3[0]},{pid3[1]},{pid3[2]}\n'
        pid_command_7 = f'PID7,{pid7[0]},{pid7[1]},{pid7[2]}\n'

        self.serial_port.write(pid_command_3.encode('utf-8'))
        self.serial_port.write(pid_command_7.encode('utf-8'))

        self.get_logger().info(f'Sent PID for Motor 3: {pid3}')
        self.get_logger().info(f'Sent PID for Motor 7: {pid7}')

    def destroy_node(self):
        """Close the serial port on shutdown."""
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = ESP32Controller()

    try:
        controller.set_pid_values()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
