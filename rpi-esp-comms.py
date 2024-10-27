import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import serial
import time
import math
import threading
from queue import Queue

class ESP32Controller(Node):
    def __init__(self):
        super().__init__('esp32_controller')

        # Serial connection setup
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(1)  # Allow the ESP32 to reset

        # ROS 2 Publishers, Subscribers, and Parameters
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states_1', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.declare_pid_parameters()
        self.add_on_set_parameters_callback(self.parameter_update_callback)

        # Serial message queue
        self.serial_queue = Queue()

        # Start background thread to read from serial
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

        # Publish joint states at regular intervals
        self.create_timer(0.1, self.publish_joint_states)
        self.create_timer(0.11, self.send_joint_commands)

        
        self.latest_joint_state = None

    def declare_pid_parameters(self):
        """Declare PID parameters with default values."""
        self.declare_parameter('motor3a_pid', [40.0, 0.0, 3.0], ParameterDescriptor(description='PIDa for Motor 3'))
        self.declare_parameter('motor3b_pid', [40.0, 5.0, 3.0], ParameterDescriptor(description='PIDb for Motor 3'))
        self.declare_parameter('motor7a_pid', [40.0, 0.0, 3.0], ParameterDescriptor(description='PIDa for Motor 7'))
        self.declare_parameter('motor7b_pid', [40.0, 5.0, 3.0], ParameterDescriptor(description='PIDb for Motor 7'))

    def joint_state_callback(self, msg):
        """Handle incoming joint states."""
        if len(msg.position) != 6:
            self.get_logger().error(f'Expected 6 joint positions, got {len(msg.position)}')
            return
        self.latest_joint_state = msg
        # self.send_joint_commands()

    def send_joint_commands(self):
        """Send joint states to ESP32 over serial."""
        if self.latest_joint_state is None:
            return

        formatted_values = [
            round(val * (180.0 / math.pi), 2) if i in [0, 1, 3, 4] else round(val, 4)
            for i, val in enumerate(self.latest_joint_state.position)
        ]
        formatted_values.insert(3, 0)  # Insert placeholder values
        formatted_values.insert(7, 0)

        command = ','.join([f"{val:.4f}" for val in formatted_values]) + '\n'
        print(command.encode('utf-8'))
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f'Sent joint states: {command.strip()}')

    def read_serial(self):
        """Continuously read from the serial port and queue messages."""
        while True:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.serial_queue.put(line)
            except Exception as e:
                self.get_logger().error(f'Error reading from serial: {e}')

    def publish_joint_states(self):
        """Publish joint states from the serial queue."""
        while not self.serial_queue.empty():
            line = self.serial_queue.get()
            try:
                values = [float(v) for v in line.split(',')]
                if len(values) != 8:
                    self.get_logger().warn(f'Received unexpected data: {line}')
                    continue

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [
                    'joint_1', 'joint_2', 'motor_3', 'joint_4',
                    'joint_5', 'joint_6', 'motor_7', 'joint_8'
                ]
                joint_state_msg.position = values
                self.joint_state_pub.publish(joint_state_msg)
                self.get_logger().info(f'Published joint states: {values}')
            except ValueError as e:
                self.get_logger().error(f'Error parsing serial data: {e}')

    def parameter_update_callback(self, params):
        """Handle parameter updates."""
        updated_params = {param.name: param.value for param in params}
        for name, value in updated_params.items():
            self.get_logger().info(f'Parameter {name} updated to {value}')
        self.set_pid_values(updated_params)
        return SetParametersResult(successful=True)

    def set_pid_values(self, updated_params):
        """Send updated PID values to ESP32."""
        for command, params in zip(
            ['PIDa3', 'PIDb3', 'PIDa7', 'PIDb7'], 
            [updated_params.get('motor3a_pid', self.get_parameter('motor3a_pid').value),
             updated_params.get('motor3b_pid', self.get_parameter('motor3b_pid').value),
             updated_params.get('motor7a_pid', self.get_parameter('motor7a_pid').value),
             updated_params.get('motor7b_pid', self.get_parameter('motor7b_pid').value)]
        ):
            self.serial_port.write(f'{command},{params[0]},{params[1]},{params[2]}\n'.encode('utf-8'))
            self.get_logger().info(f'Sent {command}: {params}')

    def destroy_node(self):
        """Close the serial port on shutdown."""
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = ESP32Controller()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
