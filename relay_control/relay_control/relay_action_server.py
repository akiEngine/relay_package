import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from relay_control_interfaces.action import Relay
from std_msgs.msg import Bool
import serial
import time
import glob

class RelayActionServer(Node):
    def __init__(self):
        super().__init__('relay_action_server')
        self.server = ActionServer(self, Relay, 'relay_control', self.execute_callback)

        # Set up serial connection to the relay
        serial_port = self.get_serial_port()
        self.serial_conn = serial.Serial(serial_port, 9600)
        time.sleep(1)

        # Initialize relay states
        self.state_r1 = False
        self.state_r2 = False

        # Set up subscriptions to relay topics
        self.create_subscription(Bool, 'relay1', self.relay1_callback, 10)
        self.create_subscription(Bool, 'relay2', self.relay2_callback, 10)

        # Create an action client
        self.action_client = ActionClient(self, Relay, 'relay_control')

    def get_serial_port(self):
        # Get the serial port using its UUID
        serial_ports = glob.glob('/dev/serial/by-id/*')
        for port in serial_ports:
            if 'usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0' in port:
                return port
        raise RuntimeError("Serial port not found")

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received order r1: {goal_handle.request.r1}, r2: {goal_handle.request.r2}")

        R1 = goal_handle.request.r1
        R2 = goal_handle.request.r2
        possible_orders = [[False, False],[True, False],[False, True],[True,True]]
        possible_bytes = [bytes([0x00]),bytes([0x01]),bytes([0x02]),bytes([0x03])]
        command = bytes([0x00])
        for i in range(len(possible_orders)):
            if [R1,R2] == possible_orders[i]:
                command = possible_bytes[i]
        
        # Send commands to the relay
        self.serial_conn.write(command)
        time.sleep(1)

        # Feedback: current state
        feedback = Relay.Feedback()
        feedback.state_r1 = R1
        feedback.state_r2 = R2
        goal_handle.publish_feedback(feedback)

        # Result: done state
        goal_handle.succeed()
        result = Relay.Result()
        result.done = True

        self.get_logger().info("Order execution completed")
        return result

    def relay1_callback(self, msg):
        self.get_logger().info(f"Received relay1 state: {msg.data}")
        self.state_r1 = msg.data
        self.send_goal(self.state_r1, self.state_r2)

    def relay2_callback(self, msg):
        self.get_logger().info(f"Received relay2 state: {msg.data}")
        self.state_r2 = msg.data
        self.send_goal(self.state_r1, self.state_r2)

    def send_goal(self, state_r1, state_r2):
        goal_msg = Relay.Goal()
        goal_msg.r1 = state_r1
        goal_msg.r2 = state_r2
        self.get_logger().info(f"Sending goal: r1={goal_msg.r1}, r2={goal_msg.r2}")

        # Ensure the action server is available before sending a goal
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server not available')
            return

        self.action_client.send_goal_async(goal_msg)

    def destroy_node(self):
        # Close the serial connection before destroying the node
        self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RelayActionServer()
    serial_port = node.get_serial_port()
    serial_conn = serial.Serial(serial_port, 9600)
    time.sleep(1)
    serial_conn.write(bytes([0x50])) #gets board's type (nb of relay)
    time.sleep(0.5)
    serial_conn.write(bytes([0x51])) #initializes com
    time.sleep(1)
    #serial_conn.write(bytes([0xFF])) #turns off all relay 
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
