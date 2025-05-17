import rclpy
from rclpy.node import Node
import can
from std_msgs.msg import UInt16MultiArray

class UltrasonicListener(Node):
    def __init__(self):
        super().__init__('ultrasonic_listener')

        # Publisher: distances[8] in mm
        self.publisher_ = self.create_publisher(UInt16MultiArray, 'ultrasonic_distances', 10)

        # Setup CAN interface (SocketCAN)
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

        self.timer = self.create_timer(0.05, self.read_can)  # 20 Hz

        self.distances = [0] * 8  # placeholder

    def read_can(self):
        try:
            message = self.bus.recv(timeout=0.01)
            if message is None:
                return

            if message.arbitration_id == 0x100 and len(message.data) == 8:
                for i in range(4):
                    self.distances[i] = (message.data[i*2] << 8) | message.data[i*2 + 1]

            elif message.arbitration_id == 0x101 and len(message.data) == 8:
                for i in range(4):
                    self.distances[i + 4] = (message.data[i*2] << 8) | message.data[i*2 + 1]

                # Publish only after full set is received
                msg = UInt16MultiArray()
                msg.data = self.distances
                self.publisher_.publish(msg)

        except can.CanError as e:
            self.get_logger().warn(f"CAN error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
