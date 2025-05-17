import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame  # standard message type for CAN frames

class CanLoadNode(Node):
    def __init__(self):
        super().__init__('can_load_node')
        self.subscription = self.create_subscription(
            Frame,
            '/can_rx',  # Adjust this topic to match your actual CAN receive topic
            self.can_callback,
            10)
        self.get_logger().info("WeightReceiverNode started and listening to CAN frames")

    def can_callback(self, msg: Frame):
        # Check for correct CAN ID (standard ID 0x123)
        if msg.id == 0x123 and not msg.is_rtr and not msg.is_error:
            if len(msg.data) >= 4:
                # Decode 4 bytes weight, big-endian signed int
                weight = int.from_bytes(msg.data[0:4], byteorder='big', signed=True)
                weight_kg = weight / 1_000_000  # convert mg to kg
                self.get_logger().info(f"Received weight: {weight} mg ({weight_kg:.3f} kg)")

def main(args=None):
    rclpy.init(args=args)
    node = CanLoadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
