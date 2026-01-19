import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import serial

class SerialSender(Node):
    def __init__(self):
        super().__init__('serial_sender')

        try:
            self.serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Serial conectada em /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Erro ao abrir serial: {e}")
            self.serial = None

        self.sub_loc_ = self.create_subscription(
            String,
            'localizacao/serial',
            self.callback_localizacao,
            10
        )

        self.sub_olh_ = self.create_subscription(
            String,
            'olhos/serial',
            self.callback_olhos,
            10
        )

        self.pub_loc_ = self.create_publisher(
            String,
            'serial/localizacao',
            10
        )

    def callback_localizacao(self, msg):
        self.get_logger().info(f"Feedback recebido: {msg.data}")

    def callback_olhos(self, msg):
        self.get_logger().info(f"Feedback recebido: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
