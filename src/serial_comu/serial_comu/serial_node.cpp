import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
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

        # --- ASSINATURA DO TÓPICO ---
        self.subscription = self.create_subscription(
            Int32,
            'comando_motor',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        valor = msg.data
        self.get_logger().info(f"Recebido: {valor}")

        if self.serial is not None:
            # Envia o valor como string para o Arduino
            try:
                self.serial.write(f"{valor}\n".encode())
            except Exception as e:
                self.get_logger().error(f"Erro ao enviar pela serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
