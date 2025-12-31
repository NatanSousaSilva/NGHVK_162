import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8

import sounddevice as sd
import json
import serial
from vosk import Model, KaldiRecognizer


class Ouvido_Node(Node):
    def __init__(self):
        super().__init__("ouvido_node")
        self.publisher_ = self.create_publisher(String, "ouvido/cerebro", 10)
        self.get_logger().info("ouvido inicializado")

        self.model_path = "/home/natan/vosk_models/vosk-model-small-pt-0.3"
        self.model = Model(self.model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)

        self.stream = sd.RawInputStream(
            samplerate=16000,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=self.audio_callback
        )
        self.stream.start()

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warning(f"Status do áudio: {status}")

        audio_bytes = bytes(indata)

        if self.recognizer.AcceptWaveform(audio_bytes):
            result = json.loads(self.recognizer.Result())
            text = result.get("text", "").strip()

            if text:
                self.publish_text(text)

    
    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"Reconhecido: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = Ouvido_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


