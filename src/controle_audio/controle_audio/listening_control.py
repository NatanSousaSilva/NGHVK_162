import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8

import sounddevice as sd
import json
import serial
from vosk import Model, KaldiRecognizer


class interpretacao_audio(Node):
    def __init__(self):
        super().__init__("listening_control")
        self.publisher_ = self.create_publisher(String, "dados_escuta", 10)
        self.get_logger().info("listening_control inicializado")

        self.estado_botao = 0
        
        try:
            self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.01)
            self.get_logger().info("Serial /dev/ttyUSB0 conectada")
        except Exception as e:
            self.get_logger().error(f"Falha ao abrir serial: {e}")
            raise

        self.timer_serial = self.create_timer(
            0.01,  
            self.serial_callback
        )
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

    def serial_callback(self):
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.readline().decode().strip()

                if data.isdigit():
                    valor = int(data)

                    self.estado_botao = valor
                    #self.get_logger().info(valor)

        except Exception as e:
            self.get_logger().error(f"Erro lendo serial: {e}")

    def audio_callback(self, indata, frames, time, status):
        if self.estado_botao == 0:
            return

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
    node = interpretacao_audio()
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

