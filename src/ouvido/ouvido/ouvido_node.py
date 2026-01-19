import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice as sd
import json
from vosk import Model, KaldiRecognizer
from rapidfuzz import process, fuzz

import os

class Ouvido_Node(Node):
    def __init__(self):
        super().__init__("ouvido_node")

        self.publisher_ = self.create_publisher(String, "ouvido/cerebro", 10)
        self.get_logger().info("ouvido inicializado com fuzzy")

        #####

        self.model_path = os.path.expanduser("~/iracema/vosk_models/vosk-model-small-pt-0.3")
        self.model = Model(self.model_path)
        self.recognizer = KaldiRecognizer(self.model, 16000)

        #####

        self.nomes_cadastrados = [
            "natan",
            "joão",
            "maria",
            "pedro"
        ]

        #####

        self.stream = sd.RawInputStream(
            samplerate=16000,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=self.audio_callback
        )
        self.stream.start()

    # -----

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warning(str(status))

        audio_bytes = bytes(indata)

        if self.recognizer.AcceptWaveform(audio_bytes):
            result = json.loads(self.recognizer.Result())
            texto = result.get("text", "").lower().strip()

            if texto:
                # texto_corrigido = self.corrigir_nome(texto)
                self.publish_text(texto)

    # -----

    def corrigir_nome(self, texto):  # Compara o texto reconhecido com a lista de nomes
        melhor, score, _ = process.extractOne(
            texto,
            self.nomes_cadastrados,
            scorer=fuzz.ratio
        )

        self.get_logger().info(
            f"Fuzzy: '{texto}' → '{melhor}' ({score}%)"
        )

        if score >= 80:
            return melhor
        else:
            return texto

    # -----

    def publish_text(self, texto):
        msg = String()
        msg.data = texto
        self.publisher_.publish(msg)

        self.get_logger().info(f"Publicado: {texto}")


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
