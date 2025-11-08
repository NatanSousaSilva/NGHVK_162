#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vosk_api.h>      // biblioteca do Vosk
#include <alsa/asoundlib.h> // captura de áudio
#include <iostream>

class VoskNode : public rclcpp::Node {
public:
    	VoskNode() : Node("listening_control") {
        	publisher_ = this->create_publisher<std_msgs::msg::String>("conversa_texto", 10);
        	RCLCPP_INFO(this->get_logger(), "Nó Vosk inicializado.");

        	model_ = vosk_model_new("/home/natan/vosk_models/vosk-model-small-pt-0.3");
        	recognizer_ = vosk_recognizer_new(model_, 16000.0); 

        	// Abre o microfone ALSA
        	snd_pcm_open(&capture_handle, "default", SND_PCM_STREAM_CAPTURE, 0);
        	snd_pcm_set_params(
            		capture_handle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
            		1, 16000, 1, 500000 
        	);

        	timer_ = this->create_wall_timer(
            		std::chrono::milliseconds(100),
            		std::bind(&VoskNode::process_audio, this)
        	);
    	}

    	~VoskNode() {
        	snd_pcm_close(capture_handle);
        	vosk_recognizer_free(recognizer_);
        	vosk_model_free(model_);
    	}

private:
    	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    	rclcpp::TimerBase::SharedPtr timer_;

    	snd_pcm_t* capture_handle;     // microfone
    	VoskModel* model_;             // modelo de voz
    	VoskRecognizer* recognizer_;   // reconhecedor

   	 void query_audio() {
               	short buffer[4000];
        	int frames = snd_pcm_readi(capture_handle, buffer, 4000);
        	if (frames < 0) {
            		snd_pcm_prepare(capture_handle);
            		return;
        	}

        	if (vosk_recognizer_accept_waveform(recognizer_, buffer, frames * sizeof(short))) {
            		const char* result = vosk_recognizer_result(recognizer_);
            		publish_text(result);
        	} else {
            		const char* partial = vosk_recognizer_partial_result(recognizer_);
            		RCLCPP_DEBUG(this->get_logger(), "Parcial: %s", partial);
        	}
    	}

    	void publish_text(const char* json_result) {
        	std::string text(json_result);
        	std_msgs::msg::String msg;
        	msg.data = text;
        	publisher_->publish(msg);
        	RCLCPP_INFO(this->get_logger(), "Reconhecido: %s", text.c_str());
    	}
};

int main(int argc, char* argv[]) {
    	rclcpp::init(argc, argv);
    	rclcpp::spin(std::make_shared<VoskNode>());
    	rclcpp::shutdown();
    	return 0;
}

