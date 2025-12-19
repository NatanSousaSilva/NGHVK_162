#include "rclcpp/rclcpp.hpp"
#include <string>
#include <sstream>
#include <std_msgs/msg/string.hpp>

class Controle_Node : public rclcpp::Node {
public:
    Controle_Node() : Node("brain_node") {

        // publishers
        pub_llm_ = this->create_publisher<std_msgs::msg::String>("llm_pergunta", 10);
        pub_des_ = this->create_publisher<std_msgs::msg::String>("destino", 10);
        pub_aud_ = this->create_publisher<std_msgs::msg::String>("audio", 10);

        // subscriptions
        sub_llm_ = this->create_subscription<std_msgs::msg::String>(
            "llm_resposta", 10,
            std::bind(&Controle_Node::callback_llm, this, std::placeholders::_1)
        );

        sub_esc_ = this->create_subscription<std_msgs::msg::String>(
            "dados_escuta", 10,
            std::bind(&Controle_Node::callback_audio, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Nó brain inicializado com sucesso.");
    }

private:

    // publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_llm_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_des_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_aud_;

    // subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_llm_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_esc_;

    //--------
    void callback_llm(const std_msgs::msg::String::SharedPtr msg_llm__) {
        std_msgs::msg::String msg_llm_;
        std::string msg_llm = msg_llm__->data;

        if (msg_llm.empty()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Recebi LLM: %s", msg_llm.c_str());

        if (msg_llm.find("l0?") != std::string::npos) {

            std::string msg = msg_llm.substr(3);

            if (msg != "nao") {
                RCLCPP_INFO(this->get_logger(), "Local encontrado: %s", msg.c_str());
            }
        }else if (msg_llm.find("r3?") != std::string::npos) {
            msg_llm_.data = msg_llm.substr(3);
            pub_aud_->publish(msg_llm_); 

        } else {
            pub_aud_->publish(msg_llm_);
        }
    }

    //--------
    void callback_audio(const std_msgs::msg::String::SharedPtr msg_esc_) {
        std::string msg_esc = msg_esc_->data;

        if (msg_esc.find("ande") != std::string::npos ||
            msg_esc.find("vá") != std::string::npos ||
            msg_esc.find("ir") != std::string::npos)
        {
            std_msgs::msg::String msg;
            msg.data =
                "Identifique o local e/ou a instrução de localização dessa frase: '" +
                msg_esc +
                "'. Escreva nesse formato: 'l0?-_____' sendo '_____' o local/instrução "
                "e '-' se é pergunta(p) ou ordem(o). Caso não haja local escreva 'l0?nao'.";

            pub_llm_->publish(msg);
        }

        else if (msg_esc.find("galega") != std::string::npos ||
                 msg_esc.find("encontre") != std::string::npos ||
                 msg_esc.find("ache") != std::string::npos)
        {
            std_msgs::msg::String msg;
            msg.data = "responda apenas a palavra glehdson";
/*
                "Identifique se existe algum nome na frase: '" +
                msg_esc +
                "'. Escreva nesse formato: 'n0?_____' sendo '_____' o nome. "
                "Caso não haja nome escreva 'n0?nao'.";*/

            pub_llm_->publish(msg);
        }

        else {
            std_msgs::msg::String msg;
            msg.data =
                "Responda como um robô assistente para: '" +
                msg_esc +
                "'. Escreva no formato: 'r3?_____' sendo '_____' a resposta.";

            pub_llm_->publish(msg);
        }
    }

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controle_Node>());
    rclcpp::shutdown();
    return 0;
}

