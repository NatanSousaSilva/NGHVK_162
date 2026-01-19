#include "rclcpp/rclcpp.hpp"
#include <string>
#include <sstream>
#include <std_msgs/msg/string.hpp>

class Cerebro_Node : public rclcpp::Node {
public:
    Cerebro_Node() : Node("cerebro_node") {

        // publishers
        pub_llm_ = this->create_publisher<std_msgs::msg::String>("cerebro/llm", 10);
        pub_des_ = this->create_publisher<std_msgs::msg::String>("cerebro/localizacao", 10);
        pub_boc_ = this->create_publisher<std_msgs::msg::String>("cerebro/boca", 10);
        pub_olh_ = this->create_publisher<std_msgs::msg::String>("cerebro/olhos", 10);

        // subscriptions
        sub_llm_ = this->create_subscription<std_msgs::msg::String>(
            "llm/cerebro", 10,
            std::bind(&Cerebro_Node::callback_llm, this, std::placeholders::_1)
        );

        sub_ouv_ = this->create_subscription<std_msgs::msg::String>(
            "ouvido/cerebro", 10,
            std::bind(&Cerebro_Node::callback_ouvido, this, std::placeholders::_1)
        );

        /*
        sub_olh_ = this->create_subscription<sensor_msgs::msg::Image>(
            "olhos/cerebro", 10,
            std::bind(&Cerebro_Node::callback_olhos, this, std::placeholders::_1)
        );*/

        RCLCPP_INFO(this->get_logger(), "Nó brain inicializado com sucesso.");
    }

private:

    // publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_llm_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_des_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_boc_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_olh_;

    // subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_llm_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ouv_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_olh_;

   //--------
    void callback_llm(const std_msgs::msg::String::SharedPtr msg_llm_) {
        std_msgs::msg::String msg; // variavel para ser enviada para um topico após tratada
        std::string msg_str_ = msg_llm_->data; // string moldavel

        if (msg_str_.empty())return;

        RCLCPP_INFO(this->get_logger(), "Recebi LLM: %s", msg_str_.c_str());

        if (msg_str_.find("l0-?") != std::string::npos) {

            std::string msgs = msg_str_.substr(4);

            if (msgs != "nao") {
                RCLCPP_INFO(this->get_logger(), "Local encontrado: %s", msgs.c_str());
            }
        }

        else if (msg_str_.find("c4-?") != std::string::npos) {
            msg.data = msg_str_.substr(4);
            pub_olh_->publish(msg); 

        } 

        else if (msg_str_.find("p1-?") != std::string::npos) {
            msg.data = msg_str_.substr(4);
            pub_boc_->publish(msg); 

        }

        else if (msg_str_.find("r3-?") != std::string::npos) {
            msg.data = msg_str_.substr(4);
            pub_boc_->publish(msg); 

        }

        else {
            msg.data = msg_llm_->data;
            pub_boc_->publish(msg);
        }



    }

    //--------
    void callback_ouvido(const std_msgs::msg::String::SharedPtr msg_esc_) {
        std::string msg_esc = msg_esc_->data;

        if (msg_esc.find("ande") != std::string::npos ||
            msg_esc.find("vá") != std::string::npos ||
            msg_esc.find("ir") != std::string::npos)
        {
            std_msgs::msg::String msg;
            msg.data =
                "Identifique o local e/ou a instrução de localização dessa frase: '" +
                msg_esc +
                "'. Escreva nesse formato: 'l0-?_____' sendo '_____' o local/instrução "
                "e '-' se é pergunta(p) ou ordem(o). Caso não haja local escreva 'l0-?nao'.";

            pub_llm_->publish(msg);
        }

        else if (msg_esc.find("prorcure") != std::string::npos ||
                 msg_esc.find("encontre") != std::string::npos ||
                 msg_esc.find("ache") != std::string::npos)
        {
            std_msgs::msg::String msg;
            msg.data =
                "Identifique se existe algum nome na frase: '" +
                msg_esc +
                "'. Escreva nesse formato: 'p1-?_____' sendo '_____' o nome. "
                "Caso não haja nome escreva 'p1-?nao'.";

            pub_llm_->publish(msg);
        }

        else if (msg_esc.find("cadastre") != std::string::npos ||
                 msg_esc.find("cadastrar") != std::string::npos ||
                 msg_esc.find("meu nome é") != std::string::npos)
        {
            std_msgs::msg::String msg;
            msg.data = 
                "Identifique se existe algum nome na frase: '" +
                msg_esc +
                "'. Escreva nesse formato: 'c4-?_____' sendo '_____' o nome. "
                "Caso não haja nome escreva 'c4-?nao'.";  

            pub_llm_->publish(msg);
        }


        else {
            std_msgs::msg::String msg;
            msg.data =
                "Responda como um robô assistente para: '" +
                msg_esc +
                "'. Escreva no formato: 'r3-?_____' sendo '_____' a resposta.";

            pub_llm_->publish(msg);
        }
    }


    //--------

    /*
    void callback_olhos(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;

        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Erro cv_bridge: %s", e.what());
            return;
        }

        auto tempo = std::chrono::system_clock::now().time_since_epoch();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tempo).count();

        std::string arquivo = pasta_ + "/img_" + std::to_string(ms) + ".png";

        cv::imwrite(arquivo, frame);

        RCLCPP_INFO(this->get_logger(), "Imagem salva em: %s", arquivo.c_str());
    }
    */

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cerebro_Node>());
    rclcpp::shutdown();
    return 0;
}
