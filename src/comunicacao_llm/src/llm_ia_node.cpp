#include "rclcpp/rclcpp.hpp"
#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <iostream>

using json = nlohmann::json;

class Llm_Node : public rclcpp::Node {
public:
    Llm_Node() : Node("llm_ia_node") {

        publisher_ = this->create_publisher<std_msgs::msg::String>("llm_resposta", 10);

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "llm_pergunta", 10,
            std::bind(&Llm_Node::subscription_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Nó llm inicializado com sucesso.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    static size_t write_callback(void *contents, size_t size, size_t nmemb, std::string *output) {
        output->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    void subscription_callback(const std_msgs::msg::String::SharedPtr msg) {
        query_llm(msg->data);
    }

    // Método principal 
    void query_llm(const std::string &prompt) {
        CURL *curl = curl_easy_init();
        if (!curl) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao inicializar CURL");
            return;
        }

        std::string url = "http://localhost:11434/api/generate";

        json body_json = {
            {"model", "robo1:latest"},
            {"prompt", prompt},
            {"stream", false}
        };

        std::string request_body = body_json.dump();
        std::string response;

        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());

        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

        CURLcode res = curl_easy_perform(curl);

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Erro CURL: %s", curl_easy_strerror(res));
            return;
        }

        try {
            auto json_response = json::parse(response);

            if (!json_response.contains("response")) {
                RCLCPP_WARN(this->get_logger(),
                    "Resposta JSON sem campo 'response'. Bruto: %s",
                    response.c_str());
                return;
            }

            std::string text = json_response["response"].get<std::string>();

            std_msgs::msg::String msg;
            msg.data = text;
            publisher_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Publicado: '%s'", text.c_str());

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao parsear JSON: %s", e.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Llm_Node>());
    rclcpp::shutdown();
    return 0;
}
