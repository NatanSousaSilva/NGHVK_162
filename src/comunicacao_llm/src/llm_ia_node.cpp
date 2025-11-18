#include "rclcpp/rclcpp.hpp"
#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <string>
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <std_msgs/msg/string.hpp>

using json = nlohmann::json;

class Gemini_Node : public rclcpp::Node {
public:
    Gemini_Node() : Node("llm_ia_node") {

        publisher_ = this->create_publisher<std_msgs::msg::String>("llm_resposta", 10);

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "pergunta_llm", 10,
            std::bind(&Gemini_Node::subscription_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Nó GeminiNode inicializado com sucesso.");
    }

private:

    std::string api_key_ = std::getenv("GEMINI_API_KEY") ? std::getenv("GEMINI_API_KEY") : "";

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    // Callback do donwload do curl
    static size_t write_callback(void *contents, size_t size, size_t nmemb, std::string *output) {
        output->append((char *)contents, size * nmemb);
        return size * nmemb;
    }

    void subscription_callback(const std_msgs::msg::String::SharedPtr msg) {
        query_gemini(msg->data);
    }

    void query_gemini(const std::string &prompt) {

        CURL *curl = curl_easy_init();
        if (!curl) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao inicializar CURL");
            return;
        }

        std::string url = "https://generativelanguage.googleapis.com/v1/models/gemini-2.5-flash:generateContent?key=" + api_key_;

        // Corpo da requisição
        json body_json = {
            {"contents", {
                {
                    {"parts", {
                        {
                            {"text", prompt}
                        }
                    }}
                 }
             }}
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
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L);

        CURLcode res = curl_easy_perform(curl);

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            RCLCPP_ERROR(this->get_logger(),
                "Falha na requisição CURL: %s",
                curl_easy_strerror(res)
            );
            return;
        }

        try {
            auto json_response = json::parse(response);

            std::string text;

            if (json_response.contains("candidates")) {
                auto &cand = json_response["candidates"][0];
                if (cand.contains("content") &&
                    cand["content"].contains("parts") &&
                    cand["content"]["parts"][0].contains("text")) {

                    text = cand["content"]["parts"][0]["text"]
                               .get<std::string>();
                }
            }

            if (!text.empty()) {
                std_msgs::msg::String msg;
                msg.data = text;

                publisher_->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Publicado: '%s'", text.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Resposta JSON sem campo 'text'. Resposta bruta: %s",
                    response.c_str()
                );
            }

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao parsear JSON: %s", e.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gemini_Node>());
    rclcpp::shutdown();
    return 0;
}
