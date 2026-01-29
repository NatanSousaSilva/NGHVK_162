#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>

class Localizacao_Node : public rclcpp::Node {
public:
    Localizacao_Node() : Node("localizacao_node") {
        pub_cer_ = this->create_publisher<std_msgs::msg::String>("localizacao/cerebro", 10);
        pub_ser_ = this->create_publisher<std_msgs::msg::String>("localizacao/serial", 10);

        sub_cer_ = this->create_subscription<std_msgs::msg::String>(
            "cerebro/localizacao", 10,
            std::bind(&Localizacao_Node::callback_cerebro, this, std::placeholders::_1)
        );

        sub_ser_ = this->create_subscription<std_msgs::msg::String>(
            "serial/localizacao", 10,
            std::bind(&Localizacao_Node::callback_serial, this, std::placeholders::_1)
        );
    }

private:

    // no codigo do serial mandar para ca so a posição do gps e se tem ou nao coisa 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ser_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ser_;
    ////////

    void callback_cerebro(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    void callback_serial(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localizacao_Node>());
    rclcpp::shutdown();
    return 0;
}
