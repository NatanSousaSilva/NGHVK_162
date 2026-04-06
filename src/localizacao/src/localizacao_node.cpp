#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>


class Localizacao_Node : public rclcpp::Node {
public:
    Localizacao_Node() : Node("localizacao_node") {
        configurar_uart();

        pub_cer_ = this->create_publisher<std_msgs::msg::String>("localizacao/cerebro", 10);

        sub_cer_ = this->create_subscription<std_msgs::msg::String>(
            "cerebro/localizacao", 10,
            std::bind(&Localizacao_Node::callback_cerebro, this, std::placeholders::_1)
        );

    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cer_;

    int fd;

    ////////

    void callback_cerebro(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    ///////

    void ler_serial(){
        char buffer[256];

        while (true){
            int n = read(fd, buffer, sizeof(buffer) - 1);

            if (n > 0){
                buffer[n] = '\0';
                std::cout << buffer;
            }
        }

    }
    ////////

    void configuracao_uart(){
        fd = open("/dev/serial0", O_RDWR | O_NOCTTY);

        if (fd < 0){
            std::cout << "Erro ao abrir GPS\n";
            return 1;
        }

        struct termios tty;
        tcgetattr(fd, &tty);

        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tcsetattr(fd, TCSANOW, &tty);
    }


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localizacao_Node>());
    rclcpp::shutdown();
    return 0;
}
