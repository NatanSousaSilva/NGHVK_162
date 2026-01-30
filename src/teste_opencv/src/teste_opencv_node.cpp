#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>

class CameraTesteNode : public rclcpp::Node {
public:
    CameraTesteNode() : Node("camera_teste_node") {

        // Abre a câmera (0 = padrão)
        cap.open(0);

        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao abrir a camera");
            rclcpp::shutdown();
            return;
        }

        // Reduz resolução (importante para Raspberry)
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

        RCLCPP_INFO(this->get_logger(), "Camera aberta com sucesso");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),   // ~30 FPS
            std::bind(&CameraTesteNode::loop, this)
        );
    }

private:
    cv::VideoCapture cap;
    rclcpp::TimerBase::SharedPtr timer_;

    void loop() {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Frame vazio");
            return;
        }

        cv::imshow("Camera ROS2 Teste", frame);
        cv::waitKey(1);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraTesteNode>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
