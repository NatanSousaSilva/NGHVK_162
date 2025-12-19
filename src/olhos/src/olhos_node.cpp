#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <vector>

using namespace cv;

class Olhos_Node : public rclcpp::Node {
public:
    Olhos_Node() : Node("olhos_node") {

        face_cascade.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml");

        cap.open(2);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Erro ao abrir câmera!");
        }

        pub_cen_ = this->create_publisher<std_msgs::msg::String>("centralizar_rosto", 10);
        pub_fac_ = this->create_publisher<sensor_msgs::msg::Image>("rosto", 10);


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&Olhos_Node::processar_frame, this)
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cen_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_fac_;
    rclcpp::TimerBase::SharedPtr timer_;

    CascadeClassifier face_cascade;
    VideoCapture cap;

    Mat frame, gray;

    std_msgs::msg::String msg_cen_;

    bool cadastrar = true;

    void processar_frame() {
        cap >> frame;
        if (frame.empty()) return;

        std::vector<Rect> faces;

        int width  = frame.cols;
        int height = frame.rows;
        int centro_x = width / 2;
        int centro_y = height / 2;

        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        equalizeHist(gray, gray);

        face_cascade.detectMultiScale(gray, faces, 1.1, 4, 0, cv::Size(30, 30));

        if (!faces.empty()) {
            Rect maior = faces[0];
            for (auto &f : faces)
                if (f.area() > maior.area())
                    maior = f;

            int face_centro_x = maior.x + maior.width / 2;
            int face_centro_y = maior.y + maior.height / 2;

            int off_set_y = face_centro_y - centro_y;
            int off_set_x = face_centro_x - centro_x;

            msg_cen_.data = std::to_string(off_set_x) + "," +  std::to_string(off_set_y);
            pub_cen_->publish(msg_cen_);


            if ((off_set_x <= 5) && (off_set_x >= -5) &&
                (off_set_y <= 5) && (off_set_y >= -5) &&
                (cadastrar == true)){
                
                cv_bridge::CvImage msg_fac;
                msg_fac.header.stamp = this->now();
                msg_fac.header.frame_id = "camera_link"; 
                
                msg_fac.encoding = "bgr8";
                msg_fac.image = frame(maior).clone();

                pub_fac_->publish(*msg_fac.toImageMsg());
                cadastrar = false;
            }
        }

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Olhos_Node>());
    rclcpp::shutdown();
    return 0;
}
