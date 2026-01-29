#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vector>
#include <cmath>
#include <string>

class Mapeamento_Node : public rclcpp::Node {
public:
    Mapeamento_Node() : Node("mapeamento_node") {

        mapa = std::vector<std::vector<int>>(3, std::vector<int>(3, -1));
        mapa[1][1] = 0; // centro do mapa

        pub_cer_ = this->create_publisher<std_msgs::msg::String>("mapeamento/cerebro", 10);

        sub_loc_ = this->create_subscription<std_msgs::msg::String>(
            "localizacao/mapeamento", 10,
            std::bind(&Mapeamento_Node::callback_localizacao, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_loc_;

    std::vector<std::vector<int>> mapa;

    double lat0 = 0.0, lon0 = 0.0;
    bool calcular_inicial = true;

    double tamanho_celula = 0.50; // metros
    int centro_y = 1, centro_x = 1;

    ///////

    void callback_localizacao(const std_msgs::msg::String::SharedPtr msg){

        std::string s = msg->data;

        size_t pos1 = s.find('!');
        size_t pos2 = s.find('?');
        if(pos1 == std::string::npos || pos2 == std::string::npos) return;

        double lat = std::stod(s.substr(0, pos1));
        double lon = std::stod(s.substr(pos1 + 1, pos2 - pos1 - 1));
        int estado = std::stoi(s.substr(pos2 + 1));

        if(calcular_inicial){
            lat0 = lat;
            lon0 = lon;
            calcular_inicial = false;
            RCLCPP_INFO(this->get_logger(), "Origem definida.");
            return;
        }

        ///////

        double deg2rad = M_PI / 180.0;
        double dLat = (lat - lat0) * deg2rad;
        double dLon = (lon - lon0) * deg2rad;

        double r = 6378137.0;
        double y = dLat * r;
        double x = dLon * r * cos(lat0 * deg2rad);

        ///////

        int cel_x = static_cast<int>(std::round(x / tamanho_celula));
        int cel_y = static_cast<int>(std::round(y / tamanho_celula));

        int mx = centro_x + cel_x;
        int my = centro_y - cel_y; // inverte Y para matriz

        ////////

        while(my < 0){
            expandir(0);
            my++;
        }

        while(my >= (int)mapa.size()){
            expandir(1);
        }

        while(mx < 0){
            expandir(2);
            mx++;
        }

        while(mx >= (int)mapa[0].size()){
            expandir(3);
        }

        ///////

        mapa[my][mx] = estado;
        RCLCPP_INFO(this->get_logger(), "Mapa: robô em [%d, %d]", my, mx);
    }

    ///////

    void expandir(int direcao){

        int colunas = mapa[0].size();

        switch(direcao){
            case 0: // Topo
                mapa.insert(mapa.begin(), std::vector<int>(colunas, -1));
                centro_y++;
                break;

            case 1: // Baixo
                mapa.push_back(std::vector<int>(colunas, -1));
                break;

            case 2: // Esquerda
                for(auto& linha : mapa)
                    linha.insert(linha.begin(), -1);
                centro_x++;
                break;

            case 3: // Direita
                for(auto& linha : mapa)
                    linha.push_back(-1);
                break;
        }
    }
 
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapeamento_Node>());
    rclcpp::shutdown();
    return 0;
}

