#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <vector>
#include <numeric>

class CorridorNavigationNode : public rclcpp::Node
{
public:
    CorridorNavigationNode() : Node("corr_nav_node")
    {
        loadParameters();

        RCLCPP_INFO(this->get_logger(), "CorridorNavigationNode started");

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/PioneerP3DX/cmd_vel", 10);
        
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/PioneerP3DX/odom", 10,
            std::bind(&CorridorNavigationNode::odomCallback, this, std::placeholders::_1));

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/PioneerP3DX/laser_scan", 10,
            std::bind(&CorridorNavigationNode::laserCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(time_step_),
            std::bind(&CorridorNavigationNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Corridor Navigation Node Initialized");
        measured_data_ = false;
    }

private:
    void loadParameters()
    {
        this->declare_parameter("time_step", 25);
        this->declare_parameter("max_linear_speed", 1.2);
        this->declare_parameter("max_angular_speed", 2.0);
        this->declare_parameter("wheel_base", 0.331);
        this->declare_parameter("wheel_radius", 0.097518);
        this->declare_parameter("corridor_width", 10.0);
        this->declare_parameter("look_ahead_distance", 1.0);
        
        time_step_ = this->get_parameter("time_step").as_int();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        corridor_width_ = this->get_parameter("corridor_width").as_double();
        look_ahead_distance_ = this->get_parameter("look_ahead_distance").as_double();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        current_theta_ = std::atan2(siny_cosp, cosy_cosp);
    }

    // Función para extraer la pared mediante regresión lineal (Least Squares Fitting)
    bool extractWall(const sensor_msgs::msg::LaserScan::SharedPtr& msg,
                     double min_angle, double max_angle,
                     double& m, double& b)
    {
        double angle_min = msg->angle_min;
        double angle_inc = msg->angle_increment;
        int n = msg->ranges.size();

        double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
        int count = 0;

        for (int i = 0; i < n; i++) {
            double angle = angle_min + i * angle_inc;

            // Filtramos por la ventana de ángulo deseada
            if (angle >= min_angle && angle <= max_angle) {
                double r = msg->ranges[i];
                if (std::isfinite(r) && r > msg->range_min && r < msg->range_max) {
                    // Convertimos a coordenadas cartesianas (x, y) locales del robot
                    double x = r * std::cos(angle);
                    double y = r * std::sin(angle);
                    sum_x += x;
                    sum_y += y;
                    sum_xy += x * y;
                    sum_xx += x * x;
                    count++;
                }
            }
        }

        if (count < 2) return false; // Se necesitan al menos 2 puntos

        double denominator = count * sum_xx - sum_x * sum_x;
        if (std::abs(denominator) < 1e-6) return false; // Evitar división por cero

        // Calculamos la pendiente (m) y la distancia (b)
        m = (count * sum_xy - sum_x * sum_y) / denominator;
        b = (sum_y - m * sum_x) / count;

        return true;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Extraemos la pared izquierda (ángulos aprox entre 30º y 150º)
        bool left_ok = extractWall(msg, M_PI/6.0, 5.0*M_PI/6.0, m_left_, b_left_);
        
        // Extraemos la pared derecha (ángulos aprox entre -150º y -30º)
        bool right_ok = extractWall(msg, -5.0*M_PI/6.0, -M_PI/6.0, m_right_, b_right_);

        if (left_ok && right_ok) {
            measured_data_ = true;
            // Usamos THROTTLE para no inundar la terminal de mensajes
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Left Wall: y=%.2fx+%.2f, Right Wall: y=%.2fx+%.2f", 
                m_left_, b_left_, m_right_, b_right_);
        } else {
            measured_data_ = false;
        }
    }

    void controlLoop()
    {
        if (!measured_data_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for laser data to fit walls...");
            return;
        }

        measured_data_ = false; // Reiniciamos la bandera

        // 'b' representa el corte con el eje Y (la distancia lateral al robot en X=0).
        // Al sumar la distancia izquierda (positiva) y la derecha (negativa) y dividir, 
        // obtenemos el desplazamiento lateral exacto del robot respecto al centro.
        double dl = (b_left_ + b_right_) / 2.0;

        // 'm' es la pendiente. Promediamos las pendientes para hallar la inclinación del pasillo.
        double m_avg = (m_left_ + m_right_) / 2.0;
        
        // Calculamos el error de orientación en radianes. Le cambiamos el signo para mantener 
        // el criterio de control del código original (theta negativo cuando apunta a la derecha).
        double theta = -std::atan(m_avg);

        RCLCPP_INFO(this->get_logger(), "Lateral Distance=%.2f m, Orientation Error=%.2f rad", dl, theta);

        /////////////////// CONTROL CODE ///////////////////
        
        // Coordenadas locales del punto objetivo
        double xpl = dl * std::sin(theta) + std::cos(theta) * look_ahead_distance_;
        double ypl = dl * std::cos(theta) - std::sin(theta) * look_ahead_distance_;

        // Calculamos distancia
        double distance = sqrt(pow(xpl, 2) + pow(ypl, 2));

        // Control proporcional
        double gamma = (2.0 * ypl) / pow(distance, 2);

        // Velocidad de las ruedas
        double wi = max_linear_speed_ * (1 - wheel_base_ * gamma) / wheel_radius_;
        double wd = max_linear_speed_ * (1 + wheel_base_ * gamma) / wheel_radius_;

        // Velocidad linear y angular global
        double linear_velocity = (wi + wd) * wheel_radius_ / 2.0;
        double angular_velocity = (wd - wi) * wheel_radius_ / wheel_base_;

        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear_velocity;
        cmd_vel_msg.angular.z = angular_velocity;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int time_step_;
    double max_linear_speed_;
    double max_angular_speed_;      
    double wheel_base_;
    double wheel_radius_;
    double corridor_width_;
    double look_ahead_distance_;

    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;  

    // Variables de las líneas extraídas
    double m_left_, b_left_;
    double m_right_, b_right_;
    bool measured_data_ = false;
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorNavigationNode>());
    rclcpp::shutdown();
    return 0;
}