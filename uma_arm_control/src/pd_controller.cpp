    #include <rclcpp/rclcpp.hpp>
    #include <sensor_msgs/msg/joint_state.hpp>
    #include <std_msgs/msg/float64_multi_array.hpp>
    #include <geometry_msgs/msg/wrench.hpp>
    #include <algorithm>
    #include <cmath>
    #include <Eigen/Dense>

    class PDControllerNode : public rclcpp::Node
    {
    public:
        PDControllerNode()
            : Node("pd_controller_node"),   
            joint_positions_(Eigen::VectorXd::Zero(2)),
            joint_velocities_(Eigen::VectorXd::Zero(2)),
            desired_joint_accelerations_(Eigen::VectorXd::Zero(2)),
            joint_torques_(Eigen::VectorXd::Zero(2))
        {
            publisher_desired_joint_accelerations_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("desired_joint_accelerations", 1);    
            
            subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 1, std::bind(&PDControllerNode::joint_states_callback, this, std::placeholders::_1));

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1), std::bind(&PDControllerNode::control_loop, this));

            // Initialize PD gains
            Kp_ = Eigen::VectorXd::Constant(2, 100.0);
            Kd_ = Eigen::VectorXd::Constant(2, 20.0);  


        }

    private:
        // Publicadores y suscriptores
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_desired_joint_accelerations_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Variables de estado y ganancias
        Eigen::VectorXd joint_positions_;
        Eigen::VectorXd joint_velocities_;
        Eigen::VectorXd desired_joint_accelerations_;
        Eigen::VectorXd joint_torques_;
        Eigen::VectorXd Kp_;
        Eigen::VectorXd Kd_;

        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            // Assuming the joint names are "joint_1" and "joint_2"
            auto joint1_index = std::find(msg->name.begin(), msg->name.end(), "joint_1") - msg->name.begin();
            auto joint2_index = std::find(msg->name.begin(), msg->name.end(), "joint_2") - msg->name.begin();

            if (static_cast<std::vector<std::string>::size_type>(joint1_index) < msg->name.size() &&
                static_cast<std::vector<std::string>::size_type>(joint2_index) < msg->name.size())
            {
                joint_positions_(0) = msg->position[joint1_index];
                joint_positions_(1) = msg->position[joint2_index];
                joint_velocities_(0) = msg->velocity[joint1_index];
                joint_velocities_(1) = msg->velocity[joint2_index];
            }
        }

        void control_loop()
        {
            // Creamos las posiciones deseadas
            Eigen::VectorXd desired_joint_positions(2);
            desired_joint_positions << 0, M_PI / 4;

            // Calculamos el error de posición y velocidad
            Eigen::VectorXd position_error = desired_joint_positions - joint_positions_;
            Eigen::VectorXd velocity_error = -joint_velocities_;  // La velocidad deseada es cero

            // Calculamos las aceleraciones deseadas usando la ley de control PD
            desired_joint_accelerations_ = Kp_.cwiseProduct(position_error) + Kd_.cwiseProduct(velocity_error);

            // Publicamos las aceleraciones deseadas
            std_msgs::msg::Float64MultiArray msg;
            msg.data.resize(2);
            msg.data[0] = desired_joint_accelerations_[0];
            msg.data[1] = desired_joint_accelerations_[1];
            publisher_desired_joint_accelerations_->publish(msg);
        }

    };

    int main(int argc, char *argv[])
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<PDControllerNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }