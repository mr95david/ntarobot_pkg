#ifndef NTAROBOT_CONTROLLER_HPP
#define NTAROBOT_CONTROLLER_HPP

// Include librerias
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "ntarobot_cpp/md49_controller.hpp"
// Include librerias for matrix calcule
#include <Eigen/Core>
#include "solver_untils/msg/raw_speed.hpp"

class MotorController : public rclcpp::Node
{
public:
    // el parametro de entrada de la clase es el nombre de la misma,
    MotorController(const std::string& name);
    ~MotorController();

private:
    // Import motor driver functions
    std::shared_ptr<MotorDriverCard> motorDriver_ = std::make_shared<MotorDriverCard>();

    // Funcion timer call back
    void timer_callback();
    void timer_enc_callback();
    void timer_state_callback();

    void velCallBack(const geometry_msgs::msg::Twist &msg);

    // Section for subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subs_vel_;

    //publisher creado: Forma numero 1 para crear un publicador
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    //Publicador de informacion de encoders
    rclcpp::Publisher<solver_untils::msg::RawSpeed>::SharedPtr publisher_encoder_;
    
    //creador de timer
    // Timer for validation of connection serial
    rclcpp::TimerBase::SharedPtr timer_enc;
    // Timer for state robot current
    rclcpp::TimerBase::SharedPtr timer_state;
    // Timer for speed change
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Seccion de variables de instancias
    size_t count_;
    // String instances
    std::string port_name_;
    // Integer instances
    int serial_baud_;
    int mode_p_;
    int mode_aceleration_;
    int speed_value_r_;
    int speed_value_l_;
    // Boolean instances
    bool mode_regulator_;
    bool mode_timeout_;
    // Double instances
    double wheel_radius_;
    double wheel_separation_;
    double percent_separation_;

    // check last values
    int last_speed_value_r_;
    int last_speed_value_l_;

    // Value validation of list 
    int listR_values[5];
    int listL_values[5];

    // Section for matrix calcule
    Eigen::Matrix2d speed_conversion_;
    // conversion values
    double value_w_r;
    double value_w_i;

    // prev time and prev encoder
    int last_encoder_r;
    int last_encoder_l; 
    rclcpp::Time prev_time_;
};

#endif