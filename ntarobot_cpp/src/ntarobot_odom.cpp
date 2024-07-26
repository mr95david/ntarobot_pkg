#include "ntarobot_cpp/ntarobot_odom.hpp"
#include <cmath>

using std::placeholders::_1;

OdomEncoders::OdomEncoders(const std::string& name)
: Node(name), count_(0)
{
    // Seccion de importe de parametros desde el archivo de parametros especifico
    // Posibles parametros agregables, nombre_header, nombre_msg_childid
    this->declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("wheel_separation", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("percent_separation", rclcpp::PARAMETER_DOUBLE);

    // Get parameter section
    this->get_parameter<double>("wheel_radius", wheel_radius);
    this->get_parameter<double>("wheel_separation", wheel_separation);
    this->get_parameter<double>("percent_separation", percent_separation);

    // Primero se ejecuta la inicializacion de variables
    // wheel_radius = 0.0754;
    wheel_separation = wheel_separation * percent_separation;
    
    // variables extra 
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    // Inicializacion de tiempo
    prev_time_ = this->get_clock()->now();

    // Inicializacion de odometria
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";

    // Establecer las posiciones y orientaciones
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    // Inicializacion de transformaciones
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_footprint";

    // Inicializacion de publicador
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/ntaRobot/odom_msg", 10);

    // inicializacion de subscriptores
    sub_rawSpeed = this->create_subscription<solver_untils::msg::RawSpeed>(
        "/ntaRobot/raws_msg",
        10,
        std::bind(&OdomEncoders::speedCallBack, this, _1)
    );
}

// Funcion de subscripcion speed obtenido
void OdomEncoders::speedCallBack(const solver_untils::msg::RawSpeed &msg)
{
    // recepcion de valores desde el subscriptor 
    double dp_r = msg.encoder_mr;
    double dp_l = msg.encoder_ml;
    // Recepcion de valores de ruedas
    double fi_right = msg.right_wheel;
    double fi_left = msg.left_wheel;

    // calcilo de velocidad lineal y angular
    double linear = (this->wheel_radius * fi_right + this->wheel_radius * fi_left) / 2;
    double angular = (this->wheel_radius * fi_right - this->wheel_radius * fi_left) / this->wheel_separation;

    // calculo de posicion delta y rotacion
    double d_s = (this->wheel_radius * dp_r + this->wheel_radius * dp_l) / 2;
    double d_theta = (this->wheel_radius * dp_r - this->wheel_radius * dp_l) / this->wheel_separation;

    // actualizacion de valores de posiciones
    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);

    // actualizacion de quaterniones
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    // asignacion de valores
    // Public position from quaternions
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();

    // asignacion de tiempo actual
    odom_msg_.header.stamp = this->get_clock()->now();

    // Set position values
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;

    // Matriz de covarianza para la orientacion desde la odometria de los encoders
    odom_msg_.pose.covariance[0] = 1e-6;
    odom_msg_.pose.covariance[7] = 1e-6;
    odom_msg_.pose.covariance[14] = 1e6;
    odom_msg_.pose.covariance[21] = 1e6;
    odom_msg_.pose.covariance[28] = 1e6;
    odom_msg_.pose.covariance[35] = 1e3;
    
    // Set twist values
    odom_msg_.twist.twist.linear.x = linear;
    odom_msg_.twist.twist.angular.z = angular;

    // Matriz de covarianza de twist
    odom_msg_.twist.covariance[0] = 1e-6;
    odom_msg_.twist.covariance[7] = 1e-6;
    odom_msg_.twist.covariance[14] = 1e6;
    odom_msg_.twist.covariance[21] = 1e6;  
    odom_msg_.twist.covariance[28] = 1e6;
    odom_msg_.twist.covariance[35] = 1e3;

    // Publicacion de odometria
    pub_odom->publish(odom_msg_);
    
    // For transform stamped instance
    //transform_stamped_.transform.translation.x = x_;
    //transform_stamped_.transform.translation.y = y_;
    
    // Set rotation values
    //transform_stamped_.transform.rotation.x = q.x();
    //transform_stamped_.transform.rotation.y = q.y();
    //transform_stamped_.transform.rotation.z = q.z();
    //transform_stamped_.transform.rotation.w = q.w();
    
    // Set header stamp for transform stamped
    //transform_stamped_.header.stamp = this->get_clock()->now();

    // Transformacion 
    //br_.sendTransform(transform_stamped_);

    //RCLCPP_INFO(this->get_logger(), "theta_: %f - x_: %f - y_: %f", theta_, x_, y_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomEncoders>("odom_encoders_node"));
    rclcpp::shutdown();
    return 0;
}
