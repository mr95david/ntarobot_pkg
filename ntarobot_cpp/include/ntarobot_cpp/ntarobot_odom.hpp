#ifndef NTAROBOT_ODOM_HPP
#define NTAROBOT_ODOM_HPP

// Include librerias
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include "solver_untils/msg/raw_speed.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomEncoders : public rclcpp::Node
{
public:
    OdomEncoders(const std::string& name);
    //~OdomEncoders();
private:
    // TODO: Crear una funcion que inicialice los valores de odometria
    // Function for sub speed
    void speedCallBack(const solver_untils::msg::RawSpeed &msg);
    // Creacion de publicadores
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    // Seccion de subscriptores
    rclcpp::Subscription<solver_untils::msg::RawSpeed>::SharedPtr sub_rawSpeed;

    // Variable generica de conteo de ejecucion
    size_t count_;
    /* Parametros de inicio del nodo de ejecucion de calculo de node */
    // Parametros de medidas de robot
    double wheel_radius;
    double wheel_separation;
    double percent_separation;

    // Parametros de odometria
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
    //std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
    tf2_ros::TransformBroadcaster br_{this};

    // Parametros de tiempo
    rclcpp::Time prev_time_;

    // Parametros de posicion
    double x_;
    double y_;
    double theta_;
};

#endif