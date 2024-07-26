#include "ntarobot_cpp/ntarobot_controller.hpp"
#include <Eigen/Geometry>

// Librerias basicas para funciones varias
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
// include specific function 
using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

// Inicio de clase
MotorController::MotorController(const std::string& name)
: Node(name), count_(0)
{
    // Seccion de declaracion multiple de parametros
    this->declare_parameter("port_name", rclcpp::PARAMETER_STRING);
    this->declare_parameter("serial_baud", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("mode_p", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("mode_aceleration", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("mode_regulator", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("mode_timeout", rclcpp::PARAMETER_BOOL);
    this->declare_parameter("speed_value_r", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("speed_value_l", rclcpp::PARAMETER_INTEGER);
    this->declare_parameter("wheel_radius", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("wheel_separation", rclcpp::PARAMETER_DOUBLE);
    // Seccion de prueba de input de separation
    this->declare_parameter("percent_separation", rclcpp::PARAMETER_DOUBLE);

    // Get parameter section
    this->get_parameter<std::string>("port_name", port_name_);
    this->get_parameter<int>("serial_baud", serial_baud_);
    this->get_parameter<int>("mode_p", mode_p_);
    this->get_parameter<int>("mode_aceleration", mode_aceleration_);
    this->get_parameter<int>("speed_value_r", speed_value_r_);
    this->get_parameter<int>("speed_value_l", speed_value_l_);
    this->get_parameter<bool>("mode_regulator", mode_regulator_);
    this->get_parameter<bool>("mode_timeout", mode_timeout_);
    this->get_parameter<double>("wheel_radius", wheel_radius_);
    this->get_parameter<double>("wheel_separation", wheel_separation_);
    // Seccion de prueba de input de separation
    this->get_parameter<double>("percent_separation", percent_separation_);

    // Last speed register
    this->last_speed_value_r_ = 128;
    this->last_speed_value_l_ = 128;

    // Nuevo valor de percent separation para rectificar valor de velocidad angular
    wheel_separation_ = wheel_separation_ * percent_separation_;

    // Asignacion de valores de listas de validacion
    int listR_values[5] = {0, 0, 0, 0, 0};
    int listL_values[5] = {0, 0, 0, 0, 0};

    /* In this section we make respective configurations for driver connection */
    // Try connection for serial port
    motorDriver_ -> serialPort_open(
        port_name_.c_str(),
        serial_baud_
    );
    // Step for connect validation
    sleep(1);
    motorDriver_ -> set_acceleration(mode_aceleration_);
    // Step for connect validation
    sleep(1);
    motorDriver_ -> set_mode(mode_p_);
    // Step for connect validation
    sleep(1);
    motorDriver_ -> enable_regulator();
    // Step for connect validation
    sleep(1);
    // motorDriver_ -> disable_timeout();
    // Step for connect validation
    // sleep(1);
    motorDriver_ -> reset_encoders();
    // Step for connect validation
    sleep(1);
    /* Final configuration */

    // Validate variables
    speed_conversion_ << wheel_radius_/2, wheel_radius_/2, wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;
    // assing variables
    value_w_i = 10.273;
    value_w_r = 10.242;

    // Get initial time and check variables state
    last_encoder_r = 0;
    last_encoder_l = 0;
    prev_time_ = this->get_clock()->now();

    // Get values from cmd_vel subscription
    subs_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        // "/ntaRobot/cmd_vel",
        "/cmd_vel",
        10,
        std::bind(&MotorController::velCallBack, this, _1)
    );

    // RCLCPP_INFO(this->get_logger(), "Integer parameter value: %d", serial_baud_);

    /* Publisher section */
    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    publisher_encoder_ = this->create_publisher<solver_untils::msg::RawSpeed>("/ntaRobot/raws_msg", 10);

    /* Timer section */
    // Timer for connection validate
    timer_enc = this->create_wall_timer(
        50ms, std::bind(&MotorController::timer_enc_callback, this)
    );

    // Timer for connection validate
    timer_state = this->create_wall_timer(
        100ms, std::bind(&MotorController::timer_state_callback, this)
    );

    // Timer for speed controller
    timer_ = this->create_wall_timer(
        50ms, std::bind(&MotorController::timer_callback, this)
    );

}

// Function for close motor controller
MotorController::~MotorController(){
    motorDriver_->closePortSerial();
}

// Se Crea una funcion para validar el estado de conexion del puerto y la tarjeta
void MotorController::timer_enc_callback(){
    /* Manera numero 1 para obtener el valor de los encoders */
    // Update encoder values
    motorDriver_->get_encoders();

    // assing values from encoders
    int val_l = motorDriver_->return_encR();
    int val_r = motorDriver_->return_encL();

    // mean values of validation
    int sum_dr = 0;
    int sum_dl = 0;

    // ciclo de validacion de valores
    for (int i = 0; i < 5 - 1; i++){
        // acumulacion de valores
        sum_dr = sum_dr + (listR_values[i+1] - listR_values[i]);
        sum_dl = sum_dl + (listL_values[i+1] - listL_values[i]);

        // Relocalizacion de posiciones
        listR_values[i] = listR_values[i+1];
        listL_values[i] = listL_values[i+1];
    }

    double mean_dr = sum_dr / 4;
    double mean_dl = sum_dl / 4;

    // Validacion de diferencia y asignacion de valores
    // Validacion der
    if (val_r - listR_values[4] > 70 || val_r - listR_values[4] < -70){
        listR_values[4] = listR_values[3] + (int)floor(mean_dr + 0.5);
        val_r = listR_values[4];
    } else{
        listR_values[4] = val_r;
    }
    // validacion izq
    if (val_l - listL_values[4] > 70 || val_l - listL_values[4] < -70){
        listL_values[4] = listL_values[3] + (int)floor(mean_dl + 0.5);
        val_l = listL_values[4];
    } else{
        listL_values[4] = val_l;
    }

    // listR_values[4] = val_r;
    // listL_values[4] = val_l;    

    // RCLCPP_INFO(this->get_logger(), "Valor Obtenido de encoders: r = %d, i = %d == diff media: r = %f, i = %f", val_r, val_l, mean_dr, mean_dl);

    // get time of check speed
    rclcpp::Time act_time = this->get_clock()->now();

    // calculo de diferencias
    double dp_r = ((double)val_r - (double)last_encoder_r) * (2 * M_PI / 980);
    double dp_l = ((double)val_l - (double)last_encoder_l) * (2 * M_PI / 980);

    // calculo de diferencia de tiempo
    rclcpp::Duration dt = act_time - prev_time_;

    // update initial values
    last_encoder_r = val_r;
    last_encoder_l = val_l;
    prev_time_ = act_time;

    // calculo de phi
    double fi_right = dp_r/(dt.seconds());
    double fi_left = dp_l/(dt.seconds());

    auto message = solver_untils::msg::RawSpeed();

    message.encoder_mr  = dp_r;
    message.encoder_ml  = dp_l;
    message.right_wheel = fi_right;
    message.left_wheel  = fi_left;

    publisher_encoder_->publish(message);

    // calculo de velocidad angular y linear obtenida
    // double linear = (wheel_radius_*fi_right + wheel_radius_*fi_left) / 2;
    // double angular = (wheel_radius_*fi_right - wheel_radius_*fi_left) / wheel_separation_;

    //RCLCPP_INFO(this->get_logger(), "Values from encoders: valor_actual = %f, valor_anterior = %f", linear, angular);

    // int val_r = motorDriver_->get_current_r();
    // int val_l = motorDriver_->get_current_l();

    // int val_r = motorDriver_->get_encoderR();
    // int val_l = motorDriver_->get_encoderL();

    //RCLCPP_INFO(this->get_logger(), "Valor de corriente de cada motor Corriente 1= %d, corriente 2 = %d, voltaje = %d", val_r, val_l, val_volts);

    // RCLCPP_INFO(this->get_logger(), "Values from encoders: encoder_r = %f, encoder_l = %f", dp_r, dp_l);
}

void MotorController::timer_state_callback(){
    // Check state of connection and reconnect, TODO: need check state of configurations
    int val_serialPort = motorDriver_->get_port();
    // Condition of state connection
    if (!motorDriver_->is_serial_port_open(val_serialPort)) {
        RCLCPP_INFO(this->get_logger(), "Conexion perdida, intento de reconeccion");
        motorDriver_->closePortSerial();
        motorDriver_ -> serialPort_open(
            port_name_.c_str(),
            serial_baud_
        );
        if (motorDriver_->get_port() >= 0){
            RCLCPP_INFO(this->get_logger(), "Conexion recuperada!");
        }
    }

    // Update encoder values
    // motorDriver_->get_encoders();

    // assing values from encoders
    // int val_volts = motorDriver_->get_volts();
    // int val_r = motorDriver_->get_current_r();
    // int val_l = motorDriver_->get_current_l();
    //int val_error = motorDriver_->get_error();
    
    //RCLCPP_INFO(this->get_logger(), "Se ha generado un error de tal manera = %d", val_error);
    //RCLCPP_INFO(this->get_logger(), "Valor de corriente de cada motor Corriente 1= %d, corriente 2 = %d, voltaje = %d", val_r, val_l, val_volts);
}

// Se crea una funcion que usa como puntero un valor obtenido de un topico especifico
void MotorController::velCallBack(const geometry_msgs::msg::Twist &msg)
{
    // Get lineal velocity and angular velocity
    // TODO: Pasar directamente al vector
    // double velLineal = msg.twist.linear.x;
    // double velAngular = msg.twist.angular.z;
    double velLineal = msg.linear.x;
    double velAngular = msg.angular.z;

    // Vector 2d
    Eigen::Vector2d robot_speed(velLineal, velAngular);
    // Calculo de velocidad
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

    // Calculo de conversion de valor, para corrreccion de salida
    // Inverti los valores
    int vel_rot_wi = static_cast<int>(std::round(wheel_speed.coeff(1) * this->value_w_r + 128));
    int vel_rot_wd = static_cast<int>(std::round(wheel_speed.coeff(0) * this->value_w_i + 128));

        
    if (value_w_r != this->speed_value_r_){
        this->speed_value_r_ = vel_rot_wd;
    }
    if (vel_rot_wi != this->speed_value_l_){
        this->speed_value_l_ = vel_rot_wi;
    }
}

void MotorController::timer_callback()
{
    /* Envio de nueva velocidad cada 50 milisegundos*/
    if (this->last_speed_value_r_ != this->speed_value_r_){
        motorDriver_->set_speed_r(this->speed_value_r_);
        this->last_speed_value_r_ = this->speed_value_r_;
    }

    if (this->last_speed_value_l_ != this->speed_value_l_){
        motorDriver_->set_speed_l(this->speed_value_l_);
        this->last_speed_value_l_ = this->speed_value_l_;
    }
    // motorDriver_->set_speed_r(this->speed_value_r_);
    // motorDriver_->set_speed_l(this->speed_value_l_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>("md49_controller_node"));
    rclcpp::shutdown();
    return 0;
}