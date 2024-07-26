#ifndef MD49_CONTROLLER_HPP
#define MD49_CONTROLLER_HPP

// Include libraries for process and communication with motor driver
/* Esta libreria es para intervenir en comandos especificos de manipulacion de archivos */
#include "fcntl.h"
/* Libreria para el control de terminales POSIX, necesaria para la configuracion de uso de terminales */
#include "termios.h"
/* Otra libreria especifica para el control de archivos */
#include "unistd.h"
/* Libreria para gestion de archivos y la memoria */
#include "stdlib.h"
#include <iostream>

// Seccion de creacion de objeto para control de conexiones
class MotorDriverCard
{
    // Methodos de clase para la inicializacion de la clase cpp
    public:
        // metodo para la apertura del puerto serial
        void serialPort_open(
            const char* serial,
            int baudRate = 9600
        );
        // Function for validate state of connection
        bool is_serial_port_open(int serialPort);
        void check_serial_port(int serialPort);

        // Function for get volts from board
        int get_volts();
        int get_current_r();
        int get_current_l();
        int get_error();

        // Functions for get values from encoders
        // Get only for one encoder
        int get_encoderR();
        int get_encoderL();
        // get both encoders
        void get_encoders();
        int return_encR();
        int return_encL();

        // Functions for set speed 
        void set_speed_l(int speed_l);
        void set_speed_r(int speed_r);

        // configurations from motor driver
        // Set initial configurations 
        void set_acceleration(int val_acc);
        void set_mode(int mode);
        void reset_encoders();

        // This is for regulator of speed from motor driver
        void enable_regulator();
        void disable_regulator();

        // This is for time to response from driver
        void enable_timeout();
        void disable_timeout();

        // Cierre de puerto serial
        void closePortSerial();

        int get_port();

    // Variables de instancia para manipulacion de datos de la clase
    private:
        // Puerto serial utilizado
        int serial_port;
        //char msg_connection;
        // instance for encoders value
        int encoder_r;
        int encoder_l;
};

#endif