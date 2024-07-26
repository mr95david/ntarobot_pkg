#include "ntarobot_cpp/md49_controller.hpp"

// Inicialization class
void MotorDriverCard::serialPort_open(
        const char* serial,
        int baudRate
    ){
    // Actual baudrate read
    int actual_baudRate;

    // BaudRate and configBaud configuration
    switch (baudRate)
    {
        // First case baud rate
        case 9600:
            actual_baudRate = B9600;
            break;
        // Faster configuration
        case 38400:
            actual_baudRate = B38400;
            break;
        // Default configuration
        default:
            actual_baudRate = B9600;
            break;
    }

    /* try and validation for open specific port */
    serial_port = open(
        serial,
        O_RDWR | O_NOCTTY | O_NDELAY
    );

    // Validation of connection
    if (serial_port < 0){
        std::cerr << "Error al abrir el puerto serie" << std::endl;
    }

    // Validation of termios and tty
    struct termios tty;
    // Identificacion de termios
    if(tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "driverController: Serial Error!" << std::endl;
    }

    // Check all validations for comunication
    // TODO: Describir especificamente cada parte del funcionamiento del codigo
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, actual_baudRate);
    cfsetospeed(&tty, actual_baudRate);

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error al establecer atributos del puerto serial!" << std::endl;
    }
    std::cerr << "Puerto serial configurado correctamente." << std::endl;
}

// Validation of state of connection
bool MotorDriverCard::is_serial_port_open(int serialPort) {
    struct termios tty;
    return tcgetattr(serialPort, &tty) == 0;
}

void MotorDriverCard::check_serial_port(int serialPort) {
    if (!is_serial_port_open(serialPort)) {
        std::cerr << "La conexión con el puerto serial se ha perdido." << std::endl;
    }
}

// Function for close serial port
void MotorDriverCard::closePortSerial(){
    // TODO: Make validation for close
    if (serial_port > 0){
        close(serial_port);
    }
}

// Lectura de voltaje desde la placa
int MotorDriverCard::get_volts()
{
    unsigned char read_buf [1];
    unsigned char msg[2] = {0x00,0x26};
    write(serial_port, msg, sizeof(msg));
    int n = read(serial_port, &read_buf, sizeof(read_buf));
    return read_buf[0];
}

// Get current from motors
int MotorDriverCard::get_current_l()
{
    unsigned char read_buf [1];
    unsigned char msg[2] = {0x00,0x27};
    write(serial_port, msg, sizeof(msg));
    int n = read(serial_port, &read_buf, sizeof(read_buf));
    return read_buf[0];
}

int MotorDriverCard::get_current_r()
{
    unsigned char read_buf [1];
    unsigned char msg[2] = {0x00,0x28};
    write(serial_port, msg, sizeof(msg));
    int n = read(serial_port, &read_buf, sizeof(read_buf));
    return read_buf[0];
}

// Function for get errors from driver
int MotorDriverCard::get_error()
{
    unsigned char read_buf [1];
    unsigned char msg[2] = {0x00,0x2D};
    write(serial_port, msg, sizeof(msg));
    int n = read(serial_port, &read_buf, sizeof(read_buf));
    return read_buf[0];
}

/* Get each encoder */
int MotorDriverCard::get_encoderR()
{
    unsigned char msg[2] = {0x00,0x23};
    unsigned char enc_buf [4];
    write(serial_port, msg, sizeof(msg));
    int i = read(serial_port, &enc_buf, sizeof(enc_buf));
    // filter_encoder(enc_buf,last_enc_l);
    printf("[%d][%d][%d][%d] \n",enc_buf[0],enc_buf[1],enc_buf[2],enc_buf[3]);
    int result = (enc_buf[0] <<24) + (enc_buf[1] << 16) + (enc_buf[2] << 8) + enc_buf[3];	// Put encoder values together
    return result;
}

int MotorDriverCard::get_encoderL()
{
    unsigned char msg[2] = {0x00,0x24};
    unsigned char enc_buf [4];
    write(serial_port, msg, sizeof(msg));
    int i = read(serial_port, &enc_buf, sizeof(enc_buf));
    //filter_encoder(enc_buf,last_enc_r);
    printf("[%d][%d][%d][%d] \n",enc_buf[0],enc_buf[1],enc_buf[2],enc_buf[3]);
    int result = (enc_buf[0] <<24) + (enc_buf[1] << 16) + (enc_buf[2] << 8) + enc_buf[3];	// Put encoder values together
    return result;
}

/* Section for read encoders values*/
void MotorDriverCard::get_encoders()
{
    unsigned char msg[2] = {0x00,0x25};
    unsigned char enc_buf [8] = {0,0,0,0,0,0,0,0};
    write(serial_port, msg, sizeof(msg));
    // Suspencion en microsegundos de la ejecucion
    usleep(3000);
    int i = read(serial_port, &enc_buf, sizeof(enc_buf));
    encoder_r = (enc_buf[0] <<24) + (enc_buf[1] << 16) + (enc_buf[2] << 8) + enc_buf[3];	// Put encoder values together
    encoder_l = (enc_buf[4] <<24) + (enc_buf[5] << 16) + (enc_buf[6] << 8) + enc_buf[7];	// Put encoder values together
}

/* Section for initial configurations */
void MotorDriverCard::set_acceleration(int val_acc)
{
    unsigned char msg[3] = {0x00,0x33,static_cast<unsigned char>(val_acc)};
    write(serial_port, msg, sizeof(msg));
}

void MotorDriverCard::set_mode(int mode)
{
    unsigned char msg[3] = {0x00,0x34,static_cast<unsigned char>(mode)};
    write(serial_port, msg, sizeof(msg));
}

void MotorDriverCard::reset_encoders()
{
    unsigned char msg[3] = {0x00,0x35};
    write(serial_port, msg, sizeof(msg));
}

/* Configuration of regulator */
void MotorDriverCard::disable_regulator()
{
    unsigned char msg[2] = {0x00,0x36};
    write(serial_port, msg, sizeof(msg));
}

void MotorDriverCard::enable_regulator()
{
    unsigned char msg[2] = {0x00,0x37};
    write(serial_port, msg, sizeof(msg));
}

/* Configuration of timeout */
void MotorDriverCard::disable_timeout()
{
    unsigned char msg[2] = {0x00,0x38};
    write(serial_port, msg, sizeof(msg));
}

void MotorDriverCard::enable_timeout()
{
    unsigned char msg[2] = {0x00,0x39};
    write(serial_port, msg, sizeof(msg));
}

/* Section for speed values*/
void MotorDriverCard::set_speed_l(int speed_l)
{
    // int act_speed_l = speed_l;
    unsigned char msg[3] = {0x00,0x31,static_cast<unsigned char>(speed_l)};
    write(serial_port, msg, sizeof(msg));
}

void MotorDriverCard::set_speed_r(int speed_r)
{
    // int act_speed_r = speed_r;
    unsigned char msg[3] = {0x00,0x32,static_cast<unsigned char>(speed_r)};
    write(serial_port, msg, sizeof(msg));
}

/* Functions for get values from motordriver object */
int MotorDriverCard::return_encR(){
    return encoder_r;
}

int MotorDriverCard::return_encL(){
    return encoder_l;
}

int MotorDriverCard::get_port(){
    return serial_port;
}