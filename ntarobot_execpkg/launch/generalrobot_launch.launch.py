# Seccion de librerias de launch
# El siguiente launch realiza la ejecucion completa de los paquetes de control
# al encender el robot, incluyendo los paquetes de control, sensores y localizacion.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

#Paquete para declaracion de parametros de lidar
from launch.substitutions import Command, LaunchConfiguration
# Importe de libreria para publicacion del robo
from launch_ros.parameter_descriptions import ParameterValue

# Liberias para manejo de archivos y rutas de ejecucion
from os.path import join as join_p
from ament_index_python.packages import get_package_share_directory

# Librerias de visualziacion de mensajes y debug
from launch.actions import TimerAction, LogInfo
# importe de librerias utilitarias
import os

# Funcion de ejecucion general.
def generate_launch_description():
    # Primero se realizan los paquetes que identifican las rutas de los archivos de nodos
    # y demas archivos de ejecucion, para esto se divide entre las secciones de cada uno de los
    # paquetes generales.
    # Seccion extra de declaracion de parametros de configuracon de rviz2
    # INICIO DE CONFIGURACION DE DISENHO DE ROBOT
    ntarobot_description_dir = get_package_share_directory("diseno_ntarobot")

    # Creacion del argumento del modelo (donde se posiciona el modelo creado) 
    model_arg = DeclareLaunchArgument(
        name = "model", # Este corresponde al nombre del modelo de robot
        default_value = os.path.join( # Aqui llamamos al archivo .xacro que contiene la descripcion del robot
            ntarobot_description_dir, # En este caso /diseno_ntabot/urdf/ntarobot.urdf.xacro
            "urdf", "ntarobot.urdf.xacro" # Se crea la ruta especifica del modelo
        ),
        # description = "Descripcion del robot nta (urdf)"
        description = "Absolute path to robot URDF file"
    )
    
    # INICIO DE SECCION DE LIDAR
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rp_lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='lidar_link')
    inverted = LaunchConfiguration('inverted', default='true')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    # FIN DE SECCION DE LIDAR

    # SECCION DE DECLARACION DE PARAMETROS DE DISENHO UDRF
    robot_description = ParameterValue(
        Command([
            "xacro ", # El tipo de archivo que se usa para la descripcion
            LaunchConfiguration("model") # Especifica la configuracion del modelo
        ]), value_type = str # Tipo de variable, del parametro
    )

    # SECCION DE DECLARACION DE ARGUMENTOS DE LIDAR
    chh_type = DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar')
        
    port_Serial = DeclareLaunchArgument(
        'serial_port',
        default_value=serial_port,
        description='Specifying usb port to connected lidar')

    baud_Serial = DeclareLaunchArgument(
        'serial_baudrate',
        default_value=serial_baudrate,
        description='Specifying usb port baudrate to connected lidar')
    
    frame_val = DeclareLaunchArgument(
        'frame_id',
        default_value=frame_id,
        description='Specifying frame_id of lidar')

    inv_val = DeclareLaunchArgument(
        'inverted',
        default_value=inverted,
        description='Specifying whether or not to invert scan data')

    angle_Com = DeclareLaunchArgument(
        'angle_compensate',
        default_value=angle_compensate,
        description='Specifying whether  or not to enable angle_compensate of scan data')
    
    scan_Mod = DeclareLaunchArgument(
        'scan_mode',
        default_value=scan_mode,
        description='Specifying scan mode of lidar')
    # FIN DE SECCION DE DECLARACION DE ARGUMENTOS DE LIDAR
    
    # 1. Paquetes de control de robot
    # Paquete de cpp de control de robot
    ntarobot_cppPath = get_package_share_directory("ntarobot_cpp")
    # Paquetes de configracion de driver y odometria de encoders
    params_driver = join_p(
        ntarobot_cppPath,
        'config',
        'motordriver_params.yaml'
    )
    # Configuracion encoders
    params_odom = join_p(
        ntarobot_cppPath,
        'config',
        'odomEnc_params.yaml'
    )

    # 2. Paquetes de control y ejecucion de IMU
    # Paquete de control de imu
    ntarobot_imuPath = get_package_share_directory("bno055")
    # Paquete de configuraciones de imu
    params_imu = join_p(
            ntarobot_imuPath,
            'config',
            'bno055_params.yaml'
        )
    # 3. Paquetes de ejecucion de nodo de localizacion
    ntarobot_localConfig = get_package_share_directory("ntarobot_execpkg")
    # Configuraciones de ejecucion de localizacion para filtro de kalman
    params_local = join_p(
        ntarobot_localConfig,
        'config',
        'ekf.yaml'
    )

    # Ruta de parametros de camara
    # params_camera = join_p(
    #     ntarobot_localConfig,
    #     'config',
    #     'camera_config.yaml'
    # )
    # Seccion de creacion de nodos
    # 1. Nodos de controlador, primero la ejecucion del driver
    controller_node = Node(
        package = "ntarobot_cpp",
        executable = "ntarobot_controller",
        # name = "md49_controller_node", # Opcional
        parameters = [
            params_driver
        ],
        output = "screen"
    )
    # Odometria y parametros de ejecucion
    odometry_node = Node(
        package = "ntarobot_cpp",
        executable = "ntarobot_odom",
        # name = "odom_encoders_node", # Opcional
        parameters = [
            params_odom
        ],
        output = "screen"
    )
    # 2. Nodo de ejecucion de imu
    imu_node = Node(
        package = 'bno055',
        executable = 'bno055',
        parameters = [
            params_imu
        ],
        output = "screen"
    )
    # 3. Nodo de ejecucion de localizacion
    robot_localization = Node(
        package = "robot_localization",
        executable = "ekf_node",
        name = "ekf_filter_node",
        output = "screen",
        parameters = [
            params_local
        ],
        remappings=[(
            "/odometry/filtered",
            "/ntarobot/odom_fusion"
        )]
    ) 

    # Creacion de nodo de lanzamiento de camara
    camera_node = Node(
        package = "usb_cam",
        executable = "usb_cam_node_exe",
        # name = "node_camera",
        # parameters = [
        #     params_camera
        # ],
        # name = "node_camera",
        output = "screen"
    )

    # Nodo para ala ejecucion del lidar
    lidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate}],
            output='screen')
    
    # Nodo para la publicacion del robot
    robot_state_publisher = Node(
        package = "robot_state_publisher", # Corresponde al paquete previamente instalado, del cual llamaremos el nodo
        executable = "robot_state_publisher", # Corresponde al nodo de ejecucion
        parameters = [{
                "robot_description": robot_description
            }] # Funcion y cambio de valor de los parametros
    )

    # Publicacion de la ventana que permite mover las articulaciones para verificar si son correctas
    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui", # Paquete donde se almacena el nodo
        executable = "joint_state_publisher_gui" # Nodo ejecutable
    )
    
    # Ejecucion general del launch
    return LaunchDescription([
        # Disenho general
        model_arg,
        # Lidar
        chh_type,
        port_Serial,
        baud_Serial,
        frame_val,
        inv_val,
        angle_Com,
        scan_Mod,
        # Mensaje de validacion de inicio de ejecucion de launch
        LogInfo(msg = "Starting execution of the general package. Validating connection with motor driver..."),
        # Nodo de control
        controller_node,
        LogInfo(msg = "Driver validation completed successfully. Pausing for 5 seconds before starting the odometry node..."),
        TimerAction(
            period = 5.0,
            actions = [
                odometry_node,
                LogInfo(msg = "Odometry node execution completed successfully. Continuing with the execution of the IMU node (Pausing for 2 seconds)...")
            ]
        ),
        # Ejecucion de nodo de encendido de imu
        TimerAction(
            period = 2.0,
            actions = [
                imu_node,
                LogInfo(msg = "IMU node execution completed successfully. Starting execution of the localization node...")
            ]
        ),
        robot_localization,
        LogInfo(msg = "robot_localization node execution completed successfully. Starting execution of the camera node..."),
        # # Seccion de inicializacion de camara 
        camera_node,
        # TimerAction(
        #     period = 2.0,
        #     actions = [
        #         camera_node,
        #         #LogInfo(msg = "IMU node execution completed successfully. Starting execution of the localization node...")
        #     ]
        # ),
        LogInfo(msg = "camera node execution completed successfully. Starting execution of the Lidar node..."),
        lidar_node,
        LogInfo(msg = "Lidar validation completed successfully. Pausing for 5 seconds before starting the robot state node..."),
        robot_state_publisher,
        # TimerAction(
        #     period = 5.5,
        #     actions = [
        #         robot_state_publisher,
        #         joint_state_publisher_gui,
        #         LogInfo(msg = "robot_state_publisher node execution completed successfully.")
        #     ]
        # ),
        LogInfo(msg = "Execution completed successfully!")
    ])