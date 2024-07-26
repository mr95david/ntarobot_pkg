# El siguiente launch corresponde a la prueba de ejecucion de launch por secciones para todas
# las funcionalidades del robot, incluyendo la publicacion de los frames, publicacion de nodos y de mas
# Primero entonces, importamos las librerias necesarias
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction, LogInfo
from launch_ros.actions import Node, PushRosNamespace
# Paquete para declaracion de parametros
from launch.substitutions import Command, LaunchConfiguration
# Paquete para la designacion de parametros de publicacion
from launch_ros.parameter_descriptions import ParameterValue

# Seccion de importe de librerias utilitarias
from os.path import join as join_p
# Libreria de interaccion con paquetes propios
from ament_index_python.packages import get_package_share_directory

# Funcion de ejecucion general.
def generate_launch_description():
    # Designacion de de paquetes utilizados
    ntarobot_description_dir = get_package_share_directory("diseno_ntarobot")
    ntarobot_localConfig = get_package_share_directory("ntarobot_execpkg")
    ntarobot_cppPath = get_package_share_directory("ntarobot_cpp")
    ntarobot_imuPath = get_package_share_directory("bno055")

    # SECCION DE NODOS Y HERRAMIENTAS DE FUNCIONAMIENTO PRINCIPAL DEL ROBOT
    # Seccion de parametros de interaccion con nodos de funcionamiento
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
    # Paquete de configuraciones de imu
    params_imu = join_p(
            ntarobot_imuPath,
            'config',
            'bno055_params.yaml'
        )
    # Configuraciones de ejecucion de localizacion para filtro de kalman
    params_local = join_p(
        ntarobot_localConfig,
        'config',
        'ekf.yaml'
    )
    # Fin de designacion de parametros
    # Inicio de descripcion de nodos de funcionamiento
    controller_node = Node(
        package = "ntarobot_cpp",
        executable = "ntarobot_controller",
        parameters = [
            params_driver
        ],
        output = "screen"
    )
    # Odometria y parametros de ejecucion
    odometry_node = Node(
        package = "ntarobot_cpp",
        executable = "ntarobot_odom",
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
    robot_localization_node = Node(
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
        output = "screen"
    )

    # Grupo de ejecicion de nodos
    grupo_funcionamiento = GroupAction(
        actions = [
            # Primero el nodo controlador
            controller_node,
            # Segundo el nodo de odometria
            odometry_node,
            # Seguido a esto la publicacion de la imu
            imu_node,
            # finalmente el nodo de localizacion
            robot_localization_node,
            # Ejecucion de camara
            camera_node,
        ]
    )
    # FIN DE SECCION DE PUBLICACION DE FUNCIONAMIENTO DE ROBOT

    # Inicio de seccion de funcionamiento de Lidar
    # Definicion de argumentos
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rp_lidar')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='lidar_link')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    
    # SECCION DE DECLARACION DE ARGUMENTOS DE LIDAR
    # Declaracion de parametros para funcionamiento de lidar
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
    # Fin de declaracion de parametros

    # Nodo de ejecucion de lidar
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
    
    # Grupo de ejecucion general de nodos y parametros
    gurpo_funcionamiento_lidar = GroupAction(
        actions = [
            chh_type,
            port_Serial,
            baud_Serial,
            frame_val,
            inv_val,
            angle_Com,
            scan_Mod,
            lidar_node
        ]
    )
    # FIN DE EJECUCION GENERAL DE LIDAR

    # INICIO DE PUBLICACION DE ROBOT STATE PUBLISHER
    model_Arg = DeclareLaunchArgument(
        name = "model",
        default_value = join_p(
            get_package_share_directory("diseno_ntarobot"),
            "urdf",
            "ntarobot.urdf.xacro"
        ),
        description = "Disenho completo del robot"
    )

    # Configuracion para uso de mismo tiempo de procesamiento para cada publicacion
    use_sim_time = LaunchConfiguration('use_sim_time', default = "true")

    # Designacion de paramettros de publicacion
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model")
        ]), value_type = str
    )

    # Nodo de publicacion de states
    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{
            "robot_description": robot_description
        }]
    )
    # Nodo de publicacion de articulaciones, llantas
    # state publisher joint
    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui"
    )
    
    # Grupo de accion de grupo de publicacion de robot
    gurpo_publicacion_robot = GroupAction(
        actions = [
            model_Arg,
            robot_state_publisher
            #joint_state_publisher_gui
        ]
    )
    # FIN DE PUBLICACION DE ROBOT STATE
    
    # Ejecucion general del launch
    return LaunchDescription([
        LogInfo(msg = "Inicio de ejecucion de launch general!"),
        grupo_funcionamiento,
        LogInfo(msg = "Inicio de funcionamiento de launch de lidar"),
        gurpo_funcionamiento_lidar,
        LogInfo(msg = "Inicio publicacion de robot state robot"),
        gurpo_publicacion_robot,
        LogInfo(msg = "Ejecucion completa de nodo realizada correctamente")
    ])
