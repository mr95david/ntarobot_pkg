# Librerias utilitarias
import os
# Modulo para extraer archivos del directorio de share
from ament_index_python.packages import get_package_share_directory
# Importe de librerias de uso de launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Seccion de obtencion de paquetes de share
    ntarobot_nav = get_package_share_directory('ntarobot_navigation')
    nav2_bringup = get_package_share_directory("nav2_bringup")
    
    # Seccion de declarracion de configuraciones
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Seccion de importe de rutas de archivos usados
    # Se incluye la ruta de launch de nav2 bringup
    launch_dir = os.path.join(
        nav2_bringup,
        'launch'
    )
    # Ruta de mapa usado
    map_file = os.path.join(
        ntarobot_nav,
        'maps',
        'ctXVIII.yaml'
    )
    # Ruta de configuracion de rviz
    rviz_dir = os.path.join(
        nav2_bringup,
        'rviz',
        'nav2_default_view.rviz'
    )

    # Se incluye la ruta de la configuracion de los parametros para el lanzamiento del robot real
    params_dir = os.path.join(
        ntarobot_nav,
        'params',
        'ntarobot_nav_params_real.yaml'
    )

    # Seccion de launch para launch prehechos, con el fin de agilizar un poco el proceso de ejecucion del robot
    launch_rviz = os.path.join(launch_dir, 'rviz_launch.py')

    # Seccion de declaraion de argumentos
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value = rviz_dir,
        description = 'Ruta completa de configuracion de rviz'
    )
    # Seccion de definicion de lista de lifecycle nodes
    lifecycle_nodes = ['map_server', 
                       'amcl',
                       'planner_server',
                       'controller_server',
                       'recoveries_server',
                       'bt_navigator'
        ]

    # Lista de remappings dados para la utilizacion del paquete
    # Es necesario agregar remapings en caso de necesitar cambios de los nodos
    #remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
    
    # Seccion de creacion de launch
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_rviz),
        #condition = IfCondition(use_rviz),
        launch_arguments={'rviz_config': rviz_config_file}.items()
    )

    # Creacion de nodos de ejecucion para el paquete de nav2
    # Nodo de ejecucion de map_server
    node_map =Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_file}]
    )
    # Nodo de lanzamiento de configuracion de amcl
    node_amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_dir]
    )
    # Nodo de lanzamiento de controlador
    node_controller = Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters= [params_dir],
                #remappings= remappings
    )
    # Nodo de lanzamiento de paneador
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_dir]
    )
    # Nodo de recoveries
    node_recoveries = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[params_dir],
            output='screen')
    # Nodo de bt
    node_bt = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_dir])

    # Nodo de automatizacion de ejecucion de nodo
    node_lifecycle_ = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Seccion de llamado de configuracion
        declare_rviz_config_file_cmd,
        # Seccion de ejecucion de launch
        rviz_cmd,
        # Seccion de ejecucion de nodos
        node_map,
        node_amcl,
        node_controller,
        node_planner,
        node_recoveries,
        node_bt,
        node_lifecycle_,
    ])