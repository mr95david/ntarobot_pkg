# Este documento realiza el lanzamiento de los paquetes de control iniciales
from launch import LaunchDescription
from launch_ros.actions import Node

# Librerias para obtener los parametros desde rutas especificas
from ament_index_python.packages import get_package_share_directory
# Libreria de actions del launch para la interaccion de los nodos
from launch.actions import LogInfo, TimerAction
import os
# import time


# Se desarrolla el proceso por el cual se ejecutan los nodos en cuestion
def generate_launch_description():
    # Primero se crea el punto especifico del directorio en la carpeta compartida
    ntarobot_cppPath = get_package_share_directory("ntarobot_cpp")

    # Se extrae la ruta de los archivos de parametros pertenecientes a los nodos que se ejecutaran.
    params_driver = os.path.join(
        ntarobot_cppPath,
        'config',
        'motordriver_params.yaml'
    )

    # Se extrae tambien la ruta de los archivos de parametros pertenecientes al nodo de odometria
    params_odom = os.path.join(
        ntarobot_cppPath,
        'config',
        'odomEnc_params.yaml'
    )

    # Se Crea la lista de ejecucion de los nodos
    controller_node = Node(
        package = "ntarobot_cpp",
        executable = "ntarobot_controller",
        # name = "md49_controller_node", # Opcional
        output = "screen",
        parameters = [
            params_driver
        ]
    )

    # Se crea tambien el nodo de ejecucion de la odometria
    odometry_node = Node(
        package = "ntarobot_cpp",
        executable = "ntarobot_odom",
        # name = "odom_encoders_node", # Opcional
        output = "screen",
        parameters = [
            params_odom
        ]
    )

    # Finalmente se ejecuta la descripcion de launch creado
    return LaunchDescription([
        LogInfo(msg = "Lanzamiento de nodo de control, conexion con driver de motor..."),
        controller_node,
        LogInfo(msg = "Lanzamiento de nodo realizado correctamente."),
        TimerAction(
            period = 5.0, # Periodo de espera para la ejecucion del nodo de odometria
            actions = [
                LogInfo(msg = "Validacion de 5 segundos para la ejecucion del nodo de odometria..."),
                LogInfo(msg = "Inicio de ejecucion de odometria"),
                odometry_node,
                LogInfo(msg = "Nodo ejecutado correctamente")
            ]
        )
    ])
