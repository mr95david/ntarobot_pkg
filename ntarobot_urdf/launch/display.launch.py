# EN UN PRIMER PLANO CREAMOS EL LAUNCH PARA EL ROBOT DEL CURSO
# Paquete para hacer un pequeño archivo de launch para toda la descripcion del robot:
# Para esto primero importamos los paquetes necesarios para crear el launch
from launch import LaunchDescription # Esta paquete apoya en el launch de la descripcion del robot
from launch_ros.actions import Node # Esta libreria nos permite llamar una libreria en un paquete especifico
from launch.actions import DeclareLaunchArgument # Esta libreria nos permite acceder a la declaracion de argumentos
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
# Seccion de librerias utilitarias
import os
# Librerias para manipulacion de archivos, referentes de ros
from ament_index_python.packages import get_package_share_directory

# Funcion para generar el launch
def generate_launch_description():
    # declaracion de paquete de descripcion
    ntarobot_description_dir = get_package_share_directory("ntarobot_urdf")

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
    print(os.path.join( # Aqui llamamos al archivo .xacro que contiene la descripcion del robot
            ntarobot_description_dir, # En este caso /diseno_ntabot/urdf/ntarobot.urdf.xacro
            "urdf", "ntarobot.urdf.xacro" # Se crea la ruta especifica del modelo
        ))

    # Parametro para establecer el tipo de urdf que se esta usando
    robot_description = ParameterValue(
        Command([
            "xacro ", # El tipo de archivo que se usa para la descripcion
            LaunchConfiguration("model") # Especifica la configuracion del modelo
        ]), value_type = str # Tipo de variable, del parametro
    )

    # Variable para almacenar el nodo de publicacion de estado de robot
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

    # Nodo de ejecucion de rviz, visualizacion de modulo de disenho de robot
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(ntarobot_description_dir, "rviz", "display.rviz")],
    )

    # Este retorna un LaunchDescription object que tiene todos los datos de la descripcion que se va a lanzar
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        #rviz_node
    ])