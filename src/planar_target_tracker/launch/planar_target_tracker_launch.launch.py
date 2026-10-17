from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    #definir los argumentos
    near_mm = DeclareLaunchArgument(
        'near_mm',
        default_value='300', # Valor por defecto
        description=''
    )
    far_mm = DeclareLaunchArgument(
            'far_mm',
            default_value='1500', # Valor por defecto
            description='Sobrescribe la velocidad del archivo YAML'
        )
    kernel_size = DeclareLaunchArgument(
            'kernel_size',
            default_value='5', # Valor por defecto
            description='Sobrescribe la velocidad del archivo YAML'
        )
    min_area_px = DeclareLaunchArgument(
            'min_area_px',
            default_value='2000', # Valor por defecto
            description='tamaño de la mascara'
        )
    

    #asignar valor de la consola al parametro del yaml
    parametros = {
        'near_mm': LaunchConfiguration('near_mm'),
        'far_mm': LaunchConfiguration('far_mm'),
        'kernel_size': LaunchConfiguration('kernel_size'),
        'min_area_px': LaunchConfiguration('min_area_px')

    }


    # 5. Configura el nodo pasando el nuevo archivo dinámico
    depth_segmenter = Node(
        package='planar_target_tracker',
        executable='depth_segmenter_node',
        name='depth_segmenter_node',
        parameters=parametros, # parametros de configuración
        output='screen'
    )

    pose_estimator = Node(
            package='planar_target_tracker',
            executable='pose_estimator_node',
            name='pose_estimator_node',
            output='screen'
        )

    return LaunchDescription([
        near_mm,
        far_mm,
        kernel_size,
        min_area_px,
        depth_segmenter,
        pose_estimator
    ])
