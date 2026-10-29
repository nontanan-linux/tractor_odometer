import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # ดึงพาทของแพ็กเกจ tractor_description ที่เก็บไฟล์ xacro ไว้
    desc_pkg_share = get_package_share_directory('tractor_description')
    xacro_file = os.path.join(desc_pkg_share, 'urdf', 'tractor_trailer.xacro')

    # แปลงไฟล์ xacro เป็น URDF XML
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # Node สำหรับ publish TF ของหุ่นยนต์
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Node สำหรับคำนวณและ publish joint states & TF จาก Odometer
    nmb_odometer_node = Node(
        package='tractor_description',
        executable='nmb_odometer_node',
        name='nmb_odometer_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Node สำหรับเปิด RViz2 (ปิดไว้เพราะ Autoware เปิดให้อยู่แล้ว)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen'
    # )

    return LaunchDescription([
        robot_state_publisher_node,
        nmb_odometer_node
    ])
