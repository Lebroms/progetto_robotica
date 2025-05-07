from launch import LaunchDescription 
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory 
import os 

def generate_launch_description(): 

    ld = LaunchDescription() 

    action_client = Node( 
        package="task_1", 
        executable="action_client", 
    ) 

    action_server = Node( 
        package="task_1", 
        executable="action_server" 
    ) 

    ld.add_action(action_client) 
    ld.add_action(action_server) 

    return ld