from launch import LaunchDescription
from launch_ros.actions import Node

import os
from glob import glob

pkg_name = 'arm_controller'

def generate_launch_description():
    ld = LaunchDescription(
    )

    arm_controller = Node(
        package             = pkg_name,
        executable          = 'arm_controller',
        name                = 'arm_controller',
        output             = 'screen',
        #respawn             = 'true',#：該当ノードが終了した場合に、再起動するようにする。
    )

    suction_unit = Node(
        package             = pkg_name,
        executable          = 'suction_unit',
        name                = 'suction_unit',
        output             = 'screen',
        #respawn             = 'true',#：該当ノードが終了した場合に、再起動するようにする。
    )

    ld.add_action(arm_controller)
    ld.add_action(suction_unit)

    return ld