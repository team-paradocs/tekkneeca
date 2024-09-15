import os
from launch.substitutions import LaunchConfiguration

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rviz',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='urdfFile',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='libFolder',
            default_value=''
        ),
        launch.actions.DeclareLaunchArgument(
            name='dummy',
            default_value=''
        ),
        # Conditionally include the 'visualize.launch.py' if 'rviz' is 'true'
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(                
                os.path.join(get_package_share_directory(
                    'ocs2_mobile_manipulator_ros'), 'launch/include/visualize_kuka.launch.py')
            ),
            condition=launch.conditions.IfCondition(LaunchConfiguration('rviz')),
            launch_arguments={
                'urdfFile': LaunchConfiguration('urdfFile'),
                'rviz': LaunchConfiguration('rviz')
            }.items()
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            prefix= "gnome-terminal -- gdb -ex run --args",
            namespace='',
            # namespace='lbr',
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_dummy_mrt_node',
            name='mobile_manipulator_dummy_mrt_node',
            prefix= "gnome-terminal --",
            namespace='',
            # namespace='lbr',
            condition=launch.conditions.IfCondition(LaunchConfiguration("dummy")),
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                }
            ],
            # remappings=[
            #     ('joint_states', 'joint_states_dummy')
            # ]
        ),
        # launch_ros.actions.Node(
        #     package='ocs2_mobile_manipulator_ros',
        #     executable='mrt_node',
        #     name='mrt_node',
        #     prefix= "gnome-terminal --",
        #     namespace='',
        #     # namespace='lbr',
        #     output='screen',
        #     parameters=[
        #         {
        #             'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
        #         },
        #         {
        #             'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
        #         },
        #         {
        #             'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
        #         }
        #     ],
        #     # remappings=[
        #     #     ('joint_states', 'joint_states_dummy')
        #     # ]
        # ),
        launch_ros.actions.Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_target',
            name='mobile_manipulator_target',
            prefix="",
            # condition=launch.conditions.UnlessCondition(LaunchConfiguration("rviz")),
            # condition=launch.conditions.IfCondition(LaunchConfiguration("rviz")),
            # namespace='lbr',
            namespace='',
            output='screen',
            parameters=[
                {
                    'taskFile': launch.substitutions.LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': launch.substitutions.LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': launch.substitutions.LaunchConfiguration('libFolder')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
