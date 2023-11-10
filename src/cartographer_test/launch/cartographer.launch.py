import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros. substitutions import FindPackageShare

def generate_launch_description () :
    # 定位功能包
    pkg_share = FindPackageShare(package= 'cartographer_test' ).find( 'cartographer_test' )
    
    #配置節點啟動訊息
    use_sim_time = LaunchConfiguration( 'use_sim_time' , default = 'true' )
    # 地圖的分辨率
    resolution = LaunchConfiguration( 'resolution' , default = '0.05' )
    # 地圖的發布週期
    publish_period_sec = LaunchConfiguration( 'publish_period_sec' , default = '1.0' )
    # 設定文件夾路徑
    configuration_directory = LaunchConfiguration( 'configuration_directory' , default = os.path.join(pkg_share, 'config' ) )
    # 設定檔
    configuration_basename = LaunchConfiguration( 'configuration_basename' , default = 'cartographer_2d.lua' )

    #聲明節點
    cartographer_node = Node(
        package= 'cartographer_ros' ,
        executable= 'cartographer_node' ,
        name= 'cartographer_node' ,
        output= 'screen' ,
        parameters=[{ 'use_sim_time' : use_sim_time}],
        arguments=[ '-configuration_directory' , configuration_directory,
                   '-configuration_basename' , configuration_basename],
        remappings=[('/odom','/diff_cont/odom')])

    cartographer_occupancy_grid_node = Node(
        package= 'cartographer_ros' ,
        executable= 'cartographer_occupancy_grid_node' ,
        name= 'cartographer_occupancy_grid_node' ,
        output= 'screen' ,
        parameters=[{ 'use_sim_time' : use_sim_time}],
        arguments=[ '-resolution' , resolution, '-publish_period_sec' , publish_period_sec])

    #啟動文件
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld