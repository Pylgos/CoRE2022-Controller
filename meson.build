project('godot-ros', 'cpp', default_options : ['cpp_std=c++17'])


sources = [
  'src/register_types.cpp',
  'src/laserscan_visualizer.cpp',
  'src/ros_node.cpp',
  'src/robot_interface.cpp',
  'src/lifecycle_manager.cpp'
]

deps = [
  dependency('godot-cpp'),
  dependency('rclcpp'),
  dependency('sensor_msgs'),
  dependency('std_msgs'),
  dependency('geometry_msgs'),
  dependency('lifecycle_msgs'),
  dependency('std_srvs')
]

lib = library('godot-ros', sources, dependencies: deps, install : true)
