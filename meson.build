project('godot-ros', 'cpp', default_options : ['cpp_std=c++17'])


sources = [
  'src/ros_node.cpp',
  'src/ros_executor.cpp',
  'src/test_node.cpp',
  'src/laserscan_visualizer.cpp',
  'src/register_types.cpp'
]

deps = [
  dependency('godot-cpp'),
  dependency('rclcpp'),
  dependency('sensor_msgs')
]

lib = library('godot-ros', sources, dependencies: deps, install : true)
