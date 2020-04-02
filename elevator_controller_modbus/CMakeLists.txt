cmake_minimum_required(VERSION 2.8.3)
project(elevator_controller_modbus)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    self_test
    diagnostic_updater
    std_srvs
    robotnik_msgs
    robotnik_elevator_component
    robotnik_elevator_interface_msgs
    rcomponent
    )

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    self_test
    diagnostic_updater
    std_srvs
    robotnik_msgs
    robotnik_elevator_component
    robotnik_elevator_interface_msgs
    rcomponent
#  DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(elevator_controller_interface_node src/elevator_controller_interface_node.cpp)

add_dependencies(elevator_controller_interface_node robotnik_msgs_generate_messages_cpp)

target_link_libraries(
	elevator_controller_interface_node
	modbus
	${catkin_LIBRARIES}
)

add_executable(robotnik_elevator_component_modbus_node src/robotnik_elevator_component_modbus_node.cpp)

add_dependencies(robotnik_elevator_component_modbus_node robotnik_msgs_generate_messages_cpp robotnik_elevator_interface_msgs_generate_messages_cpp)

target_link_libraries(
	robotnik_elevator_component_modbus_node
	${catkin_LIBRARIES}
)
