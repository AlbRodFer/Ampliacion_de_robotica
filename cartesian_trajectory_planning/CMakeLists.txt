cmake_minimum_required(VERSION 3.16)
project(cartesian_trajectory_planning LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  ament_index_cpp
  kdl_parser
  rclcpp
  trajectory_msgs
  tf2
)
set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)
add_executable(send_linear_trajectory reference_generator/send_linear_trajectory.cpp)
add_executable(send_circular_trajectory reference_generator/send_circular_trajectory.cpp)

ament_target_dependencies(
  send_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)
target_link_libraries(send_trajectory PUBLIC yaml-cpp)

ament_target_dependencies(
  send_linear_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

ament_target_dependencies(
  send_circular_trajectory PUBLIC
  ${REF_GEN_INCLUDE_DEPENDS}
)

add_library(
  cartesian_trajectory_planning
  SHARED
  hardware/r6bot_hardware.cpp
  controller/r6bot_controller.cpp
)

target_compile_features(cartesian_trajectory_planning PUBLIC cxx_std_17)
target_include_directories(cartesian_trajectory_planning PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/cartesian_trajectory_planning>
)
ament_target_dependencies(
  cartesian_trajectory_planning PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface cartesian_trajectory_planning.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface cartesian_trajectory_planning.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/cartesian_trajectory_planning
)
install(
  DIRECTORY description/
  DESTINATION share/cartesian_trajectory_planning
)
install(
  DIRECTORY ros2_control_demo_description/
  DESTINATION share/cartesian_trajectory_planning
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/cartesian_trajectory_planning
)
install(
  DIRECTORY config/
  DESTINATION share/cartesian_trajectory_planning/config
)
install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/cartesian_trajectory_planning
)
install(
    TARGETS send_linear_trajectory
    RUNTIME DESTINATION lib/cartesian_trajectory_planning
)
install(
    TARGETS send_circular_trajectory
    RUNTIME DESTINATION lib/cartesian_trajectory_planning
)

install(TARGETS cartesian_trajectory_planning
  EXPORT export_cartesian_trajectory_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_7_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_7_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_7_launch test/test_r6bot_controller_launch.py)
endif()

## EXPORTS
ament_export_targets(export_cartesian_trajectory_planning HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()