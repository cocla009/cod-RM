cmake_minimum_required(VERSION 3.10)
project(armor_solver)

## Use C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror -O3)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_sources(${PROJECT_NAME} PRIVATE src/tracker_logic.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/gimbal_mpc.cpp src/aim_reference.cpp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::ArmorSolverNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})

  ament_add_gtest(test_aim_correction test/test_aim_correction.cpp)
  target_link_libraries(test_aim_correction ${PROJECT_NAME})

  ament_add_gtest(test_target_ekf_model test/test_target_ekf_model.cpp)
  target_link_libraries(test_target_ekf_model ${PROJECT_NAME})

  ament_add_gtest(test_trajectory_planner test/test_trajectory_planner.cpp)
  target_link_libraries(test_trajectory_planner ${PROJECT_NAME})

  add_executable(test_tracker_logic test/test_tracker_logic.cpp src/tracker_logic.cpp)
  target_include_directories(test_tracker_logic PRIVATE include)
  add_test(NAME tracker_logic COMMAND test_tracker_logic)

  add_executable(test_gimbal_mpc test/test_gimbal_mpc.cpp src/gimbal_mpc.cpp)
  target_include_directories(test_gimbal_mpc PRIVATE include)
  target_link_libraries(test_gimbal_mpc Eigen3::Eigen)
  add_test(NAME gimbal_mpc COMMAND test_gimbal_mpc)

  add_executable(test_gimbal_mpc_closed_loop test/test_gimbal_mpc_closed_loop.cpp
    src/gimbal_mpc.cpp src/aim_reference.cpp src/trajectory_planner.cpp)
  target_include_directories(test_gimbal_mpc_closed_loop PRIVATE include)
  target_link_libraries(test_gimbal_mpc_closed_loop Eigen3::Eigen)
  add_test(NAME gimbal_mpc_closed_loop COMMAND test_gimbal_mpc_closed_loop)

  add_executable(test_mpc_protocol_closed_loop
    test/test_mpc_protocol_closed_loop.cpp src/gimbal_mpc.cpp)
  target_include_directories(test_mpc_protocol_closed_loop PRIVATE
    include ${CMAKE_CURRENT_SOURCE_DIR}/../../rm_hardware_driver/rm_serial_driver/include)
  add_test(NAME mpc_protocol_closed_loop COMMAND test_mpc_protocol_closed_loop)

  add_executable(test_gimbal_command_contract test/test_gimbal_command_contract.cpp)
  target_include_directories(test_gimbal_command_contract PRIVATE include)
  ament_target_dependencies(test_gimbal_command_contract rm_interfaces)
  add_test(NAME gimbal_command_contract COMMAND test_gimbal_command_contract)

  add_executable(test_aim_reference test/test_aim_reference.cpp
    src/aim_reference.cpp src/trajectory_planner.cpp)
  target_include_directories(test_aim_reference PRIVATE include)
  target_link_libraries(test_aim_reference Eigen3::Eigen)
  add_test(NAME aim_reference COMMAND test_aim_reference)

endif()

#############
## Install ##
#############

ament_auto_package()
