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

## TinyMPC, vendored from TongjiSuperPower/sp_vision_25 (MIT, see
## thirdparty/LICENSE-TinyMPC). Kept out of src/ on purpose: ament_auto_add_library
## globs src/ recursively, so vendored sources placed there would also be compiled
## into the main library under -Wall -Werror.
add_library(tinympc STATIC
  thirdparty/tinympc/admm.cpp
  thirdparty/tinympc/tiny_api.cpp
  thirdparty/tinympc/rho_benchmark.cpp
)
target_include_directories(tinympc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
target_link_libraries(tinympc PUBLIC Eigen3::Eigen)
set_target_properties(tinympc PROPERTIES POSITION_INDEPENDENT_CODE ON)
## Upstream code trips -Wdeprecated-declarations and -Wunused-variable; silence
## warnings for this target only rather than patching vendored files.
target_compile_options(tinympc PRIVATE -w)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)
target_link_libraries(${PROJECT_NAME} tinympc)

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

  add_executable(test_tracker_logic test/test_tracker_logic.cpp src/tracker_logic.cpp)
  target_include_directories(test_tracker_logic PRIVATE include)
  add_test(NAME tracker_logic COMMAND test_tracker_logic)

  # Guards the vendored solver before anything is built on top of it.
  add_executable(test_tinympc_smoke test/test_tinympc_smoke.cpp)
  target_link_libraries(test_tinympc_smoke tinympc)
  add_test(NAME tinympc_smoke COMMAND test_tinympc_smoke)

  # Hardware-free planner validation: synthetic target, no camera, no MCU.
  # Plain executable rather than a gtest: it reports its own pass/fail summary
  # and needs no assertion framework.
  add_executable(test_planner_offline test/test_planner_offline.cpp)
  target_link_libraries(test_planner_offline ${PROJECT_NAME})
  add_test(NAME planner_offline COMMAND test_planner_offline)

  add_executable(test_gimbal_command_contract test/test_gimbal_command_contract.cpp)
  target_include_directories(test_gimbal_command_contract PRIVATE include)
  ament_target_dependencies(test_gimbal_command_contract rm_interfaces)
  add_test(NAME gimbal_command_contract COMMAND test_gimbal_command_contract)

endif()

#############
## Install ##
#############

ament_auto_package()
