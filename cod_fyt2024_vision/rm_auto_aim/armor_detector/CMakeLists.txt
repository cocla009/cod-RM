cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_compile_options(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
# Only use the stable C ABI. Some OpenVINO distributions set the old C++ ABI
# directory-wide; preserve ROS/OpenCV compile definitions when loading the SDK.
get_directory_property(_ros_compile_definitions COMPILE_DEFINITIONS)
find_package(OpenVINO 2024.6 REQUIRED COMPONENTS Runtime)
set_property(DIRECTORY PROPERTY COMPILE_DEFINITIONS "${_ros_compile_definitions}")
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  openvino::runtime::c
)


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN fyt::auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
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
    ament_cmake_lint_cmake
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})

endif()

#############
## Install ##
#############

# Model provisioning is explicit and reproducible; never download during CMake.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/model/rp_0526.onnx")
  message(WARNING "0526 weights are missing. Run armor_detector/model/fetch_model.py before installation.")
endif()
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_auto_package(
  INSTALL_TO_SHARE
  model
)
