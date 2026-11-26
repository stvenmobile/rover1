cmake_minimum_required(VERSION 3.16)
project(rover1_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPS hardware_interface pluginlib rclcpp rclcpp_lifecycle)
find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(rover1_hardware SHARED
  src/rover1_system.cpp
  src/serial_port.cpp
)
target_compile_features(rover1_hardware PUBLIC cxx_std_17)
target_include_directories(rover1_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/rover1_hardware>
)
ament_target_dependencies(rover1_hardware PUBLIC ${DEPS})

pluginlib_export_plugin_description_file(hardware_interface rover1_hardware.xml)

install(DIRECTORY include/ DESTINATION include/rover1_hardware)
install(TARGETS rover1_hardware
  EXPORT export_rover1_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_llc_protocol test/test_llc_protocol.cpp)
  target_include_directories(test_llc_protocol PRIVATE include)
endif()

ament_export_targets(export_rover1_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})
ament_package()
