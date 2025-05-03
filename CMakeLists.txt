cmake_minimum_required(VERSION 3.5)
project(multi_truck_scenario)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(msg_files
  "msg/VehicleBaseData.msg"
  "msg/Solution.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(scenario_lib
  STATIC src/scenario_solver.cpp
         src/scenario_detector.cpp
) 

# Ausführbares Ziel
add_executable(map_node src/map.cpp)
add_executable(vehicle_node src/vehicle.cpp)

# Abhängigkeiten setzen

ament_target_dependencies(vehicle_node rclcpp geometry_msgs std_msgs)
target_include_directories(vehicle_node PRIVATE includes)

target_include_directories(scenario_lib PRIVATE includes)

ament_target_dependencies(map_node rclcpp nav_msgs visualization_msgs)
target_include_directories(map_node PRIVATE includes)

target_link_libraries(vehicle_node scenario_lib)

rosidl_target_interfaces(map_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(vehicle_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(scenario_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")



# Installation
install(
  TARGETS map_node
          vehicle_node
          DESTINATION lib/${PROJECT_NAME}
)


# Testen
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  DIRECTORY launch
            config
            script
  DESTINATION share/${PROJECT_NAME}
)
