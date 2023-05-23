cmake_minimum_required(VERSION 3.5)
project(simple_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(Threads REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/simple_node/node.cpp
  src/simple_node/actions/action_client.cpp
  src/simple_node/actions/action_server.cpp
)

set(DEPENDENCIES
    rclcpp 
    rcl_action
    rclcpp_action
    action_msgs
)

add_library(${LIB} STATIC ${SOURCES})

ament_target_dependencies(${LIB} ${DEPENDENCIES} Threads)

install(TARGETS ${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${LIB})
ament_export_dependencies(${DEPENDENCIES})


# Python
ament_python_install_package(${PROJECT_NAME})


ament_package()
