cmake_minimum_required(VERSION 3.8)
project(take_picture)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)
add_definitions(-O3)


## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} src/take_picture.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs nav_msgs std_srvs cv_bridge OpenCV
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
ament_package()
