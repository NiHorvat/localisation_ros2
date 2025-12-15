cmake_minimum_required(VERSION 3.8)
project(serial_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)



find_package(Boost 1.70 REQUIRED COMPONENTS system)
find_package(custom_msgs REQUIRED)                      



if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()


add_executable(serial_parser src/serial_parser.cpp)
ament_target_dependencies(serial_parser rclcpp std_msgs)
target_link_libraries(serial_parser Boost::system)
install(TARGETS
        serial_parser 
        DESTINATION lib/${PROJECT_NAME}
        )
        
ament_target_dependencies(serial_parser rclcpp custom_msgs)   
ament_package()