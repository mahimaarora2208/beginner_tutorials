cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
)
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} 
#   "rosidl_typesupport_cpp"
# )
# target_link_libraries(talker "${cpp_typesupport_target}") 

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(client src/change_string_client.cpp)
ament_target_dependencies(client rclcpp std_msgs)

install(TARGETS
  talker
  listener
  client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

rosidl_target_interfaces(talker
${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(client
${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(listener
${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
