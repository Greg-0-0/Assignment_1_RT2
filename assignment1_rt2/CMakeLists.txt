cmake_minimum_required(VERSION 3.8)
project(assignment1_rt2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_tutorials_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(action_server SHARED src/action_server.cpp)
add_library(action_client SHARED src/action_client.cpp)
add_library(frame_publisher SHARED src/broadcaster.cpp)
add_library(user_interface SHARED src/user_interface.cpp)

target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(frame_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(user_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


target_compile_definitions(action_client PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
target_compile_definitions(action_server PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
target_compile_definitions(frame_publisher PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
target_compile_definitions(user_interface PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

ament_target_dependencies(action_client "action_tutorials_interfaces" "rclcpp" "geometry_msgs" "nav_msgs" "rclcpp_action" "rclcpp_components" "tf2")
ament_target_dependencies(action_server "action_tutorials_interfaces" "rclcpp" "geometry_msgs" "nav_msgs" "rclcpp_action" "rclcpp_components" "tf2" "tf2_ros")
ament_target_dependencies(frame_publisher "tf2" "tf2_ros" "tf2_geometry_msgs" "rclcpp" "geometry_msgs" "nav_msgs" "rclcpp_components")
ament_target_dependencies(user_interface "action_tutorials_interfaces" "rclcpp" "geometry_msgs" "nav_msgs" "rclcpp_action" "rclcpp_components" "tf2" "tf2_ros")

rclcpp_components_register_node(action_client PLUGIN "assignment1_rt2::NavigationActionClient" EXECUTABLE robot_client)
rclcpp_components_register_node(action_server PLUGIN "assignment1_rt2::NavigationActionServer" EXECUTABLE robot_controller)
rclcpp_components_register_node(frame_publisher PLUGIN "assignment1_rt2::FramePublisher" EXECUTABLE frame_publisher_node)
rclcpp_components_register_node(user_interface PLUGIN "assignment1_rt2::UserInterface" EXECUTABLE user_interface_node)

ament_export_targets(export_${PROJECT_NAME}_action_client)
ament_export_targets(export_${PROJECT_NAME}_action_server)
ament_export_targets(export_${PROJECT_NAME}_frame_publisher)
ament_export_targets(export_${PROJECT_NAME}_user_interface)


install(TARGETS action_client
  EXPORT export_${PROJECT_NAME}_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS action_server
  EXPORT export_${PROJECT_NAME}_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS frame_publisher
  EXPORT export_${PROJECT_NAME}_frame_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS user_interface
  EXPORT export_${PROJECT_NAME}_user_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
