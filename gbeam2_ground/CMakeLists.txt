cmake_minimum_required(VERSION 3.8)
project(gbeam2_ground)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gbeam2_interfaces REQUIRED)
find_package(gbeam2_library REQUIRED)



# Definisci gli eseguibili dei nodi
add_executable(poly_drawer src/poly_drawer.cpp)
add_executable(graph_drawer src/graph_drawer.cpp)
add_executable(communication_drawer src/communication_drawer.cpp)

set(my_dep
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "visualization_msgs"
  "foxglove_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "gbeam2_interfaces"
  "gbeam2_library")


ament_target_dependencies(poly_drawer PUBLIC ${my_dep})
ament_target_dependencies(graph_drawer PUBLIC ${my_dep})
ament_target_dependencies(communication_drawer PUBLIC ${my_dep})


#target_include_directories(poly_drawer PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(poly_drawer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS poly_drawer graph_drawer communication_drawer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
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

#################################################################################################
#################################################################################################

# cmake_minimum_required(VERSION 3.8)
# project(gbeam2_ground)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# # Find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(visualization_msgs REQUIRED)
# find_package(gbeam2_library REQUIRED)
# find_package(gbeam2_interfaces REQUIRED)

# # Include directories
# include_directories(
#   include
#   ${rclcpp_INCLUDE_DIRS}
#   ${std_msgs_INCLUDE_DIRS}
#   ${sensor_msgs_INCLUDE_DIRS}
#   ${geometry_msgs_INCLUDE_DIRS}
#   ${nav_msgs_INCLUDE_DIRS}
#   ${tf2_ros_INCLUDE_DIRS}
#   ${tf2_geometry_msgs_INCLUDE_DIRS}
#   ${visualization_msgs_INCLUDE_DIRS}
#   ${gbeam2_library_INCLUDE_DIRS}
#   ${gbeam2_interfaces_INCLUDE_DIRS}
# )

# # Create executables
# add_executable(poly_drawer src/poly_drawer.cpp)
# add_executable(graph_drawer src/graph_drawer.cpp)

# # Add dependencies for executables
# ament_target_dependencies(poly_drawer
#   rclcpp
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   visualization_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   gbeam2_library
#   gbeam2_interfaces
# )
# ament_target_dependencies(graph_drawer
#   rclcpp
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   visualization_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   gbeam2_library
#   gbeam2_interfaces
# )

# # Install executables
# install(TARGETS
#   poly_drawer
#   graph_drawer
#   DESTINATION lib/${PROJECT_NAME})

# # Don't forget to uncomment the installation of other files if needed

# # Add tests if required

# ament_package()

