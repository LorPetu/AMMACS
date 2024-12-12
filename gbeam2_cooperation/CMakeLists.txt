cmake_minimum_required(VERSION 3.8)
project(gbeam2_cooperation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

######### DEPENDENCIES ##############
 set(DEPENDANCIES
 "rclcpp"
 "std_msgs"
 "std_srvs"
 "geometry_msgs"
 "sensor_msgs"
 "nav_msgs"
 "visualization_msgs"
 "gbeam2_interfaces"
 "tf2_ros"
 "tf2_geometry_msgs"
 "gbeam2_library"
 "Eigen3"

)
# Loop over each dependancy
foreach(DEP IN LISTS DEPENDANCIES)

 find_package(${DEP} REQUIRED)
 
endforeach(DEP IN LISTS DEPENDANCIES)

add_executable(cooperation_manager src/cooperation_manager.cpp)

ament_target_dependencies(cooperation_manager PUBLIC ${DEPENDANCIES})

# Add Spectra (Header-only library, specify the include path manually)
set(SPECTRA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../spectra/include")
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${SPECTRA_DIR}
)

target_include_directories(cooperation_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cooperation_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS cooperation_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
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
