cmake_minimum_required(VERSION 3.5)
project(gbeam2_controller)

# Trova le dipendenze di ROS 2
#ament_auto_find_build_dependencies()

# Dependencies
# find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gbeam2_interfaces REQUIRED)
find_package(gbeam2_library REQUIRED)


install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Definisci gli eseguibili dei nodi
add_executable(polytope_generation_node src/polytope_generation_node.cpp)
add_executable(graph_update_node src/graph_update_node.cpp)                #DA AGGIUNGERE PER GRAPH_UPDATE
add_executable(exploration_node src/exploration_node.cpp)

set(my_dep
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
  "gbeam2_library")


ament_target_dependencies(polytope_generation_node PUBLIC ${my_dep})

ament_target_dependencies(graph_update_node PUBLIC ${my_dep})    #DA AGGIUNGERE PER GRAPH_UPDATE

ament_target_dependencies(exploration_node PUBLIC ${my_dep})





# Dichiarare le dipendenze
# ament_target_dependencies(
#   polytope_generation_node
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   nav_msgs
#   std_msgs
#   tf2_ros
#   tf2_geometry_msgs
#   visualization_msgs
#   gbeam2_library
# )



# Installa gli eseguibili dei nodi
install(TARGETS
  polytope_generation_node
  graph_update_node
  exploration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



# # Installa i file di configurazione
# install(
#   DIRECTORY config/
#   DESTINATION config)



# Install library
# install(
#   TARGETS polytope_fcn graph_fcn exploration_fcn
#   EXPORT  export_gbeam2_library
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# ament_export_include_directories(include)
# ament_export_dependencies(std_msgs)

# Dichiarare il supporto per la compilazione del pacchetto

# ament_auto_package(INSTALL_TO_SHARE launch config)
ament_package()

