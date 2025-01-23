std_msgs/Header	                    header
int32                               robot_id
int32[]	                            connection_status
geometry_msgs/PoseWithCovarianceStamped	        current_position
geometry_msgs/PoseStamped           last_known_target
geometry_msgs/Vector3[]             joint_vector
geometry_msgs/Vector3[]             normal_joint_vector
gbeam2_interfaces/GraphClusterNode  current_cluster
gbeam2_interfaces/GraphClusterNode  target_cluster
float32	                            distance

	