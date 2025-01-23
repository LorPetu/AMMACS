#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"
#include "gbeam2_interfaces/msg/status.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sensor_msgs/point_cloud_conversion.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "library_fcn.hpp"
// #include "polytope_fcn.hpp"

class CommDrawer : public rclcpp::Node
{
public:
    CommDrawer() : Node("comm_drawer")
    {
        joint_line_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/status_visualization/joint_line", 1);
        joint_vectors_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/status_visualization/joint_vectors", 1);

        status_sub_ = this->create_subscription<gbeam2_interfaces::msg::Status>(
            "/status", 1,
            std::bind(&CommDrawer::statusCallback, this, std::placeholders::_1));      

        

        // Initialize parameters
        this->declare_parameter<float>("scaling", 0.0);
        this->declare_parameter<int>("N_robot",0);
        this->declare_parameter<double>("communication_range",0.0);

        // Get parameters
        scaling = this->get_parameter("scaling").get_parameter_value().get<float>();
        N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
        wifi_range = this->get_parameter("communication_range").get_parameter_value().get<double>();

        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF COMM_DRAWER: ############# ");
        RCLCPP_INFO(this->get_logger(),"1) SCALING: %f", scaling);
        RCLCPP_INFO(this->get_logger(),"2) Number of robots: %d",N_robot);
        RCLCPP_INFO(this->get_logger(),"3) Communication range: %f",wifi_range);

        curr_status.resize(N_robot);
        for (int i = 0; i < N_robot; i++)
        {
            curr_status[i].connection_status.resize(N_robot);
            curr_status[i].joint_vector.resize(N_robot);
            curr_status[i].normal_joint_vector.resize(N_robot);
        }
  

    }

private:

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr joint_vectors_pub_; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr joint_line_pub_; 

    rclcpp::Subscription<gbeam2_interfaces::msg::Status>::SharedPtr status_sub_;


    float scaling;
    int N_robot;
    double wifi_range;
    std::vector<gbeam2_interfaces::msg::Status> curr_status;

  void statusCallback(const gbeam2_interfaces::msg::Status::SharedPtr received_status){
    curr_status[received_status->robot_id] = *received_status;
    //std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom";
    
    // Initialize edge_markers for /joint_vector
    visualization_msgs::msg::Marker joint_vector_markers;
    joint_vector_markers.ns = "comm_drawer";
    joint_vector_markers.id = 1;
    joint_vector_markers.type = visualization_msgs::msg::Marker::LINE_LIST;
    joint_vector_markers.scale.x = 0.01 * scaling;

    // Initialize edge_markers for /joint_line
    visualization_msgs::msg::Marker joint_line_markers;
    joint_line_markers.ns = "comm_drawer";
    joint_line_markers.id = 1;
    joint_line_markers.type = visualization_msgs::msg::Marker::LINE_LIST;
    joint_line_markers.scale.x = 0.02 * scaling;
    
    // Colors for the markers
    std_msgs::msg::ColorRGBA normals_color;
    normals_color.r = 1.0;
    normals_color.g = 0.8;
    normals_color.b = 0.5;
    normals_color.a = 1.0;

    // Colors for the markers
    std_msgs::msg::ColorRGBA lines_color;
    lines_color.r = 1.0;
    lines_color.g = 0.0;
    lines_color.b = 0.0;
    lines_color.a = 0.7;
    // Colors for the markers
    std_msgs::msg::ColorRGBA connected_lines_color;
    connected_lines_color.r = 0.0;
    connected_lines_color.g = 1.0;
    connected_lines_color.b = 0.0;
    connected_lines_color.a = 0.7;

    std::string target_frame =  "robot0/odom"; //becasue lookupTransform doesn't allow "/" as first character

 
 
    for (int i = 0; i < N_robot; i++) {
        for (int j = i + 1; j < N_robot; j++) { 
            // Push the points corresponding to robot i and robot j
            joint_line_markers.points.push_back(curr_status[i].current_position.pose.pose.position);
            joint_line_markers.points.push_back(curr_status[j].current_position.pose.pose.position);
            if(sqrt(distSq(curr_status[i].current_position.pose.pose.position,curr_status[j].current_position.pose.pose.position)) > wifi_range) {
                joint_line_markers.colors.push_back(lines_color);
                joint_line_markers.colors.push_back(lines_color);
            } else {
                joint_line_markers.colors.push_back(connected_lines_color);
                joint_line_markers.colors.push_back(connected_lines_color);
            }
                

                // Calculate the median point between robot i and robot j
                geometry_msgs::msg::Point median;
                median.x = (curr_status[i].current_position.pose.pose.position.x + curr_status[j].current_position.pose.pose.position.x) / 2.0;
                median.y = (curr_status[i].current_position.pose.pose.position.y + curr_status[j].current_position.pose.pose.position.y) / 2.0;
                median.z = (curr_status[i].current_position.pose.pose.position.z + curr_status[j].current_position.pose.pose.position.z) / 2.0;

                // Add vectors for each pair
                for (int k = 0; k < curr_status[i].joint_vector.size(); k++) {
                    if (i != k ) {  // Ensure not comparing the robot to itself
                        geometry_msgs::msg::Point w, z;

                        // Initialize w and z based on the median
                        w = median;
                        z = median;

                        // Adjust w and z based on the joint vector
                        z.x += 0.1 * curr_status[i].normal_joint_vector[k].x;
                        z.y += 0.1 * curr_status[i].normal_joint_vector[k].y;
                        z.z += 0.1 * curr_status[i].normal_joint_vector[k].z;

                        w.x -= 0.1 * curr_status[i].normal_joint_vector[k].x;
                        w.y -= 0.1 * curr_status[i].normal_joint_vector[k].y;
                        w.z -= 0.1 * curr_status[i].normal_joint_vector[k].z;

                        // Add the points and colors for the visualization
                        joint_vector_markers.points.push_back(w);
                        joint_vector_markers.points.push_back(z);
                        joint_vector_markers.colors.push_back(normals_color);
                        joint_vector_markers.colors.push_back(normals_color);
                    }
                }
   
        }

    }

    joint_vector_markers.header.frame_id = "world";
    joint_line_markers.header.frame_id = "world";



    joint_vectors_pub_->publish(joint_vector_markers);
    joint_line_pub_->publish(joint_line_markers);
    
}



};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommDrawer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
