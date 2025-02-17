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

        robot_triangle_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/status_visualization/robot_triangle", 1);

        robot_triangle_cluster_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/status_visualization/robot_triangle_cluster", 1);
    

        

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

        

        robot_color.resize(N_robot);
        for (size_t i = 0; i <N_robot; i++)
        {
            robot_color[i].r = 0.0; robot_color[i].g = 0.0; robot_color[i].b = 0.0; robot_color[i].a = 1.0;

            // Assign a rainbow color to each marker
            float hue = 360.0f * (static_cast<float>(i) / static_cast<float>(N_robot)); // Normalize hue [0, 360]
            float r, g, b;

            // Convert HSV to RGB (1.0f for full saturation and value)
            

            float c = 1.0f * 1.0f; // Chroma
            float x = c * (1 - std::fabs(std::fmod(hue / 60.0, 2) - 1));
            float m = 1.0f - c;

            if (hue >= 0 && hue < 60) {
                r = c, g = x, b = 0;
            } else if (hue >= 60 && hue < 120) {
                r = x, g = c, b = 0;
            } else if (hue >= 120 && hue < 180) {
                r = 0, g = c, b = x;
            } else if (hue >= 180 && hue < 240) {
                r = 0, g = x, b = c;
            } else if (hue >= 240 && hue < 300) {
                r = x, g = 0, b = c;
            } else {
                r = c, g = 0, b = x;
            }

            r += m;
            g += m;
            b += m;
        
            robot_color[i].r = r; robot_color[i].g = g; robot_color[i].b = b; robot_color[i].a = 1.0;
        }
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

    // Publisher for triangle marker
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_triangle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr robot_triangle_cluster_pub_;



    float scaling;
    int N_robot;
    double wifi_range;
    std::vector<gbeam2_interfaces::msg::Status> curr_status;

    std::vector<std_msgs::msg::ColorRGBA> robot_color;
   

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

    // Triangle position marker
    float z_height_triangle= 12;
    visualization_msgs::msg::Marker triangle_marker;
    triangle_marker.header.frame_id = "world";
    triangle_marker.header.stamp = this->now();
    triangle_marker.ns = "robot_triangle";
    triangle_marker.id = received_status->robot_id;
    triangle_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    triangle_marker.action = visualization_msgs::msg::Marker::ADD;
    triangle_marker.scale.x = 0.2;
    triangle_marker.scale.y = 0.2;
    triangle_marker.scale.z = 0.2;
    triangle_marker.color = robot_color[received_status->robot_id];

    // Extract position and orientation
    const auto& position = received_status->current_position.pose.pose.position;
    const auto& orientation = received_status->current_position.pose.pose.orientation;

    // Convert quaternion to yaw
    double yaw = atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                    1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));

    // Define triangle size
    double base_length = 0.15;  // Base width of the triangle
    double height = 0.25;        // Distance to the front vertex

    // Compute vertices
    geometry_msgs::msg::Point front_vertex, left_vertex, right_vertex;
    front_vertex.x = position.x + height * cos(yaw);
    front_vertex.y = position.y + height * sin(yaw);
    front_vertex.z = z_height_triangle;

    left_vertex.x = position.x - base_length * cos(yaw + M_PI_2);
    left_vertex.y = position.y - base_length * sin(yaw + M_PI_2);
    left_vertex.z = z_height_triangle;

    right_vertex.x = position.x - base_length * cos(yaw - M_PI_2);
    right_vertex.y = position.y - base_length * sin(yaw - M_PI_2);
    right_vertex.z = z_height_triangle;

    // Add the triangle vertices (CCW order)
    triangle_marker.points.push_back(front_vertex);
    triangle_marker.points.push_back(left_vertex);
    triangle_marker.points.push_back(right_vertex);

    // Publish triangle marker
    robot_triangle_pub_->publish(triangle_marker);


    // Triangle Current Cluster marker
    visualization_msgs::msg::Marker cluster_triangle_marker;
    cluster_triangle_marker.header.frame_id = "world";
    cluster_triangle_marker.header.stamp = this->now();
    cluster_triangle_marker.ns = "robot_triangle";
    cluster_triangle_marker.id = received_status->robot_id+25;
    cluster_triangle_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    cluster_triangle_marker.action = visualization_msgs::msg::Marker::ADD;
    cluster_triangle_marker.scale.x = 0.2;
    cluster_triangle_marker.scale.y = 0.2;
    cluster_triangle_marker.scale.z = 0.2;
    cluster_triangle_marker.color = robot_color[received_status->robot_id];

    // Extract position and orientation
    const auto& position2 = received_status->current_cluster.centroid;
    

    // Compute vertices
    front_vertex.x = position2.x + height * cos(yaw);
    front_vertex.y = position2.y + height * sin(yaw);
    front_vertex.z = z_height_triangle;

    left_vertex.x = position2.x - base_length * cos(yaw + M_PI_2);
    left_vertex.y = position2.y - base_length * sin(yaw + M_PI_2);
    left_vertex.z = z_height_triangle;

    right_vertex.x = position2.x - base_length * cos(yaw - M_PI_2);
    right_vertex.y = position2.y - base_length * sin(yaw - M_PI_2);
    right_vertex.z = z_height_triangle;

    // Add the triangle vertices (CCW order)
    cluster_triangle_marker.points.push_back(front_vertex);
    cluster_triangle_marker.points.push_back(left_vertex);
    cluster_triangle_marker.points.push_back(right_vertex);

    // Publish triangle marker
    robot_triangle_cluster_pub_->publish(cluster_triangle_marker);

 
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
