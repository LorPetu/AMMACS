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
#include "gbeam2_interfaces/msg/graph_cluster.hpp"
#include "gbeam2_interfaces/msg/graph_cluster_node.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sensor_msgs/point_cloud_conversion.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "library_fcn.hpp"
// #include "polytope_fcn.hpp"

class GraphDrawer : public rclcpp::Node
{
public:
    GraphDrawer() : Node("graph_draw")
    {
        name_space = this->get_namespace();
        name_space_id = name_space.back()- '0';
        graph_nodes_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("gbeam_visualization/graph_nodes", 1);
        graph_normals_pub = this->create_publisher<visualization_msgs::msg::Marker>("gbeam_visualization/graph_nodes_normals", 1);
        graph_edges_pub = this->create_publisher<visualization_msgs::msg::Marker>("gbeam_visualization/graph_edges", 1);
        graph_node_labels_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("gbeam_visualization/graph_node_labels", 1);
        cluster_nodes_pub_= this->create_publisher<visualization_msgs::msg::MarkerArray>("gbeam_visualization/clusters/centroids",1);
        cluster_nodes_labels_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("gbeam_visualization/clusters/centroids_labels",1);
        cluster_edges_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("gbeam_visualization/clusters/edges",1);

        graph_sub = this->create_subscription<gbeam2_interfaces::msg::Graph>(
            "gbeam/reachability_graph", 1,
            std::bind(&GraphDrawer::graphCallback2, this, std::placeholders::_1));

        cluster_graph_sub = this->create_subscription<gbeam2_interfaces::msg::GraphCluster>(
            "coop/Globalclusters", 1,
            std::bind(&GraphDrawer::ClusterCallback, this, std::placeholders::_1));

        this->declare_parameter<float>("scaling", 0.0);
        this->declare_parameter<int>("N_robot",0);

        scaling = this->get_parameter("scaling").get_parameter_value().get<float>();
        N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();

        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF GRAPH_DRAW: ############# ");
        RCLCPP_INFO(this->get_logger(),"1) SCALING: %f", scaling);
        RCLCPP_INFO(this->get_logger(),"2) Number of robots: %d",N_robot);

    }

private:
    std::string name_space;
    int name_space_id;
    int N_robot;

    double mapping_z = 0.25;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr graph_nodes_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_normals_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_edges_pub;
    rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr graph_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_node_labels_pub;

    rclcpp::Subscription<gbeam2_interfaces::msg::GraphCluster>::SharedPtr cluster_graph_sub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cluster_edges_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_nodes_pub_;  
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_nodes_labels_pub_;  

    float scaling;
    
    float packRGB(float r, float g, float b) {
        uint32_t rgb = (static_cast<uint8_t>(r * 255) << 16) |
                    (static_cast<uint8_t>(g * 255) << 8)  |
                    (static_cast<uint8_t>(b * 255));
        float rgb_float;
        std::memcpy(&rgb_float, &rgb, sizeof(float));  // Convert int to float without precision loss
        return rgb_float;
    }

    float getClusterColorRGB(int cluster_id, int robot_id, int total_clusters) {
        if (cluster_id == -2) return packRGB(0.5, 0.5, 0.5);  // Obstacle color
        if (cluster_id == -1) return packRGB(0.5, 0.5, 0.5);  // Not clustered color

        float hue = 360.0f * (static_cast<float>(cluster_id + robot_id * total_clusters) / static_cast<float>(N_robot * total_clusters));
        float r, g, b;
        HSVtoRGB(hue, 1.0f, 1.0f, r, g, b);  // Full saturation and value for bright colors

        return packRGB(r, g, b);
    }

    std_msgs::msg::ColorRGBA getClusterColor(int cluster_id, int robot_id, int total_clusters) {
        std_msgs::msg::ColorRGBA color;
        if (cluster_id == -2) {  // Special case for obstacles
            color.r = 0.5, color.g = 0.5, color.b = 0.5, color.a = 1;
            return color;
        }

        float hue = 360.0f * (static_cast<float>(cluster_id + robot_id * total_clusters) / static_cast<float>(N_robot * total_clusters));
        HSVtoRGB(hue, 1.0f, 1.0f, color.r, color.g, color.b);  // Full saturation and value for bright colors
        color.a = 1.0;

        return color;
    }


    void graphCallback2(const gbeam2_interfaces::msg::Graph::SharedPtr graph_ptr){

        std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom"; //becasue lookupTransform doesn't allow "/" as first character
        
        // define colors
        std_msgs::msg::ColorRGBA boundary_color;
        boundary_color.r = 1, boundary_color.g = 0.8, boundary_color.b = 0.5, boundary_color.a = 1;
        std_msgs::msg::ColorRGBA inside_color;
        inside_color.r = 0, inside_color.g = 0.6, inside_color.b = 0.5, inside_color.a = 0.15;
        boundary_color.r = 1, boundary_color.g = 0.8, boundary_color.b = 0.5, boundary_color.a = 1;
        std_msgs::msg::ColorRGBA walkable_color;
        walkable_color.r = 1, walkable_color.g = 0.1, walkable_color.b = 0.8, walkable_color.a = 0.15;
        std_msgs::msg::ColorRGBA normals_color;
        normals_color.r = 0.6, normals_color.g = 0.3, normals_color.b = 0.6, normals_color.a = 1;



        float normal_length = 0.2 * scaling;

        //initialize node_labels
        visualization_msgs::msg::MarkerArray text_markers;

        //initialize normals, for /graph_nodes_normals
        visualization_msgs::msg::Marker nodes_normals;
        nodes_normals.ns = "graph_drawer", nodes_normals.id = 1, nodes_normals.type = 5, nodes_normals.scale.x = 0.005 * scaling;

        //initialize edge_markers for /graph_edges
        visualization_msgs::msg::Marker edges_markers;
        edges_markers.ns = "graph_drawer", edges_markers.id = 1, edges_markers.type = 5, edges_markers.scale.x = 0.01 * scaling;
        edges_markers.action =0;
        edges_markers.points.clear();
        edges_markers.colors.clear();
        // edges_markers.pose   could be initialized, actually not needed, ony gives a warning

        std_msgs::msg::ColorRGBA custom_color;
        custom_color.r = 0.5;
        custom_color.g = 0.1;
        custom_color.b = 0.7;
        custom_color.a = 1.0;

        std_msgs::msg::ColorRGBA black_color;
        float gray_value = static_cast<float>(name_space_id) / static_cast<float>(N_robot);
        black_color.r = gray_value;
        black_color.g = gray_value;
        black_color.b = gray_value;
        black_color.a = 0.65;

        
        // DEBUG CLOUDPOINT 
        // Prepare the PointCloud2 message
        sensor_msgs::msg::PointCloud2 node_points_cloud;
        node_points_cloud.header.stamp = this->now();  
        
        int N = graph_ptr->nodes.size();    

        // Reserve space for the points and the additional "side" field
        node_points_cloud.height = 1;                  // Unordered point cloud (1D array)
        node_points_cloud.is_dense = false;            // Allow for possible invalid points
        node_points_cloud.width = N;        // Number of points

        // Define the PointCloud2 fields
        sensor_msgs::PointCloud2Modifier modifier(node_points_cloud);
        modifier.setPointCloud2Fields(9,  // Number of fields: x, y, z, and side
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "exp_gain", 1, sensor_msgs::msg::PointField::FLOAT32,
            "is_obstacle", 1, sensor_msgs::msg::PointField::UINT8,
            "is_compl_connected", 1,  sensor_msgs::msg::PointField::UINT8,
            "node_id", 1, sensor_msgs::msg::PointField::UINT32,
            "cluster_id", 1, sensor_msgs::msg::PointField::INT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32); 

        modifier.resize(N);  // Resize the point cloud to accommodate all points

        // Use iterators for better handling of PointCloud2
        sensor_msgs::PointCloud2Iterator<float> iter_x(node_points_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(node_points_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(node_points_cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_gain(node_points_cloud, "exp_gain");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_is_obstacle(node_points_cloud, "is_obstacle");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_is_connected(node_points_cloud, "is_compl_connected");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_node_id(node_points_cloud, "node_id");
        sensor_msgs::PointCloud2Iterator<int32_t> iter_cluster_id(node_points_cloud, "cluster_id");
        sensor_msgs::PointCloud2Iterator<float> iter_rgb(node_points_cloud, "rgb");

        for (int n = 0; n < N; n++){
            auto node = graph_ptr->nodes[n];
            *iter_x = node.x;
            *iter_y = node.y;
            *iter_z = mapping_z;
            *iter_gain = node.gain;
            *iter_node_id = node.id;
            *iter_is_obstacle = node.is_obstacle;
            *iter_is_connected = node.is_completely_connected;
            *iter_cluster_id = node.is_obstacle ? -2: node.cluster_id; 
            *iter_rgb = getClusterColorRGB(node.cluster_id, name_space_id,graph_ptr->cluster_graph.clusters.size());  // Assign color
            
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_cluster_id; ++iter_node_id; ++iter_gain;
            ++iter_is_obstacle; ++iter_is_connected; ++iter_rgb;

           

            // Assign cluster color
            std_msgs::msg::ColorRGBA cluster_color = getClusterColor(node.cluster_id, name_space_id,graph_ptr->cluster_graph.clusters.size());

            // Create a text marker for each node
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = target_frame;
            text_marker.ns = "node_labels";
            text_marker.id = n;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = node.x;
            text_marker.pose.position.y = node.y;
            text_marker.pose.position.z = mapping_z + 0.1; // Offset the text slightly above the node
            text_marker.scale.z = 0.1 * scaling; // Text height
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = std::to_string(n);
            text_markers.markers.push_back(text_marker);



            // Create Normals for
            if (node.is_obstacle)
            {
                geometry_msgs::msg::Point w, z;
                w.x = node.x, w.y = node.y, w.z = mapping_z;
                z.x = node.x, z.y = node.y, z.z = mapping_z;
                w.x += 0.5 * normal_length * node.obstacle_normal.x;
                w.y += 0.5 * normal_length * node.obstacle_normal.y;
                z.x -= 0.5 * normal_length * node.obstacle_normal.x;
                z.y -= 0.5 * normal_length * node.obstacle_normal.y;
                nodes_normals.points.push_back(w);
                nodes_normals.points.push_back(z);
                nodes_normals.colors.push_back(normals_color);
                nodes_normals.colors.push_back(normals_color);
            }
        }

        //add edges
        for (int e = 0; e < graph_ptr->edges.size(); e++)
        {   
            auto& node_i = graph_ptr->nodes[graph_ptr->edges[e].v1]; node_i.z = mapping_z-0.03;
            auto& node_j = graph_ptr->nodes[graph_ptr->edges[e].v2]; node_j.z = mapping_z-0.03;
            edges_markers.points.push_back(vertex2point(node_i));
            edges_markers.points.push_back(vertex2point(node_j));

            edges_markers.colors.push_back(black_color);
            edges_markers.colors.push_back(black_color);
        }

        for (auto& bridge : graph_ptr->cluster_graph.bridges){
            if(bridge.belong_to==name_space_id){
                gbeam2_interfaces::msg::Vertex centr_i = graph_ptr->nodes[bridge.v1]; centr_i.z = mapping_z-0.03;
                gbeam2_interfaces::msg::Vertex centr_j; centr_j.z = mapping_z-0.03;
                centr_j.x = centr_i.x + bridge.length*bridge.direction.x;
                centr_j.y = centr_i.y + bridge.length*bridge.direction.y;
                edges_markers.points.push_back(vertex2point(centr_i));
                edges_markers.points.push_back(vertex2point(centr_j));

                edges_markers.colors.push_back(custom_color);
                edges_markers.colors.push_back(custom_color);
            }
        }

        edges_markers.header.frame_id = target_frame;
        nodes_normals.header.frame_id = target_frame;
        node_points_cloud.header.frame_id = target_frame; 


        // Publish the point cloud
        graph_nodes_pub->publish(node_points_cloud);
        graph_normals_pub->publish(nodes_normals);
        graph_edges_pub->publish(edges_markers);
        graph_node_labels_pub->publish(text_markers);

    }

    void HSVtoRGB(float h, float s, float v, float& r, float& g, float& b) {
        float c = v * s; // Chroma
        float x = c * (1 - std::fabs(std::fmod(h / 60.0, 2) - 1));
        float m = v - c;

        if (h >= 0 && h < 60) {
            r = c, g = x, b = 0;
        } else if (h >= 60 && h < 120) {
            r = x, g = c, b = 0;
        } else if (h >= 120 && h < 180) {
            r = 0, g = c, b = x;
        } else if (h >= 180 && h < 240) {
            r = 0, g = x, b = c;
        } else if (h >= 240 && h < 300) {
            r = x, g = 0, b = c;
        } else {
            r = c, g = 0, b = x;
        }

        r += m;
        g += m;
        b += m;
    }


    void ClusterCallback(const gbeam2_interfaces::msg::GraphCluster::SharedPtr cluster_graph_ptr) {
        double gain_scale = 0.005;
        double cluster_height = 5.0;
        double min_cluster_size = 0.08;
        std::string target_frame = name_space.substr(1, name_space.length() - 1) + "/odom"; // becasue lookupTransform doesn't allow "/" as first character

        visualization_msgs::msg::MarkerArray centroid_points;
        visualization_msgs::msg::MarkerArray text_markers; // Marker array for text markers
        visualization_msgs::msg::Marker edges_markers;
        edges_markers.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_markers.id = 2;
        edges_markers.header.frame_id = target_frame;
        edges_markers.ns = "clusters_e";
        edges_markers.scale.x = 0.05 * scaling;

        std_msgs::msg::ColorRGBA walkable_color;
        walkable_color.r = 1, walkable_color.g = 0.1, walkable_color.b = 0.8, walkable_color.a = 0.15;

        std::vector<std_msgs::msg::ColorRGBA> robot_color;
        std_msgs::msg::ColorRGBA white;
        robot_color.resize(N_robot);
        for (size_t i = 0; i <N_robot; i++)
        {
            robot_color[i].r = 0.0; robot_color[i].g = 0.0; robot_color[i].b = 0.0; robot_color[i].a = 1.0;

            // Assign a rainbow color to each marker
            float hue = 360.0f * (static_cast<float>(i) / static_cast<float>(N_robot)); // Normalize hue [0, 360]
            float r, g, b;
            HSVtoRGB(hue, 1.0f, 1.0f, r, g, b); // Convert HSV to RGB (1.0f for full saturation and value)
            robot_color[i].r = r; robot_color[i].g = g; robot_color[i].b = b; robot_color[i].a = 1.0;
        }

        std_msgs::msg::ColorRGBA black_color;
        float gray_value = static_cast<float>(name_space_id) / static_cast<float>(N_robot);
        black_color.r = gray_value;
        black_color.g = gray_value;
        black_color.b = gray_value;
        black_color.a = 0.65;

        std::vector<int> cluster_count(N_robot, 0);

        for (const auto& cluster : cluster_graph_ptr->clusters) {
            if (cluster.belong_to >= 0 && cluster.belong_to < N_robot) {
                cluster_count[cluster.belong_to]++;
            }
        }

        // Draw each centroid of the cluster
        int i=0;
        for (auto& cluster : cluster_graph_ptr->clusters) {
            // CYLINDER marker for the cluster centroid
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = target_frame;
            marker.ns = "cluster_centroid";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;

            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = cluster.centroid.x;
            marker.pose.position.y = cluster.centroid.y;
            marker.pose.position.z = cluster_height;
            marker.scale.x = (cluster.nodes.size()*0.03 < min_cluster_size)? min_cluster_size : cluster.nodes.size()*0.03;
            marker.scale.y = (cluster.nodes.size()*0.03 < min_cluster_size)? min_cluster_size : cluster.nodes.size()*0.03;
            marker.scale.z = (cluster.total_gain * gain_scale< min_cluster_size) ? min_cluster_size : cluster.total_gain * gain_scale;

            marker.color = getClusterColor(cluster.cluster_id,cluster.belong_to, cluster_count[cluster.belong_to]);//(cluster.unexplored_nodes.empty()) ? white : robot_color[cluster.belong_to];

            

            // TEXT marker for the cluster ID
            visualization_msgs::msg::Marker text_marker;

            text_marker.header.frame_id = target_frame;
            text_marker.ns = "cluster_id";
            text_marker.id = i + 1000; // Ensure unique IDs for text markers
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

            text_marker.action = visualization_msgs::msg::Marker::ADD;

            text_marker.pose.position.x = cluster.centroid.x;
            text_marker.pose.position.y = cluster.centroid.y;
            text_marker.pose.position.z = cluster_height + (cluster.total_gain * gain_scale) + 0.5; 

            text_marker.scale.z = 0.2; // Font size
            text_marker.color.r = 1.0; // White text
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            text_marker.text = "C" + std::to_string(cluster.cluster_id)+"R"+std::to_string(cluster.belong_to); // Cluster ID as text

            text_markers.markers.push_back(text_marker);
            centroid_points.markers.push_back(marker);
            i++;
        }

        // Process adjacency matrix and edges
        auto adj_matrix = GraphAdj2matrix(cluster_graph_ptr->adj_matrix);

        for (int i = 0; i < adj_matrix.size(); i++) {
            for (int j = i + 1; j < adj_matrix.size(); j++) {
                if (adj_matrix[i][j] > 0.0) {
                    gbeam2_interfaces::msg::Vertex centr_i = cluster_graph_ptr->clusters[i].centroid;
                    centr_i.z = cluster_height;
                    gbeam2_interfaces::msg::Vertex centr_j = cluster_graph_ptr->clusters[j].centroid;
                    centr_j.z = cluster_height;
                    edges_markers.points.push_back(vertex2point(centr_i));
                    edges_markers.points.push_back(vertex2point(centr_j));

                    edges_markers.colors.push_back(black_color);
                    edges_markers.colors.push_back(black_color);
                }
            }
        }

        

        // 

        // Publish markers
        cluster_nodes_pub_->publish(centroid_points);
        cluster_nodes_labels_pub_->publish(text_markers); // Publish text markers
        cluster_edges_pub_->publish(edges_markers);

        text_markers.markers.clear();
        centroid_points.markers.clear();
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphDrawer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
