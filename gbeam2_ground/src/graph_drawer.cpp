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

        graph_sub = this->create_subscription<gbeam2_interfaces::msg::Graph>(
            "gbeam/reachability_graph", 1,
            std::bind(&GraphDrawer::graphCallback, this, std::placeholders::_1));

        this->declare_parameter<float>("scaling", 0.0);

        scaling = this->get_parameter("scaling").get_parameter_value().get<float>();

        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF GRAPH_DRAW: ############# ");
        RCLCPP_INFO(this->get_logger(),"1) SCALING: %f", scaling);

    }

private:
    std::string name_space;
    int  name_space_id;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr graph_nodes_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_normals_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_edges_pub;
    rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr graph_sub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_node_labels_pub;

    float scaling;
     sensor_msgs::msg::PointCloud2 pointCloudTOpointCloud2(const sensor_msgs::msg::PointCloud msg)
    {
        sensor_msgs::msg::PointCloud2 pointcloud2_msg;
        pointcloud2_msg.header = msg.header;
        pointcloud2_msg.height = 1;
        pointcloud2_msg.width = msg.points.size();
        pointcloud2_msg.is_dense = false;
        pointcloud2_msg.is_bigendian = false;
        
        // Define fields
        pointcloud2_msg.fields.resize(7); // x, y, z, exp_gain, is_obstacle, is_completely_connected, node_id
        pointcloud2_msg.fields[0].name = "x";
        pointcloud2_msg.fields[0].offset = 0;
        pointcloud2_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud2_msg.fields[0].count = 1;
        
        pointcloud2_msg.fields[1].name = "y";
        pointcloud2_msg.fields[1].offset = 4;
        pointcloud2_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud2_msg.fields[1].count = 1;
        
        pointcloud2_msg.fields[2].name = "z";
        pointcloud2_msg.fields[2].offset = 8;
        pointcloud2_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud2_msg.fields[2].count = 1;
        
        pointcloud2_msg.fields[3].name = "exp_gain";
        pointcloud2_msg.fields[3].offset = 12;
        pointcloud2_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloud2_msg.fields[3].count = 1;

        pointcloud2_msg.fields[4].name = "is_obstacle";
        pointcloud2_msg.fields[4].offset = 16;
        pointcloud2_msg.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
        pointcloud2_msg.fields[4].count = 1;

        pointcloud2_msg.fields[5].name = "is_completely_connected";
        pointcloud2_msg.fields[5].offset = 17;
        pointcloud2_msg.fields[5].datatype = sensor_msgs::msg::PointField::UINT8;
        pointcloud2_msg.fields[5].count = 1;

        pointcloud2_msg.fields[6].name = "node_id";
        pointcloud2_msg.fields[6].offset = 18;
        pointcloud2_msg.fields[6].datatype = sensor_msgs::msg::PointField::UINT32;
        pointcloud2_msg.fields[6].count = 1;

        pointcloud2_msg.point_step = 22;  // 3 fields x 4 bytes/field + 2 fields x 1 byte/field + 1 field x 4 bytes/field
        pointcloud2_msg.row_step = pointcloud2_msg.point_step * pointcloud2_msg.width;
        
        // Reserve memory for the point data
        pointcloud2_msg.data.resize(pointcloud2_msg.row_step * pointcloud2_msg.height);
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud2_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud2_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud2_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_exp_gain(pointcloud2_msg, "exp_gain");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_is_obstacle(pointcloud2_msg, "is_obstacle");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_is_connected(pointcloud2_msg, "is_completely_connected");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_node_id(pointcloud2_msg, "node_id");
        
        for (size_t i = 0; i < msg.points.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_exp_gain, ++iter_is_obstacle, ++iter_is_connected, ++iter_node_id)
        {
            *iter_x = msg.points[i].x;
            *iter_y = msg.points[i].y;
            *iter_z = msg.points[i].z;
            *iter_exp_gain = msg.channels[0].values[i];
            *iter_is_obstacle = static_cast<uint8_t>(msg.channels[1].values[i]);
            *iter_is_connected = static_cast<uint8_t>(msg.channels[2].values[i]);
            *iter_node_id = static_cast<uint32_t>(msg.channels[3].values[i]);
        }
        
        // return PointCloud2 message
        return pointcloud2_msg;
    };

    void graphCallback(const gbeam2_interfaces::msg::Graph::SharedPtr graph_ptr)
    {
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

        //initialize node_points for /graph_nodes
        sensor_msgs::msg::PointCloud node_points;
        sensor_msgs::msg::ChannelFloat32 expGainChan, obstacleChan, connectedChan, nodeIdChan;
        expGainChan.name = "exploration_gain";
        obstacleChan.name = "is_obstacle";
        connectedChan.name = "is_compl_connected";
        nodeIdChan.name = "node_id";

        //initialize normals, for /graph_nodes_normals
        visualization_msgs::msg::Marker nodes_normals;
        nodes_normals.ns = "graph_drawer", nodes_normals.id = 1, nodes_normals.type = 5, nodes_normals.scale.x = 0.005 * scaling;

        //initialize edge_markers for /graph_edges
        visualization_msgs::msg::Marker edges_markers;
        edges_markers.ns = "graph_drawer", edges_markers.id = 1, edges_markers.type = 5, edges_markers.scale.x = 0.005 * scaling;
        // edges_markers.pose   could be initialized, actually not needed, ony gives a warning

        //add nodes and nodes normals
        for (int n = 0; n < graph_ptr->nodes.size(); n++)
        {
            geometry_msgs::msg::Point32 point;
            point.x = graph_ptr->nodes[n].x;
            point.y = graph_ptr->nodes[n].y;
            point.z = graph_ptr->nodes[n].z;
            node_points.points.push_back(point);
            expGainChan.values.push_back(graph_ptr->nodes[n].gain);
            obstacleChan.values.push_back(graph_ptr->nodes[n].is_obstacle);
            connectedChan.values.push_back(graph_ptr->nodes[n].is_completely_connected);
            nodeIdChan.values.push_back(static_cast<float>(n));  // Add node ID

            // Create a text marker for each node
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = target_frame;
            text_marker.ns = "node_labels";
            text_marker.id = n;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = point.x;
            text_marker.pose.position.y = point.y;
            text_marker.pose.position.z = point.z + 0.1; // Offset the text slightly above the node
            text_marker.scale.z = 0.1 * scaling; // Text height
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = std::to_string(n);
            text_markers.markers.push_back(text_marker);

            if (graph_ptr->nodes[n].is_obstacle)
            {
                geometry_msgs::msg::Point w, z;
                w.x = graph_ptr->nodes[n].x, w.y = graph_ptr->nodes[n].y, w.z = graph_ptr->nodes[n].z;
                z.x = graph_ptr->nodes[n].x, z.y = graph_ptr->nodes[n].y, z.z = graph_ptr->nodes[n].z;
                w.x += 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.x;
                w.y += 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.y;
                z.x -= 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.x;
                z.y -= 0.5 * normal_length * graph_ptr->nodes[n].obstacle_normal.y;
                nodes_normals.points.push_back(w);
                nodes_normals.points.push_back(z);
                nodes_normals.colors.push_back(normals_color);
                nodes_normals.colors.push_back(normals_color);
            }
        }

        //add edges
        for (int e = 0; e < graph_ptr->edges.size(); e++)
        {
            edges_markers.points.push_back(vertex2point(graph_ptr->nodes[graph_ptr->edges[e].v1]));
            edges_markers.points.push_back(vertex2point(graph_ptr->nodes[graph_ptr->edges[e].v2]));
            if (graph_ptr->edges[e].is_boundary)
            {
                edges_markers.colors.push_back(boundary_color);
                edges_markers.colors.push_back(boundary_color);
            }
            else
            {
                if (graph_ptr->edges[e].is_walkable)
                {
                    edges_markers.colors.push_back(walkable_color);
                    edges_markers.colors.push_back(walkable_color);
                }
                else
                {
                    edges_markers.colors.push_back(inside_color);
                    edges_markers.colors.push_back(inside_color);
                }
            }
        }
        node_points.channels.push_back(expGainChan);
        node_points.channels.push_back(obstacleChan);
        node_points.channels.push_back(connectedChan);
        node_points.channels.push_back(nodeIdChan);  // Add node ID channel

        

        node_points.header.frame_id = target_frame;
        edges_markers.header.frame_id = target_frame;
        nodes_normals.header.frame_id = target_frame;

        sensor_msgs::msg::PointCloud2 node_points_2 = pointCloudTOpointCloud2(node_points);

        graph_nodes_pub->publish(node_points_2);
        graph_normals_pub->publish(nodes_normals);
        graph_edges_pub->publish(edges_markers);
        graph_node_labels_pub->publish(text_markers);
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
