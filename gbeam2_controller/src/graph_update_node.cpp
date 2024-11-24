//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <sstream>



#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"

#include "gbeam2_interfaces/msg/graph_cluster_node.hpp"
#include "gbeam2_interfaces/msg/graph_cluster.hpp"
#include "gbeam2_interfaces/srv/set_mapping_status.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // For easier point cloud population

#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "library_fcn.hpp"

#define INF 100000
//-------------------------------------------------------------------------------------------------------------

struct V_info
{
    int node_id;
    double coeff;
    int cluster_id;
};



class GraphUpdateNode : public rclcpp::Node
{
public:
    GraphUpdateNode() : Node("graph_update") 
    {
        name_space = this->get_namespace();
        name_space_id = name_space.back()- '0';
        graph.robot_id = name_space_id;
        graph.last_updater_id = name_space_id;
        
        RCLCPP_INFO(this->get_logger(), "namespace: %s ",name_space.c_str());
        RCLCPP_INFO(this->get_logger(), "namespace_id: %d",name_space_id);
        poly_sub_ = this->create_subscription<gbeam2_interfaces::msg::FreePolygonStamped>(
            "gbeam/free_polytope", 1, std::bind(&GraphUpdateNode::polyCallback, this, std::placeholders::_1));

        graph_pub_ =this->create_publisher<gbeam2_interfaces::msg::Graph>(
          "gbeam/reachability_graph",1);

        clusters_pub_ =this->create_publisher<gbeam2_interfaces::msg::GraphCluster>(
          "gbeam/clusters",1);

        external_poly_sub_ = this->create_subscription<gbeam2_interfaces::msg::FreePolygonStamped>(
            "external_nodes", 1, std::bind(&GraphUpdateNode::extNodesCallback, this, std::placeholders::_1));

        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "merged_obstacles",1);

        
        // SERVICE
        status_server_ = this->create_service<gbeam2_interfaces::srv::SetMappingStatus>(
           "gbeam/set_mapping_status", std::bind(&GraphUpdateNode::setStatus, this, std::placeholders::_1, std::placeholders::_2));

        //Initialize parameters
        this->declare_parameter<double>("node_dist_min",0.0);
        this->declare_parameter<double>("node_dist_open", 0.0);
        this->declare_parameter<double>("node_bound_dist",0.0);
        this->declare_parameter<double>("obstacle_margin",0.0);
        this->declare_parameter<double>("safe_dist",0.0);
        this->declare_parameter<double>("max_lenght_edge",0.0);
        
        //Exploration limits
        this->declare_parameter<double>("limit_xi",0.0);
        this->declare_parameter<double>("limit_xs",0.0);
        this->declare_parameter<double>("limit_yi",0.0);
        this->declare_parameter<double>("limit_ys",0.0);


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Get parameter from yaml file
        node_dist_min = this->get_parameter("node_dist_min").get_parameter_value().get<double>();
        node_dist_open = this->get_parameter("node_dist_open").get_parameter_value().get<double>();
        node_bound_dist = this->get_parameter("node_bound_dist").get_parameter_value().get<double>();
        obstacle_margin = this->get_parameter("obstacle_margin").get_parameter_value().get<double>();
        safe_dist = this->get_parameter("safe_dist").get_parameter_value().get<double>();
        max_lenght_edge = this->get_parameter("max_lenght_edge").get_parameter_value().get<double>();

        limit_xi = this->get_parameter("limit_xi").get_parameter_value().get<double>();
        limit_xs = this->get_parameter("limit_xs").get_parameter_value().get<double>();
        limit_yi = this->get_parameter("limit_yi").get_parameter_value().get<double>();
        limit_ys = this->get_parameter("limit_ys").get_parameter_value().get<double>();
      
        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF GRAPH_UPDATE: ############# ");
        RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
        RCLCPP_INFO(this->get_logger(),"1) NODE_DIST_MIN: %f", node_dist_min);
        RCLCPP_INFO(this->get_logger(),"2) NODE_DIST_OPEN: %f", node_dist_open);
        RCLCPP_INFO(this->get_logger(),"3) NODE_BOUND_DIST: %f", node_bound_dist);
        RCLCPP_INFO(this->get_logger(),"4) OBSTACLE_MARGIN: %f", obstacle_margin);
        RCLCPP_INFO(this->get_logger(),"5) SAFE_DIST: %f", safe_dist);
        RCLCPP_INFO(this->get_logger(),"6) LIMIT_XI: %f", limit_xi);
        RCLCPP_INFO(this->get_logger(),"7) LIMIT_XS: %f", limit_xs);
        RCLCPP_INFO(this->get_logger(),"8) LIMIT_YI: %f", limit_yi);
        RCLCPP_INFO(this->get_logger(),"9) LIMIT_YS: %f", limit_ys);

    }    

    // Declaration of the setStatus function
    bool setStatus(
        const std::shared_ptr<gbeam2_interfaces::srv::SetMappingStatus::Request> request,
        std::shared_ptr<gbeam2_interfaces::srv::SetMappingStatus::Response> response)
    {
        //RCLCPP_INFO(this->get_logger(),"setting status -------> done");
        // Assuming mapping_status is a member variable of your GraphUpdateNode class
        mapping_status = request->request;
        response->response = true;

        return true;
   }

private:
    bool mapping_status;
    double node_dist_min;
    double node_dist_open;
    double node_bound_dist;
    double obstacle_margin;
    double safe_dist;
    double max_lenght_edge;

    bool received_ext_nodes;

    
    double limit_xi, limit_xs, limit_yi, limit_ys;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string name_space;
    int name_space_id;

    gbeam2_interfaces::msg::FreePolygonStamped external_nodes;

    gbeam2_interfaces::msg::Graph graph;
    std::vector<gbeam2_interfaces::msg::Vertex> new_reach_node;
    int E_new=0; //new added edges
    int V_new=0; //new just added edges
    int cluster_state=0;
    std::vector<gbeam2_interfaces::msg::Vertex> first_batch;
    std::vector<gbeam2_interfaces::msg::Vertex> second_batch;
    double gamma_1 = std::numeric_limits<double>::quiet_NaN();
    double gamma_2 = std::numeric_limits<double>::quiet_NaN();
    int V_1;
    int V_2;
    int E_1;
    int E_2;
    std::vector<int> first_batch_ids;
    std::vector<int> second_batch_ids;
    bool start_batch=false;

    gbeam2_interfaces::msg::GraphCluster clusters;
    std::vector<V_info> unclustered_nodes;

    rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr graph_pub_;
    rclcpp::Publisher<gbeam2_interfaces::msg::GraphCluster>::SharedPtr clusters_pub_;
    rclcpp::Subscription<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr poly_sub_;
    rclcpp::Subscription<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr external_poly_sub_;
    rclcpp::Service<gbeam2_interfaces::srv::SetMappingStatus>::SharedPtr status_server_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    void logDoubleVector(rclcpp::Logger logger, const std::vector<double>& vec, const std::string& vec_name = "vector") {
        std::ostringstream oss;
        oss << vec_name << ": [";
        for (size_t i = 0; i < vec.size(); ++i) {
            oss << vec[i];
            if (i < vec.size() - 1) {
                oss << ", ";
            }
        }
        oss << "]";
        RCLCPP_INFO(logger, "%s", oss.str().c_str());
    }

    void printMatrix(rclcpp::Logger logger, const std::vector<std::vector<float>> &matrix, const std::string &matrix_name = "Matrix") {
    RCLCPP_INFO(logger, "Printing %s:", matrix_name.c_str());
    for (size_t i = 0; i < matrix.size(); ++i) {
        std::ostringstream row_stream;
        row_stream << "[ ";
        for (size_t j = 0; j < matrix[i].size(); ++j) {
            row_stream << matrix[i][j];
            if (j < matrix[i].size() - 1) {
                row_stream << ", ";
            }
        }
        row_stream << " ]";
        RCLCPP_INFO(logger, "Row %zu: %s", i, row_stream.str().c_str());
    }
}

    gbeam2_interfaces::msg::Vertex computeCentroid(gbeam2_interfaces::msg::GraphClusterNode& cluster) {
        gbeam2_interfaces::msg::Vertex centroid_vert;
        if (cluster.nodes.empty()) return centroid_vert;

        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        for (int node_id : cluster.nodes) {
            sum_x += graph.nodes[node_id].x;
            sum_y += graph.nodes[node_id].y;
            sum_z += graph.nodes[node_id].z;
            graph.nodes[node_id].cluster_id=cluster.cluster_id;
        }

        int count = cluster.nodes.size();
        centroid_vert.x = sum_x / count;
        centroid_vert.y = sum_y / count;
        centroid_vert.z = sum_z / count;
    return centroid_vert;
    }
    
    double computeModularityGain(gbeam2_interfaces::msg::Vertex candidate_node,gbeam2_interfaces::msg::GraphClusterNode cluster,std::vector<std::vector<float>> weight_adj_matrix, double m){
        // Compute the Modularity gain taking into account a batch of nodes 
        double delta_mod;
        double sum_in=0.0; // This represents the sum of the weights of the links inside a community. It only includes the connections between nodes that are both within the same community.
        double sum_tot=0.0; // This represents the total sum of the weights of the links incident to nodes in a community. It accounts for all connections involving nodes within that community.
        double k_cand =0.0; //sum of the weights of the links incident to node i, 
        double k_in_cand=0.0; //ki,in is the sum of the weights of the links from i to nodes in C
        for (int node_id : cluster.nodes) {
            const auto& node = graph.nodes[node_id];

            // Internal cluster connections
            for (int neighbor_id : node.neighbors) {
                if (std::find(cluster.nodes.begin(), cluster.nodes.end(), neighbor_id) != cluster.nodes.end()) {
                    sum_in += weight_adj_matrix[node_id][neighbor_id];
                }
                // Total edge weights for sum_tot
                sum_tot += weight_adj_matrix[node_id][neighbor_id];
            }
        }

        // Compute k_cand and k_in_cand
        for (int neighbor_id : candidate_node.neighbors) {
                k_cand += weight_adj_matrix[candidate_node.id][neighbor_id];
                if (std::find(cluster.nodes.begin(), cluster.nodes.end(), neighbor_id) != cluster.nodes.end()) {
                    k_in_cand += weight_adj_matrix[candidate_node.id][neighbor_id];
                }
        }

        delta_mod = ((sum_in + 2.0 * k_in_cand) / (2.0 * m) - pow((sum_tot + k_cand) / (2.0 * m), 2)) - (sum_in / (2.0 * m) - pow(sum_tot / (2.0 * m), 2) - pow(k_cand / (2.0 * m), 2));
        
        return delta_mod;  
    }
    
    double computeLocalModularityGain(gbeam2_interfaces::msg::Vertex candidate_node,gbeam2_interfaces::msg::GraphClusterNode cluster, std::unordered_map<int, int> node_to_cluster,std::vector<std::vector<float>> weight_adj_matrix, double m){
        // Compute the Modularity gain taking into account a batch of nodes 
        double delta_mod;
        double sum_in=0.0; // This represents the sum of the weights of the links inside a community. It only includes the connections between nodes that are both within the same community.
        double sum_tot=0.0; // This represents the total sum of the weights of the links incident to nodes in a community. It accounts for all connections involving nodes within that community.
        double k_cand =0.0; //sum of the weights of the links incident to node i, 
        double k_in_cand=0.0; //ki,in is the sum of the weights of the links from i to nodes in C
        for (int node_id : cluster.nodes) {
            const auto& node = graph.nodes[node_id];

            // Internal cluster connections
            for (int neighbor_id : node.neighbors) {
                if (std::find(cluster.nodes.begin(), cluster.nodes.end(), neighbor_id) != cluster.nodes.end()) {
                    sum_in += weight_adj_matrix[node_id][neighbor_id];
                }
                // Total edge weights for sum_tot
                if(node_to_cluster.find(neighbor_id) != node_to_cluster.end()) sum_tot += weight_adj_matrix[node_id][neighbor_id];
            }
        }

        // Compute k_cand and k_in_cand
        for (int neighbor_id : candidate_node.neighbors) {
                if(node_to_cluster.find(neighbor_id) != node_to_cluster.end()) k_cand += weight_adj_matrix[candidate_node.id][neighbor_id];
                if (std::find(cluster.nodes.begin(), cluster.nodes.end(), neighbor_id) != cluster.nodes.end()) {
                    k_in_cand += weight_adj_matrix[candidate_node.id][neighbor_id];
                }
        }

        delta_mod = ((sum_in + 2.0 * k_in_cand) / (2.0 * m) - pow((sum_tot + k_cand) / (2.0 * m), 2)) - (sum_in / (2.0 * m) - pow(sum_tot / (2.0 * m), 2) - pow(k_cand / (2.0 * m), 2));
        
        return delta_mod;  
    }

    gbeam2_interfaces::msg::GraphClusterNode createCluster(std::vector<gbeam2_interfaces::msg::Vertex> nodes, int id){
        gbeam2_interfaces::msg::GraphClusterNode new_cl;
        new_cl.cluster_id = id;
        int N=0;
        for(auto& node: nodes){
            new_cl.nodes.push_back(node.id);
            graph.nodes[node.id].cluster_id =id;
            N++;
        }
        

        /*// Update adjacency matrix
        int N = clusters.adj_matr.size;
        clusters.adj_matr.size = N + 1;
        std::vector<float> new_data((N + 1) * (N + 1), -1.0f);

        // Copy old data to new adjacency matrix
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                new_data[i * (N + 1) + j] = clusters.adj_matr.data[i * N + j];
            }
        }

        // Initialize new row and column
        for (int i = 0; i < N + 1; ++i) {
            new_data[i * (N + 1) + N] = -1.0f;  // New column
            new_data[N * (N + 1) + i] = -1.0f;  // New row
        }

        // Replace old data with new data
        clusters.adj_matr.data = new_data;*/


        return new_cl;

    }
   
    double computeDistanceWeightedTransitivity(gbeam2_interfaces::msg::Vertex& node, gbeam2_interfaces::msg::GraphClusterNode cluster, std::vector<std::vector<float>>& adj_matrix, bool add_node) {
        int closed_triplets = 0;
        int total_triplets = 0;
        double distance_weighted_closed_triplets = 0.0;
        double distance_weighted_total_triplets = 0.0;

        // Compute centroid of the cluster including the new node (if specified)
        if (add_node)  cluster.nodes.push_back(node.id);

        gbeam2_interfaces::msg::Vertex centroid = computeCentroid(cluster);

        for (int u : cluster.nodes) {
            const auto& neighbors = graph.nodes[u].neighbors;

            // Collect neighbors within the cluster
            std::vector<int> subset_neighbors;
            for (int v : neighbors) {
                if (std::find(cluster.nodes.begin(), cluster.nodes.end(), v) != cluster.nodes.end()) {
                    subset_neighbors.push_back(v);
                }
            }

            int k = subset_neighbors.size();
            for (int i = 0; i < k; ++i) {
                for (int j = i + 1; j < k; ++j) {
                    int a = subset_neighbors[i];
                    int b = subset_neighbors[j];

                    // Calculate edge lengths and check if a-b are connected
                    double edge_ab = graph.edges[adj_matrix[a][b]].length;
                    double edge_au = graph.edges[adj_matrix[a][u]].length;
                    double edge_ub = graph.edges[adj_matrix[u][b]].length;
                    
                    if (edge_ab != -1) {
                        closed_triplets++;

                        // Calculate distances to centroid
                        double dist_u = dist(graph.nodes[u],centroid);
                        double dist_a = dist(graph.nodes[a],centroid);
                        double dist_b = dist(graph.nodes[b],centroid);
                        // Add inverse distance as weight (closer triplets contribute more)
                        distance_weighted_closed_triplets += 1.0 / (dist_u + dist_a + dist_b + 1e-6);  // Add small value to avoid division by zero
                    }

                    total_triplets++;
                    distance_weighted_total_triplets += 1.0 / (edge_ab + edge_au + edge_ub + 1e-6);
                }
            }
        }

    return distance_weighted_total_triplets > 0 ? 
           (distance_weighted_closed_triplets / distance_weighted_total_triplets) : -1.0;
    }


    double computeWeightedTransitivity(gbeam2_interfaces::msg::Vertex& node, gbeam2_interfaces::msg::GraphClusterNode& cluster, std::vector<std::vector<float>>& adj_matrix, bool add_node) {
        int closed_triplets = 0;
        int total_triplets = 0;
        double weighted_closed_triplets = 0.0;
        double weighted_total_triplets = 0.0;

        auto temp_cluster = cluster.nodes;
        if(add_node) temp_cluster.push_back(node.id);

        for (int u : temp_cluster) {
            const auto& neighbors = graph.nodes[u].neighbors;

            // Consider only neighbors in the cluster
            std::vector<int> subset_neighbors;
            for (int v : neighbors) {
                if (std::find(temp_cluster.begin(), temp_cluster.end(), v) != temp_cluster.end()) {
                    subset_neighbors.push_back(v);
                }
            }

            int k = subset_neighbors.size();
            for (int i = 0; i < k; ++i) {
                for (int j = i + 1; j < k; ++j) {
                    int a = subset_neighbors[i];
                    int b = subset_neighbors[j];

                    // Calculate the edge lengths between nodes u-a, u-b, and a-b
                    double edge_ua = adj_matrix[u][a];
                    double edge_ub = adj_matrix[u][b];
                    double edge_ab = adj_matrix[a][b];
                    
                    // Check if triplet is closed (a and b are connected)
                    if (edge_ab != -1) {
                        closed_triplets++;
                        weighted_closed_triplets += 1.0 / (edge_ua + edge_ub + edge_ab);  // Weight by sum of edge lengths
                    }
                    total_triplets++;
                    weighted_total_triplets += 1.0 / (edge_ua + edge_ub + (edge_ab != -1 ? edge_ab : 1.0));
                }
            }
        }

        return weighted_total_triplets > 0 ? (weighted_closed_triplets / weighted_total_triplets) : -1.0;
    }


    double computeTransitivity(gbeam2_interfaces::msg::Vertex& node, gbeam2_interfaces::msg::GraphClusterNode& cluster, std::vector<std::vector<float>>& adj_matrix, bool add_node) {
        int closed_triplets = 0;
        int total_triplets = 0;
        int closed_triplets_with_node = 0;
        int total_triplets_with_node = 0;

        auto temp_cluster = cluster.nodes;
        if(add_node) temp_cluster.push_back(node.id); //The las one is the just added node 

        for (int u : temp_cluster) {
            const auto& neighbors = graph.nodes[u].neighbors;
            
            // Only consider neighbors in the cluster
            std::vector<int> subset_neighbors;
            for (int v : neighbors) {
                if (std::find(temp_cluster.begin(), temp_cluster.end(), v) != temp_cluster.end()) {
                    subset_neighbors.push_back(v);
                }
            }

            // Count triplets
            int k = subset_neighbors.size();
            for (int i = 0; i < k; ++i) {
                for (int j = i + 1; j < k; ++j) {
                    int a = subset_neighbors[i];
                    int b = subset_neighbors[j];
                    // Check if a and b are connected
                    if(u!=node.id){
                        if (adj_matrix[a][b]!=-1) {
                        closed_triplets++;
                        closed_triplets_with_node=closed_triplets;
                        }
                        total_triplets++;
                        total_triplets_with_node=total_triplets;
                    }else{
                        if (adj_matrix[a][b]!=-1) {
                        closed_triplets_with_node++;
                        }
                        total_triplets_with_node++;
                    }
                    
                }
            }
        }

        return total_triplets_with_node > 0 ? 
        (3.0 * closed_triplets_with_node / total_triplets_with_node) - 
        (3.0 * closed_triplets / total_triplets) 
        : -10.0;

    }


    double computeClusterCoeff(gbeam2_interfaces::msg::Vertex& node, std::vector<int>& batch_nodes_ids, gbeam2_interfaces::msg::GraphClusterNode& cluster, std::vector<std::vector<float>>& adj_matrix){
        double coeff=0.0;
        int E=0;

        if(node.neighbors.empty()) return -1.0; // BAD CASE OF DISCONNECTED NODE TODO

        std::vector<int> first_intersection; // Temporary storage for intersections
        std::vector<int> neighbours;

        // Ensure inputs are sorted before intersection
        std::sort(node.neighbors.begin(), node.neighbors.end());
        std::sort(batch_nodes_ids.begin(), batch_nodes_ids.end());
        std::sort(cluster.nodes.begin(), cluster.nodes.end());

        // First intersection: node.neighbors and batch_nodes_ids
        std::set_intersection(node.neighbors.begin(), node.neighbors.end(),
                            batch_nodes_ids.begin(), batch_nodes_ids.end(),
                            std::back_inserter(first_intersection));

        // Second intersection: first_intersection and cluster.nodes
        std::set_intersection(node.neighbors.begin(), node.neighbors.end(),
                            cluster.nodes.begin(), cluster.nodes.end(),
                            std::back_inserter(neighbours));

        int V = neighbours.size();
        double min_edge_lenght=INF;
        int triangles=0;

        for (int i : neighbours) {
            for (int j : neighbours) {
                if(adj_matrix[i][j]!=-1){

                }
            auto N_i = graph.nodes[i].neighbors;
            auto N_j = graph.nodes[j].neighbors;
            // Check if 'j' is also a neighbor of 'node'
            if (std::find(node.neighbors.begin(), node.neighbors.end(), j) != node.neighbors.end()) {
                coeff+=1/log(graph.nodes[i].neighbors.size());
            }
            }
        }


        // for (int i:neighbours)
        //     {
        //        coeff+=1/log(graph.nodes[i].neighbors.size());
                
        //     }
        //coeff = (V>1) ? E/(V*(V-1)): 0.0; //E/(V*(V-1))

        return coeff;
    }
    

    void extNodesCallback(const std::shared_ptr<gbeam2_interfaces::msg::FreePolygonStamped> received_nodes){
        // Each time i receive external nodes I store them 
        external_nodes = *received_nodes;
        received_ext_nodes = true;
    }

    void polyCallback(const std::shared_ptr<gbeam2_interfaces::msg::FreePolygonStamped> poly_ptr)
    {
        if(!mapping_status)
            return;

        bool is_changed = false;
        new_reach_node.clear();
        // Compute density before adding new nodes
        int E =graph.edges.size();
        int V = graph.nodes.size();
        double tot_density_prev = 2.0*E/(V*(V-1));

        geometry_msgs::msg::TransformStamped l2g_tf;
        //RCLCPP_INFO(this->get_logger(),"TransformStamped local to global tf -------> done");
        std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom"; //becasue lookupTransform doesn't allow "/" as first character
        std::string source_frame = poly_ptr->header.frame_id;
        try {
            
            //RCLCPP_INFO(this->get_logger(), "lookupTransform -------> %s to %s", target_frame.c_str(), source_frame.c_str());
            l2g_tf = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero); //poly_ptr->header.stamp we get tranformation in the future
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(
                this->get_logger(), "GBEAM:graph_update:lookupTransform: Could not transform %s to %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return;
        }
        
        //std::is_empty<gbeam2_interfaces::msg::FreePolygonStamped>
        if(received_ext_nodes){
            graph.last_updater_id = external_nodes.robot_id;
            is_changed = true;
            RCLCPP_INFO(this->get_logger(), "I received some nodes from %d!", external_nodes.robot_id);

        } else {
            graph.last_updater_id = name_space_id;
        }


        // ####################################################
        // ####### ---------- ADD GRAPH NODES --------- #######
        // ####################################################

        for (int i=0; i<poly_ptr->polygon.vertices_reachable.size(); i++)
        {
            //RCLCPP_INFO(this->get_logger(),"entrato nel primo for -------> ");
            gbeam2_interfaces::msg::Vertex vert = poly_ptr->polygon.vertices_reachable[i];  //get vertex from polytope
            vert.belong_to = name_space_id;
            vert = vert_transform(vert, l2g_tf); //change coordinates to global position

            float vert_dist = vert_graph_distance_noobstacle(graph, vert);
            // vert = applyBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys);

            /// ####### NEW PART FOR COMMUNICATION ############
            float vert_ext_dist; 
            if(received_ext_nodes && external_nodes.polygon.vertices_reachable.size()!=0){
                gbeam2_interfaces::msg::Graph fake_graph;
                fake_graph.nodes = external_nodes.polygon.vertices_reachable;
                vert_ext_dist = vert_graph_distance_noobstacle(fake_graph, vert);
            }
            else{
                vert_ext_dist = INF;
            }
            // ################################################

            if (vert_dist > node_dist_open && vert_ext_dist> node_dist_open) // modified also this condition
            {
            vert.id = graph.nodes.size();
            vert.is_reachable = true;
            vert.gain ++;
            if (!isInBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys))
            {
                vert.is_reachable = false;
                vert.gain = 0;
            }
            addNode(graph,vert); //add vertex to the graph
            if(vert.is_reachable) new_reach_node.push_back(vert);
            is_changed = true;
            }
            else if (vert_ext_dist< node_dist_open)
            {
                RCLCPP_INFO(this->get_logger(),"REACHABLE Vertex %d wasn't added due to conflict with external nodes",i);
            }
            
        }
        for (int i=0; i<poly_ptr->polygon.vertices_obstacles.size(); i++)
        {
            gbeam2_interfaces::msg::Vertex vert = poly_ptr->polygon.vertices_obstacles[i];  //get vertex from polytope
            vert.belong_to = name_space_id;
            vert = vert_transform(vert, l2g_tf); //change coordinates to global position

            vert = moveAway(vert, obstacle_margin);

            float vert_dist = vert_graph_distance_obstacle(graph, vert);

            /// ####### NEW PART FOR COMMUNICATION ############
            float vert_ext_dist; 
            if(received_ext_nodes && external_nodes.polygon.vertices_obstacles.size()!=0){
                gbeam2_interfaces::msg::Graph fake_graph;
                fake_graph.nodes = external_nodes.polygon.vertices_obstacles;
                vert_ext_dist = vert_graph_distance_noobstacle(fake_graph, vert);
            }
            else{
                vert_ext_dist = INF;
            }
            // #################################################
            if ((vert_dist > node_dist_min && vert_ext_dist> node_dist_min) && vert.is_obstacle)
            {
                
            vert.id = graph.nodes.size();
            vert.gain ++;
            if (!isInBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys))
            {
                vert.is_reachable = false;
                vert.gain = 0;
            }

            addNode(graph,vert);         //add vertex to the graph
            if(vert.is_reachable) new_reach_node.push_back(vert);
            is_changed = true;
            }
            else if (vert_ext_dist< node_dist_open)
            {
                RCLCPP_INFO(this->get_logger(),"OBSTACLE Vertex %d wasn't added due to conflict with external nodes",i);
            }
        }

        auto new_adj_matrix = GraphAdj2matrix(graph.adj_matrix);
        int N = new_adj_matrix.size();
        double m = 0.0;
        std::vector<std::vector<float>> weight_adj_matrix(N, std::vector<float>(N, 0.0f));  // Initialize with zeros

        for (int i = 0; i < N; ++i) {
            for (int j = i+1; j < N; ++j) {
                int e_ij = new_adj_matrix[i][j];
                if(e_ij!=-1){
                    weight_adj_matrix[i][j] = 1/graph.edges[e_ij].length;
                } 
                else{
                    weight_adj_matrix[i][j] = 0.0;
                }
                m+= weight_adj_matrix[i][j];
            }
           
        }
        
        //auto cl_adj_matrix = GraphAdj2matrix(clusters.adj_matr);

        // ####################################################
        // ####### ---------- ADD GRAPH EDGES --------- #######
        // ####################################################
        
        //compute polygon in global coordinates
        gbeam2_interfaces::msg::FreePolygon polyGlobal = poly_transform(poly_ptr->polygon, l2g_tf);

        //create vectors with indices of vertices inside polytopes
        std::vector<int> inReachableId;
        std::vector<int> inObstacleNotReachableId;
        for (int i=0; i<graph.nodes.size(); i++)
            if(isInsideObstacles(polyGlobal,graph.nodes[i])){
                if (isInsideReachable(polyGlobal, graph.nodes[i])){
                    inReachableId.push_back(i);
                }else{
                    if(graph.nodes[i].is_obstacle) inObstacleNotReachableId.push_back(i);
                }
            }
            

        
        for (int i=0; i<inReachableId.size(); i++)
        { 
            for (int j=i+1; j<inReachableId.size(); j++)
            {
            if (new_adj_matrix[inReachableId[i]][inReachableId[j]] == -1) 
            {
                //then add edge i-j to graph
                gbeam2_interfaces::msg::Vertex& node_i = graph.nodes[inReachableId[i]];
                gbeam2_interfaces::msg::Vertex& node_j = graph.nodes[inReachableId[j]]; 
                gbeam2_interfaces::msg::GraphEdge edge = computeEdge(node_i, node_j, node_bound_dist);
                edge.id = graph.edges.size();
                if(!node_i.is_obstacle && !node_j.is_obstacle && edge.length < max_lenght_edge){
                    if(isInsideReachable(polyGlobal, node_i) && isInsideReachable(polyGlobal, node_j))
                    edge.is_walkable = true;  // if both vertices are inside reachable poly, then the edge is walkable
                    graph.edges.push_back(edge);

                    //update adjacency matrix
                    new_adj_matrix[inReachableId[i]][inReachableId[j]] = edge.id;
                    new_adj_matrix[inReachableId[j]][inReachableId[i]] = edge.id;
                    graph.nodes[inReachableId[i]].neighbors.push_back(node_j.id);
                    graph.nodes[inReachableId[j]].neighbors.push_back(node_i.id);

                    is_changed = true;

                }
                
            }
            else  // if edge is present, check if it is walkable
            {
                //RCLCPP_INFO(this->get_logger()," -------> entra nell'else del secondo if (ADD GRAPH EDGES)");
                int e = new_adj_matrix[inReachableId[i]][inReachableId[j]];
                if(isInsideReachable(polyGlobal, graph.nodes[graph.edges[e].v1]) && isInsideReachable(polyGlobal, graph.nodes[graph.edges[e].v2]))
                graph.edges[e].is_walkable = true;
            }
            }
        }

        for (int i=0; i<inObstacleNotReachableId.size(); i++)
        {
           for (int j=i+1; j<inObstacleNotReachableId.size(); j++)
            {
               if (new_adj_matrix[inObstacleNotReachableId[i]][inObstacleNotReachableId[j]] == -1) 
                { 
                    //then add edge i-j to graph
                    gbeam2_interfaces::msg::Vertex& obs_i = graph.nodes[inObstacleNotReachableId[i]];
                    gbeam2_interfaces::msg::Vertex& obs_j = graph.nodes[inObstacleNotReachableId[j]]; 
                    gbeam2_interfaces::msg::GraphEdge edge = computeEdge(obs_i, obs_j, node_bound_dist);
                    edge.id = graph.edges.size();
                    if(edge.is_boundary){ //&& obs_i.is_completely_connected && obs_j.is_completely_connected

                        graph.edges.push_back(edge);

                        //update adjacency matrix
                        new_adj_matrix[inObstacleNotReachableId[i]][inObstacleNotReachableId[j]] = edge.id;
                        new_adj_matrix[inObstacleNotReachableId[j]][inObstacleNotReachableId[i]] = edge.id;
                        obs_i.neighbors.push_back(obs_j.id);
                        obs_j.neighbors.push_back(obs_i.id);

                        is_changed = true;
                    } 

                }
            }
        }
        

        graph.adj_matrix=matrix2GraphAdj(new_adj_matrix);

        // ####################################################
        // ############### --- CLUSTERING --- #################
        // ####################################################

        // For new added nodes
        // - Populate first batch according to gamma index that increase until a treshold
        // - Start populate second batch according to gamma index and then stop when same trashold is reached
        // This trigger the clustering evaluation:
        // We have three entities: existing cluster, first batch, second batch.
        // We assume that the second batch compose a new cluster and we focus mainly on assign the nodes "in the middle"
        // or to an existing cluster or otherwhise to the second batch cluster

        V = graph.nodes.size();
        E = graph.edges.size();

        double tot_density_curr;
        V_info node_info;
        double gamma_min = 0.3;

        switch (cluster_state){

            case 0:
                for(auto& new_node:new_reach_node){
                    first_batch.push_back(new_node);
                    first_batch_ids.push_back(new_node.id);
                    node_info.node_id=new_node.id;
                    node_info.coeff = -1.0;
                    node_info.cluster_id =-1;
                    unclustered_nodes.push_back(node_info);
                }
                E_1=0;
                // Count how many edges are between nodes of this batch
                for (int i = 0; i < first_batch.size(); i++)
                {
                    for (int j = first_batch[i].id +1; j < V; j++)
                    {
                        if(new_adj_matrix[first_batch[i].id][j]!=-1) E_1++;
                    }
                    
                }
                V_1 = first_batch.size();
                if (V_1>1)
                {
                    tot_density_curr = 2.0*E_1/(V_1*(V_1-1)); // Current Edge density of the FIRST batch

                    RCLCPP_INFO(this->get_logger(), "Case: %d || current_density: %f V: %d E: %d  || delta: %f",cluster_state,tot_density_curr,V_1,E_1,abs(tot_density_curr - gamma_1)/gamma_1);
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Case: %d || I can't compute current_density V: %d E: %d ",cluster_state,V_1,E_1);
                    break;
                }

                if(tot_density_curr!=0.0 && std::isnan(gamma_1)){
                    gamma_1 = tot_density_curr; 
                    // If i have already enough edge density, i have already a revelant batch
                    if(gamma_1>gamma_min) cluster_state=1;
                } else{
                    if(gamma_1!=0 && abs(tot_density_curr - gamma_1)/gamma_1>gamma_min) cluster_state=1;
                }    


                break;

            case 1:
                // Add new nodes to the batch
                for(auto& new_node:new_reach_node){
                    second_batch.push_back(new_node);
                    second_batch_ids.push_back(new_node.id);
                    node_info.node_id=new_node.id;
                    node_info.coeff = -1.0;
                    node_info.cluster_id =-1;
                    unclustered_nodes.push_back(node_info);
                }
                E_2=0;
                // Count how many edges are between nodes of this batch
                for (int i = 0; i < second_batch.size(); i++)
                {
                    for (int j = second_batch[i].id +1; j < V; j++)
                    {
                        if(new_adj_matrix[second_batch[i].id][j]!=-1) E_2++;
                    }
                    
                }
                V_2 = second_batch.size();

                if (V_2>1)
                {
                    tot_density_curr = 2.0*E_2/(V_2*(V_2-1)); // Current Edge density of the SECOND batch

                    RCLCPP_INFO(this->get_logger(), "Case: %d ||current_density %f V: %d E: %d || delta: %f",cluster_state,tot_density_curr,V_2,E_2,abs(tot_density_curr - gamma_2)/gamma_2);

                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Case: %d ||I can't compute current_density V: %d E: %d ",cluster_state,V_2,E_2);
                    break;
                }

                if(tot_density_curr!=0.0 && std::isnan(gamma_2)){
                    gamma_2 = tot_density_curr; 
                    // If i have already enough edge density, i have already a revelant batch
                    if(gamma_2>gamma_min) cluster_state=1;
                } else{
                    if(gamma_2!=0 && abs(tot_density_curr - gamma_2)/gamma_2>gamma_min) cluster_state=2;
                } 
                
                break;

            default:
                break;
        }
            

        RCLCPP_INFO(this->get_logger(), "Case: %d ||gamma_1: %f gamma_2 %f",cluster_state,gamma_1,gamma_2);

        // Evaluate delta between density and switch logic

      

        if(cluster_state==1 && clusters.clusters.empty()){
            // If I haven't create any cluster yet first batch is the first cluster

            clusters.clusters.push_back(createCluster(first_batch,clusters.clusters.size()));
            RCLCPP_INFO(this->get_logger(), " ###### Created the FIRST new cluster! ######");

            clusters_pub_->publish(clusters);

            //RESET
            tot_density_curr = 0.0;
            first_batch.clear();
            first_batch_ids.clear();
            second_batch.clear();
            second_batch_ids.clear();
            cluster_state=0;
            unclustered_nodes.clear();
            gamma_1 = std::numeric_limits<double>::quiet_NaN();
            gamma_2 = std::numeric_limits<double>::quiet_NaN();
        }

        if(cluster_state==2){ // Evaluate clustering coefficient and create new cluster
            
            int curr_cluster_id;
            bool mod_gain_increase=true;
            int max_iterations=0;
            gbeam2_interfaces::msg::GraphCluster louvain_Com;
            //std::vector<gbeam2_interfaces::msg::GraphClusterNode> louvain_Com;

            // Mapping from node ID to cluster ID for quick lookup
            std::unordered_map<int, int> node_to_cluster;

            // Initialize singleton communities for the second batch
            RCLCPP_INFO(this->get_logger(), "Initializing singleton communities...");
            int next_cluster_id = 0; // Declare this outside the loop or as a class member

            for (int i = unclustered_nodes.size() - 1; i >= 0; i--) {
                auto new_cluster = createCluster(std::vector<gbeam2_interfaces::msg::Vertex>{graph.nodes[unclustered_nodes[i].node_id]}, next_cluster_id);
                node_to_cluster[unclustered_nodes[i].node_id] = next_cluster_id;
                unclustered_nodes[i].cluster_id = next_cluster_id;
                louvain_Com.clusters.push_back(new_cluster);
                RCLCPP_INFO(this->get_logger(), "Node %d initialized as a singleton cluster with ID %d.", unclustered_nodes[i].node_id, next_cluster_id);
                next_cluster_id++; // Increment for the next cluster
            }

            // FIRST PHASE OF LOUVAIN ALGORITHM: COMMUNITY DETECTION
            while (mod_gain_increase && max_iterations < 50) {
                mod_gain_increase = false; // Reset at the beginning of each iteration
                RCLCPP_INFO(this->get_logger(), "Starting iteration %d of the Louvain algorithm.", max_iterations + 1);

                // Inside the iteration loop
                for (int i = unclustered_nodes.size() - 1; i >= 0; i--) {
                    int candidate_id = unclustered_nodes[i].node_id;
                    RCLCPP_INFO(this->get_logger(), "Processing node %d.", candidate_id);
                    int best_cl_id=-1;
                    int old_cluster_id=-1;
                    for (int neigh_id = 0; neigh_id < new_adj_matrix[candidate_id].size(); neigh_id++) {
                        if (new_adj_matrix[candidate_id][neigh_id] != -1 && node_to_cluster.find(neigh_id) != node_to_cluster.end()) {
                            int found_cl_id = node_to_cluster[neigh_id];
                            gbeam2_interfaces::msg::Vertex temp_node = graph.nodes[candidate_id];
                            double temp_gain = computeLocalModularityGain(temp_node, louvain_Com.clusters[found_cl_id], node_to_cluster,weight_adj_matrix, m);
                            RCLCPP_INFO(this->get_logger(), "Node %d -> Neighbor %d: Local modularity gain = %.6f", candidate_id, neigh_id, temp_gain);

                            if (temp_gain > 0.0 && temp_gain > unclustered_nodes[i].coeff) {
                                unclustered_nodes[i].coeff = temp_gain;
                                mod_gain_increase =true; 
                                best_cl_id =found_cl_id;
                            }
                        }
                    }

                    if(best_cl_id!=-1){
                        RCLCPP_INFO(this->get_logger(), "Node %d has MAX modularity gain = %.6f with cluster %d", candidate_id, unclustered_nodes[i].coeff, best_cl_id);
                        // Add the node to the best community
                        old_cluster_id = node_to_cluster[candidate_id];
                        node_to_cluster[candidate_id] = best_cl_id;
                        unclustered_nodes[i].cluster_id = best_cl_id;
                        auto& new_cluster_nodes = louvain_Com.clusters[best_cl_id].nodes;
                        if (std::find(new_cluster_nodes.begin(), new_cluster_nodes.end(), candidate_id) == new_cluster_nodes.end()) {
                            new_cluster_nodes.push_back(candidate_id);
                            
                            RCLCPP_INFO(this->get_logger(), "Node %d added to cluster %d.", candidate_id, best_cl_id);
                        }

                        // Erase it from the old one 
                        if (old_cluster_id >= 0 ) { //&& old_cluster_id != best_cl_id
                            auto& old_cluster_nodes = louvain_Com.clusters[old_cluster_id].nodes;
                            old_cluster_nodes.erase(std::remove(old_cluster_nodes.begin(), old_cluster_nodes.end(), candidate_id), old_cluster_nodes.end());
                        }

                    }
                }

                max_iterations++;
                RCLCPP_INFO(this->get_logger(), "Iteration %d completed.", max_iterations);
                
            }

            if (max_iterations >= 50) {
                RCLCPP_WARN(this->get_logger(), "Maximum iterations reached without convergence.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Louvain algorithm converged after %d iterations.", max_iterations);
            }

            // SEECOND PHASE COMMUNITY AGGREGATION WITH EXISTING CLUSTERS
            int new_id = clusters.clusters.size();
            // Use an iterator-based loop to safely modify the container
            for (auto it = louvain_Com.clusters.begin(); it != louvain_Com.clusters.end(); ) {
                if (!it->nodes.empty()) {
                    it->cluster_id = new_id;
                    for(auto& node_id:it->nodes){
                        graph.nodes[node_id].cluster_id = new_id;
                    }
                    // Add louvain communities to cluster Graph
                    clusters.clusters.push_back(*it);
                    new_id++;
                    ++it; // Move to the next cluster
                } else {
                    RCLCPP_INFO(this->get_logger(), "Removing empty cluster with ID %d.", it->cluster_id);
                    it = louvain_Com.clusters.erase(it); // Erase and get the next iterator
                }
            }
            std::vector<std::vector<float>> cluster_adj_matrix(clusters.clusters.size(), std::vector<float>(clusters.clusters.size(), 0.0f));
            
            for (const auto& pair : node_to_cluster) {
                RCLCPP_INFO(this->get_logger(), "Node %d is in cluster %d", pair.first, pair.second);
            }


            for (auto& cl_i : clusters.clusters) {
                for (auto& node_id : cl_i.nodes) {
                    for (int neigh_id = 0; neigh_id < new_adj_matrix[node_id].size(); neigh_id++) {
                        // Check if the edge exists in the adjacency matrix
                        if (new_adj_matrix[node_id][neigh_id] != -1) {
                            gbeam2_interfaces::msg::Vertex neigh_node = graph.nodes[neigh_id];

                            // Determine the cluster ID of the neighbor node
                            int neigh_cluster_id = neigh_node.cluster_id;
                            RCLCPP_INFO(this->get_logger(), "Neighbor node %d -> cluster ID %d", neigh_id, neigh_cluster_id);
                            if (neigh_cluster_id == -1) {
                                // Fallback to mapping if cluster ID is not set
                                auto it = node_to_cluster.find(neigh_id);
                                if (it != node_to_cluster.end()) {
                                    neigh_cluster_id = it->second;
                                } else {
                                    RCLCPP_WARN(this->get_logger(),"Neighbor node %d has no cluster mapping!", neigh_id);
                                    continue; // Skip if no mapping exists
                                }
                            }

                            // Increment the cluster adjacency matrix entry
                            cluster_adj_matrix[cl_i.cluster_id][neigh_cluster_id] += 1; // 1/graph.edges[new_adj_matrix[node_id][neigh_id]].length
                        }
                    }
                }
            }

            printMatrix(this->get_logger(),cluster_adj_matrix);

            

            while (mod_gain_increase && max_iterations < 50) {
                mod_gain_increase = false; // Reset at the beginning of each iteration
                RCLCPP_INFO(this->get_logger(), "Starting iteration %d of the Louvain algorithm PHASE 2.", max_iterations + 1);

                // Inside the iteration loop
                for (auto& cluster:clusters.clusters) {
                    
                }

                max_iterations++;
                RCLCPP_INFO(this->get_logger(), "Iteration %d completed.", max_iterations);
                
            }


            clusters_pub_->publish(louvain_Com);

            // DEBUG CLOUDPOINT 
            // Prepare the PointCloud2 message
            sensor_msgs::msg::PointCloud2 cloud_msg;
            cloud_msg.header.stamp = this->now();  // Set timestamp
            cloud_msg.header.frame_id = "world";     // Set frame ID (adjust if necessary)

            // Reserve space for the points and the additional "side" field
            cloud_msg.height = 1;                  // Unordered point cloud (1D array)
            cloud_msg.is_dense = false;            // Allow for possible invalid points
            int total_points = 0;
            for(auto& cluster:louvain_Com.clusters){
                total_points+= cluster.nodes.size();
            }
            cloud_msg.width = total_points;        // Number of points

            // Define the PointCloud2 fields
            sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
            modifier.setPointCloud2Fields(4,  // Number of fields: x, y, z, and side
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "comp", 1, sensor_msgs::msg::PointField::FLOAT32); 

            modifier.resize(total_points);  // Resize the point cloud to accommodate all points

            // Use iterators for better handling of PointCloud2
            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_comp(cloud_msg, "comp");

            // Fill the data in row-major order (first all values of reach_node_label, then all values of field_vector)
            int i=0;
            for (const auto& cluster : louvain_Com.clusters) {
                for(int id:cluster.nodes){
                auto node = graph.nodes[id];
                *iter_x = node.x;
                *iter_y = node.y;
                *iter_z = node.z;
                *iter_comp = cluster.cluster_id; 
                
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_comp;

                i++;
                }
                
            }
            // Process obstacles and reachables

            // Publish the point cloud
            point_cloud_publisher_->publish(cloud_msg);


            

            /*for(int i=unclustered_nodes.size()-1;i>=0;i--)
            {   
                std::vector<double> temp_clus_coeff_values;
                for (auto& cluster:clusters.clusters)
                {
                    if(i>first_batch.size()){ // Processing only node of the second batch that could forms a new cluster
                        gbeam2_interfaces::msg::Vertex temp_node=graph.nodes[unclustered_nodes[i].node_id];
                        //double temp=computeDistanceWeightedTransitivity(temp_node,cluster,new_adj_matrix,false);
                        double temp=computeModularityGain(temp_node,cluster,weight_adj_matrix,m);
                        //double temp=computeClusterCoeff(temp_node,second_batch_ids,cluster,new_adj_matrix);
                        temp_clus_coeff_values.push_back(temp);
                        if(temp >= unclustered_nodes[i].coeff){
                            unclustered_nodes[i].coeff=temp;
                            unclustered_nodes[i].cluster_id = cluster.cluster_id;
                        }  
                        curr_cluster_id=2;
                    }
                    else{ 
                        gbeam2_interfaces::msg::Vertex temp_node=graph.nodes[unclustered_nodes[i].node_id];
                        //double temp=computeDistanceWeightedTransitivity(temp_node,cluster,new_adj_matrix,true);
                        double temp=computeModularityGain(temp_node,cluster,weight_adj_matrix,m);
                        //double temp=computeClusterCoeff(graph.nodes[unclustered_nodes[i].node_id],first_batch_ids,cluster,new_adj_matrix);
                        temp_clus_coeff_values.push_back(temp);
                        if(temp >= unclustered_nodes[i].coeff){
                            unclustered_nodes[i].coeff=temp;
                            unclustered_nodes[i].cluster_id = cluster.cluster_id;
                        }   
                        curr_cluster_id=1;

                    }

                                  
                }
                // After evaluate all the cluster coeff, i need to eventually reassign the ones in the second batch cluster

                if (i > first_batch.size() && unclustered_nodes[i].cluster_id != cluster_temp.cluster_id) {
                    // If the node of the second batch is re-assigned, remove it from the previous cluster if present
                    auto& nodes = cluster_temp.nodes;

                    // Find the node in the list of nodes
                    auto it = std::find(nodes.begin(), nodes.end(), unclustered_nodes[i].node_id);

                    // If the node ID is found, erase it
                    if (it != nodes.end()) {
                        nodes.erase(it);
                    }

                }

                RCLCPP_INFO(this->get_logger(),"Clustering || node %d (from batch: %d) assigned to cluster %d",unclustered_nodes[i].node_id,curr_cluster_id, unclustered_nodes[i].cluster_id);
                logDoubleVector(this->get_logger(),temp_clus_coeff_values); 
                if(unclustered_nodes[i].cluster_id !=-1)
                clusters.clusters[unclustered_nodes[i].cluster_id].nodes.push_back(unclustered_nodes[i].node_id);             

                           
            }



            for(auto& cl:clusters.clusters){
                cl.centroid = computeCentroid(cl);
            }
            //clusters.adj_matr = matrix2GraphAdj(cl_adj_matrix);
            */
            

            RCLCPP_INFO(this->get_logger(), "Case: %d || Compute clusters and reset",cluster_state);

            //clusters_pub_->publish(clusters);

            tot_density_curr = 0.0;
            first_batch.clear();
            first_batch_ids.clear();
            second_batch.clear();
            second_batch_ids.clear();
            cluster_state=0;
            unclustered_nodes.clear();
            gamma_1 = std::numeric_limits<double>::quiet_NaN();
            gamma_2 = std::numeric_limits<double>::quiet_NaN();

        }

        
        // ####################################################
        // ####### --- UPDATE CONNECTIONS AND GAINS --- #######
        // #################################################### 

        if(is_changed)
        {
            for(int n=0; n<graph.nodes.size(); n++)
            {
            if(graph.nodes[n].is_obstacle && !graph.nodes[n].is_completely_connected)
            {
                bool connected_left = false, connected_right = false;
                for(int e : new_adj_matrix[n]){
                    if(e!= -1 && graph.edges[e].is_boundary)
                {
                    // compute angular coefficient of the line containing normal: y=mx
                    float m = graph.nodes[n].obstacle_normal.y/graph.nodes[n].obstacle_normal.x;
                    // compute value of the inequality mx-y>0, evaluated for edge direction
                    float value = m * graph.edges[e].direction.x - graph.edges[e].direction.y;
                    if(graph.edges[e].v2 == n)
                    value = -value;
                    if(value>0)
                    connected_right = true;
                    else
                    connected_left = true;
                }

                }


                if (connected_left && connected_right)
                graph.nodes[n].is_completely_connected = true;
                is_changed = true;
                graph.nodes[n].gain = 0;
            }
            }
        }

        // update exploration gain
        gbeam2_interfaces::msg::Vertex position;
        position = vert_transform(position, l2g_tf);  // create temporary vertex at robot position
        for (int i=0; i<graph.nodes.size(); i++)
        {
            if (dist(position, graph.nodes[i]) < node_dist_open)
            {
            graph.nodes[i].gain = 0;
            graph.nodes[i].is_visited = true;
            is_changed = true;
            }
        }

        // publish graph if some change has occurred
        if(is_changed)
            graph_pub_->publish(graph);
            
        if(is_changed && received_ext_nodes)   received_ext_nodes = false;
    
        //end of polyCallback
    }
        
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphUpdateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

