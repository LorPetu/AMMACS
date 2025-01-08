//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <utility>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Vector3.h"

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/graph_cluster_node.hpp"
#include "library_fcn.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // For easier point cloud population


#define INF 100000

class CooperationNode : public rclcpp::Node
{
public:
  CooperationNode() : Node("coop_manager"){

    //SUBSCRIBED TOPICS
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            name_space+ "odom", 1, std::bind(&CooperationNode::odomCallback, this, std::placeholders::_1));

    merged_graph_sub_= this->create_subscription<gbeam2_interfaces::msg::Graph>(
      "gbeam/merged_graph",1,std::bind(&CooperationNode::mergedGraphCallback,this,std::placeholders::_1));

    // Get namespace
    name_space = this->get_namespace();
    name_space_id = name_space.back()- '0';
    

    std::string filter_string = "robot_id!=%0";          
    rclcpp::SubscriptionOptions options;
    options.content_filter_options.filter_expression = filter_string;
    options.content_filter_options.expression_parameters = {std::to_string(name_space_id)};
    
    status_sub_ = this->create_subscription<gbeam2_interfaces::msg::Status>("/status", 1,
              std::bind(&CooperationNode::statusCallback, this, std::placeholders::_1),//);
              options);

    //PUBLISHING TOPICS
    assigned_graph_pub_ = this->create_publisher<gbeam2_interfaces::msg::Graph>(
      "coop/assigned_graph",1);

    clusters_pub_ =this->create_publisher<gbeam2_interfaces::msg::GraphCluster>(
          "coop/Globalclusters",1);

    start_coop_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_coop",std::bind(&CooperationNode::startCooperation,this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&CooperationNode::navigationCallback, this));

    //cluster_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&CooperationNode::CreateClusterGraph, this));

     // Initialize parameters
    this->declare_parameter<int>("N_robot",0);
    this->declare_parameter<double>("communication_range",0.0);
    this->declare_parameter<double>("elipse_scaling_obs",0.0);
    this->declare_parameter<double>("elipse_scaling_robot",0.0);

    // Get parameters
    N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
    wifi_range = this->get_parameter("communication_range").get_parameter_value().get<double>();

    RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF COOPERATION: ############# ");
    RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
    RCLCPP_INFO(this->get_logger(),"1) Number of robots: %d",N_robot);
    RCLCPP_INFO(this->get_logger(),"2) Communication range: %f",wifi_range);

    // Initialize vectors with the correct size
    stored_Graph.resize(N_robot);
    last_status.resize(N_robot);
    cluster_l2g_index.resize(N_robot);
    last_status[name_space_id].robot_id = name_space_id;
    for (int i = 0; i < N_robot; i++)
    {
      stored_Graph[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
      last_status[i].connection_status.resize(N_robot);
      last_status[i].joint_vector.resize(N_robot);
      last_status[i].normal_joint_vector.resize(N_robot);
      //cluster_l2g_index[i][0] = i;
    }

  }

private:
  std::string name_space;
  int name_space_id;

  // Parameters variables
  int N_robot;
  double wifi_range;

  // Storage of external information 
  std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> stored_Graph;
  gbeam2_interfaces::msg::GraphCluster GlobalClusters;
  std::vector<std::unordered_map<int,int>> cluster_l2g_index; 
  int last_updated_node;
  int last_updated_edge;
  int last_updated_cluster;
  bool start_coop = false;


  //Navigation variables
  int last_selected_cluster=-1;
 


  std::vector<gbeam2_interfaces::msg::Status> last_status;
  nav_msgs::msg::Odometry robot_odom_;
  geometry_msgs::msg::PointStamped robot_pos;


  //adjacency matrix is related to that graph
  gbeam2_interfaces::msg::Graph merged_graph; 
  gbeam2_interfaces::msg::Graph assigned_graph;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Declare topics variables
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr merged_graph_sub_;
  rclcpp::Subscription<gbeam2_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr assigned_graph_pub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::GraphCluster>::SharedPtr clusters_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  start_coop_service_;

  //rclcpp::TimerBase::SharedPtr cluster_timer_;

  double deg_90 = M_PI / 2.0;
  double deg_30 = M_PI / 6.0;



  void printUnorderedMap(const rclcpp::Logger& logger, const std::unordered_map<int, int>& map,  const std::string& map_name = "UnorderedMap") {
      std::ostringstream oss;
      oss << map_name << ": {";

      for (const auto& pair : map) {
          oss << pair.first << ": " << pair.second << ", ";
      }

      std::string map_string = oss.str();
      // Remove trailing comma and space, if the map is not empty
      if (!map.empty()) {
          map_string.pop_back();
          map_string.pop_back();
      }

      map_string += "}";
      RCLCPP_INFO(logger, "%s", map_string.c_str());
  }


  void printMatrix(rclcpp::Logger logger, const std::vector<std::vector<float>> &matrix, const std::string &matrix_name = "Matrix") {
    RCLCPP_INFO(logger, "Printing %s:", matrix_name.c_str());
    
    // Determine the maximum width for each column
    std::vector<size_t> column_widths;
    for (const auto& row : matrix) {
        if (row.size() > column_widths.size()) {
            column_widths.resize(row.size(), 0);
        }
        for (size_t j = 0; j < row.size(); ++j) {
            std::ostringstream temp;
            temp << std::fixed << std::setprecision(2) << row[j];
            column_widths[j] = std::max(column_widths[j], temp.str().size());
        }
    }

    // Print each row with formatted spacing
    for (size_t i = 0; i < matrix.size(); ++i) {
        std::ostringstream row_stream;
        row_stream << "[ ";
        for (size_t j = 0; j < matrix[i].size(); ++j) {
            row_stream << std::setw(column_widths[j]) << std::fixed << std::setprecision(2) << matrix[i][j];
            if (j < matrix[i].size() - 1) {
                row_stream << ", ";
            }
        }
        row_stream << " ]";
        RCLCPP_INFO(logger, "Row %zu: %s", i, row_stream.str().c_str());
    }
    }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
  {
      robot_odom_ = *odom_ptr;
      robot_pos.point.x = robot_odom_.pose.pose.position.x;
      robot_pos.point.y = robot_odom_.pose.pose.position.y;
      robot_pos.point.z = robot_odom_.pose.pose.position.z;
      //RCLCPP_INFO(this->get_logger(),"odom received: x:%f y:%f",robot_odom_.pose.pose.position.x,robot_odom_.pose.pose.position.y);
  }
  
  gbeam2_interfaces::msg::Graph compareUpdates(
        const std::shared_ptr<const gbeam2_interfaces::msg::Graph>& current_graph,
        const std::shared_ptr<const gbeam2_interfaces::msg::Graph>& previous_graph)
    {
        gbeam2_interfaces::msg::Graph result;
        int last_node_index = previous_graph->nodes.size();
        int last_edge_index = previous_graph->edges.size();

        // Reserve space for potential changes (this is a rough estimate)
        result.nodes.reserve(current_graph->nodes.size() - last_node_index);
        result.edges.reserve(current_graph->edges.size() - last_edge_index);

        // Check for modifications in existing NODES
        for (int i = 0; i < last_node_index; ++i) {
            if (hasVertexChanged(current_graph->nodes[i], previous_graph->nodes[i])) {
                result.nodes.push_back(current_graph->nodes[i]);
            }
        }

        // Add new nodes
        result.nodes.insert(
            result.nodes.end(),
            std::make_move_iterator(current_graph->nodes.begin() + last_node_index),
            std::make_move_iterator(current_graph->nodes.end())
        );

        // Check for modifications in existing EDGES
        for (int i = 0; i < last_edge_index; ++i) {
            if (hasEdgeChanged(current_graph->edges[i], previous_graph->edges[i])) {
                result.edges.push_back(current_graph->edges[i]);
            }
        }

        // Add new edges
        result.edges.insert(
            result.edges.end(),
            std::make_move_iterator(current_graph->edges.begin() + last_edge_index),
            std::make_move_iterator(current_graph->edges.end())
        );

        result.adj_matrix = current_graph->adj_matrix;
        result.robot_id = current_graph->robot_id;


        return result;
    }

  void mergedGraphCallback(const std::shared_ptr<gbeam2_interfaces::msg::Graph> graph_received){
    
    bool new_nodes=false;
     
    int req_robot_id = graph_received->robot_id;
    last_updated_node = (stored_Graph[req_robot_id]->nodes.empty()) ? -1 : stored_Graph[req_robot_id]->nodes.back().id;
    last_updated_edge = (stored_Graph[req_robot_id]->edges.empty()) ? -1 : stored_Graph[req_robot_id]->edges.back().id;

    stored_Graph[req_robot_id]->cluster_graph = graph_received->cluster_graph;
  
    // Update node and edges for each graph update received
    for(gbeam2_interfaces::msg::Vertex node: graph_received->nodes){

      if(node.id<last_updated_node){
        //update old ones
        stored_Graph[req_robot_id]->nodes[node.id]=node;          
      }
      else{
        // adding new nodes

        //RCLCPP_INFO(this->get_logger(), "Add new node %d", node.id);
        stored_Graph[req_robot_id]->nodes.push_back(node);
        if(!node.is_obstacle){
          new_nodes = true;
        }
        
      }

    }
   

    for(gbeam2_interfaces::msg::GraphEdge edge : graph_received->edges){
      
      if(edge.id<last_updated_edge){
        stored_Graph[req_robot_id]->edges[edge.id]=edge;          
      }
      else{
        stored_Graph[req_robot_id]->edges.push_back(edge);
      }
    }    

    GlobalClusters.clusters.clear();

    // Calculate total size needed for global array
    size_t totalSize = 0;
    int offset_index=0;
    for (int i = 0; i < N_robot; i++) {
        totalSize += stored_Graph[i]->cluster_graph.clusters.size();
        if(i<req_robot_id){
          offset_index += totalSize;
        } 

        auto to_add = (i!=req_robot_id) ? stored_Graph[i]->cluster_graph.clusters :graph_received->cluster_graph.clusters;

        GlobalClusters.clusters.insert( GlobalClusters.clusters.end(), to_add.begin(), to_add.end() );  
        cluster_l2g_index[i].clear();   
    }

    std::vector<std::vector<float>> global_adj_matrix(GlobalClusters.clusters.size(), std::vector<float>(GlobalClusters.clusters.size(), -1.0));
    std::vector<std::vector<float>> belong_matrix(GlobalClusters.clusters.size(), std::vector<float>(GlobalClusters.clusters.size(), -1.0));

    for(int i=0; i<GlobalClusters.clusters.size(); i++){
      auto& cl_i = GlobalClusters.clusters[i];
      cluster_l2g_index[cl_i.belong_to][cl_i.cluster_id] = i;
    }
    //for (int i = 0; i < N_robot; i++) {printUnorderedMap(this->get_logger(),cluster_l2g_index[i],"mapped for robot"+std::to_string(i));}
    
    for(int i=0; i<GlobalClusters.clusters.size(); i++){
      auto& cl_i = GlobalClusters.clusters[i];
      int N = stored_Graph[cl_i.belong_to]->cluster_graph.clusters.size();

      // adj.data[i * N + j] = matrix[i][j]; matrix[i][j] = adj.data[i * N + j];
      for(auto neigh_id:cl_i.neighbours_centroids){
        auto el = stored_Graph[cl_i.belong_to]->cluster_graph.adj_matr.data[i*N + neigh_id];
        global_adj_matrix[i][cluster_l2g_index[cl_i.belong_to][neigh_id]] = (el!=0.0) ? el: -1.0;
               
        belong_matrix[i][cluster_l2g_index[cl_i.belong_to][neigh_id]] = cl_i.belong_to;    
      }

    }

    //printMatrix(this->get_logger(),global_adj_matrix);

    for(auto& bridge : stored_Graph[req_robot_id]->cluster_graph.bridges){
      global_adj_matrix[cluster_l2g_index[bridge.r1][bridge.c1]][cluster_l2g_index[bridge.r2][bridge.c2]] = bridge.id;
      global_adj_matrix[cluster_l2g_index[bridge.r2][bridge.c2]][cluster_l2g_index[bridge.r1][bridge.c1]] = bridge.id;
      //belong_matrix[cluster_l2g_index[bridge.belong_to][bridge.c1]][cluster_l2g_index[bridge.belong_to][bridge.c2]] = bridge.belong_to;
    }

    //printMatrix(this->get_logger(),global_adj_matrix);

    clusters_pub_->publish(GlobalClusters);

  }

  gbeam2_interfaces::msg::Graph getAssignedGraph(const std::vector<gbeam2_interfaces::msg::GraphClusterNode> clusters){
      gbeam2_interfaces::msg::Graph result;

      // How to deal with edges? 
      
      for(auto& cl:clusters){

      }
  }

  int getCurrentCluster(geometry_msgs::msg::PointStamped pos){
    // Return the id which has the nearest centroid from the pos specified in input
    double d_min=INF, d;
    int id_min=-1, i=0;
  
    for(auto& cl:GlobalClusters.clusters){
      d=dist(cl.centroid,pos);
      if(d<d_min){
        id_min=i;
        d_min =d;
      } 
      RCLCPP_INFO(this->get_logger(),"cluster %d --> id: %d dist: %f",i,cl.cluster_id,d);
      i++;
    }

    return id_min;
  }

  int getBestCluster(const int curr_cl_id){
    // Get the best cluster to explored based on the one in which i am
    // Here we should avoid to get access to occupied cluster from "status" topic

    // Get current cluster of other robots

    // Verify if it exists any unexplored nodes in the current cluster 

    // If yes, verify if is there a better cluster to explore, based on average and total gain
    // avoiding clusters occupied by other

  }

  void navigationCallback(){

    // Exploration based on second level Map taking into account also clusters of others robot
    bool AreaDivision_isrequired = false;

    if(last_selected_cluster<0){
      // Node is just started
      if(!GlobalClusters.clusters.empty()){
        if(!stored_Graph[name_space_id]->cluster_graph.clusters.empty()){

          RCLCPP_INFO(this->get_logger()," I would select the first cluster among mine");
          last_selected_cluster = 0;

        }else{
          // I have no cluster at all or I didn't already compute one
          RCLCPP_INFO(this->get_logger(),"I have no cluster at all or I didn't already compute one");
          if(stored_Graph[name_space_id]->nodes.size()>4){
            RCLCPP_INFO(this->get_logger(),"I have some nodes but they don't compose a cluster yet");
          }
          else{
            int curr_cl_id = getCurrentCluster(robot_pos);
            if(curr_cl_id!=-1){
              auto curr_cl = GlobalClusters.clusters[curr_cl_id];

              RCLCPP_INFO(this->get_logger(),"I'm in global cluster %d --> id: %d belong to: %d",curr_cl_id,curr_cl.cluster_id,curr_cl.belong_to);
              AreaDivision_isrequired=true;
            }

          }
          
        }

      }else{
        RCLCPP_WARN(this->get_logger(),"Can't select any cluster because there aren't any");
      }     
      

      
    }

    if(last_selected_cluster>0 && !AreaDivision_isrequired){
      //

      auto curr_cl = GlobalClusters.clusters[last_selected_cluster];
      int best_cl_id = getBestCluster(last_selected_cluster);

      // is the current cluster the best? 
      // BEST CLUSTER SEARCH 

      if(last_selected_cluster!=best_cl_id){
        auto best_cl = GlobalClusters.clusters[last_selected_cluster];
        if(best_cl.belong_to!=name_space_id){
          last_selected_cluster = best_cl_id;
          AreaDivision_isrequired = true;
          RCLCPP_INFO(this->get_logger(),"AREA DIVISION REQUIRED for selected cluster of robot%d with id: %d",GlobalClusters.clusters[last_selected_cluster].belong_to, last_selected_cluster);
        }else{
          RCLCPP_INFO(this->get_logger(),"Select cluster of mine with id: %d",last_selected_cluster);
        }
      }else{
        RCLCPP_INFO(this->get_logger(),"Mantain last selected id: %d",last_selected_cluster);
      }
      
      
    }

    // If the selected current belong to an other robot:
    // PAIRWISE AREA DIVISION 
    // Client and service call to be pairwise syncronized 

    if(AreaDivision_isrequired){

    }


    

    // Publish to status the current cluster

    // Build a graph with only selected cluster and publish it to the explorer 

  }


  

  

  void startCooperation(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
      start_coop=request->data; 

  }

  void statusCallback(const gbeam2_interfaces::msg::Status::SharedPtr received_status){
    last_status[received_status->robot_id]=*received_status;
    if(!start_coop) return;


  }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CooperationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
