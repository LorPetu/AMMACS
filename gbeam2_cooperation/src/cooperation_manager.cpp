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
#include <queue>

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
    current_cluster_pub_ = this->create_publisher<gbeam2_interfaces::msg::GraphClusterNode>(
      "current_cluster",1);

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
  int last_selected_cluster_id=-1;
  gbeam2_interfaces::msg::GraphClusterNode last_selected_cluster;
 


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
  rclcpp::Publisher<gbeam2_interfaces::msg::GraphClusterNode>::SharedPtr current_cluster_pub_;
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

      for(int j=i+1;j<GlobalClusters.clusters.size(); j++){
        auto& cl_j = GlobalClusters.clusters[j];
        if(cl_i.belong_to==cl_j.belong_to){
          auto el = stored_Graph[cl_i.belong_to]->cluster_graph.length_matrix.data[i*N + j];
          global_adj_matrix[i][j] = (el!=0.0) ? el: -1.0;
          global_adj_matrix[j][i] = global_adj_matrix[i][j];
          //belong_matrix[i][cluster_l2g_index[cl_i.belong_to][neigh_id]] = cl_i.belong_to;    
        }else{
          // Bridges
        }
      }
    }


    //printMatrix(this->get_logger(),global_adj_matrix);

    for(auto& bridge : stored_Graph[req_robot_id]->cluster_graph.bridges){
      global_adj_matrix[cluster_l2g_index[bridge.r1][bridge.c1]][cluster_l2g_index[bridge.r2][bridge.c2]] = bridge.length;
      global_adj_matrix[cluster_l2g_index[bridge.r2][bridge.c2]][cluster_l2g_index[bridge.r1][bridge.c1]] = bridge.length;
      //belong_matrix[cluster_l2g_index[bridge.belong_to][bridge.c1]][cluster_l2g_index[bridge.belong_to][bridge.c2]] = bridge.belong_to;
    }

    GlobalClusters.bridges = stored_Graph[req_robot_id]->cluster_graph.bridges;
    GlobalClusters.adj_matrix = matrix2GraphAdj(global_adj_matrix);

    printMatrix(this->get_logger(),global_adj_matrix);

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

  std::pair<std::vector<int>,double> dijkstraWithAdjandPath(gbeam2_interfaces::msg::GraphCluster graph, int s, int t)
{
  int N = graph.clusters.size();
  int E = graph.adj_matrix.size;
  auto adjMatrix = graph.adj_matrix.data;
  //RCLCPP_INFO(this->get_logger(),"Size of adjacency matrix: %d",N);

  // Since we're considering only reachable node we skip the filtering part.

  std::vector<double> dist(N, INF);
  std::vector<int> parent(N, -1); // To store the shortest path tree
  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

  std::vector<int> path;

  dist[s] = 0;
  pq.push({0, s});

  while (!pq.empty()) {
        auto [currentDist, u] = pq.top();
        pq.pop();

        // If the distance is already larger, skip
        if (currentDist > dist[u]) continue;
        auto cl_u = graph.clusters[u];
        // Explore neighbors
        //RCLCPP_INFO(this->get_logger(),"DIJSKTRA: Searching in neighbourhood");
        for (int v=0; v<N;v++) {
          if(u!=v && adjMatrix[u*N + v]!=-1.0){
            //RCLCPP_INFO(this->get_logger(),"DIJSKTRA: get access to neigh: %d of cluster %d", v,u);
            auto cl_v = graph.clusters[v];
            double weight; 
            //RCLCPP_INFO(this->get_logger(),"DIJSKTRA:  %d belong to robot%d and %d belong to robot%d", v,cl_v.belong_to,u,cl_u.belong_to);
            if(cl_u.belong_to==cl_v.belong_to){
              
              weight = adjMatrix[u*N + v]; // adj.data[i * N + j] = matrix[i][j];
              //RCLCPP_INFO(this->get_logger(),"DIJSKTRA: weight matrix i:%d j%d %f",u,v, weight);
            }else{
              // Get access to bridges
              //RCLCPP_INFO(this->get_logger(),"DIJSKTRA: bridges");
              weight = graph.bridges[adjMatrix[u*N + v]].length;
            }

            if (weight > 0 && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push({dist[v], v});
            }

          }
            
        }
    }

    
    for (int at = t; at != -1; at = parent[at]) {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end()); // Reverse to get the path from source to target

    // Check if the path starts with the source
    if (!path.empty() && path[0] == s) {
        return std::make_pair(path,dist[t]);
    }
    return {}; // Return empty if there's no valid path

    return std::make_pair(path,dist[t]);
}

  int getBestCluster(const int curr_cl_id){
    // Get the best cluster to explored based on the one in which i am
    // Here we should avoid to get access to occupied cluster from "status" topic
    std::vector<int> occupied_clusters;
    float exp_distance = 1.0;
    double best_reward=-1.0;
    int best_cluster_id=-1;
    std::vector<int> best_path;
    
    

    /*for(int z=0; z<N_robot;z++){
      if(z!=name_space_id){
        RCLCPP_INFO(this->get_logger(),"Cluster occupied by %d is local id:%d global id: %d",z,last_status[z].current_cluster.cluster_id,cluster_l2g_index[last_status[z].current_cluster.belong_to][last_status[z].current_cluster.cluster_id]);
        occupied_clusters.push_back(cluster_l2g_index[last_status[z].current_cluster.belong_to][last_status[z].current_cluster.cluster_id]);
        // What if i'm tryng to map an external cluster that I don't have?
      }
    }*/
   RCLCPP_INFO(this->get_logger(),"Start searching for best cluster");

   for(int i=0; i<GlobalClusters.clusters.size(); i++){
      //      if (std::find(cluster.nodes.begin(), cluster.nodes.end(), neighbor_id) != cluster.nodes.end()) {
        //RCLCPP_INFO(this->get_logger(),"cluster: %d Start searching for best cluster",i);
  
        auto [path, distance] = (i!=last_selected_cluster_id) ? dijkstraWithAdjandPath(GlobalClusters,curr_cl_id,i):std::make_pair(std::vector<int>{last_selected_cluster_id}, 0.01);;
        
        
        if(path.size()>0){
          double reward_avg = (GlobalClusters.clusters[i].total_gain/GlobalClusters.clusters[i].unexplored_nodes.size()) / pow(distance,exp_distance);
          RCLCPP_INFO(this->get_logger(),"CLUSTER: %d - average reward: %f - distance %f - total_gain: %f - number of unexplored nodes: %d",
                                                    i, reward_avg,distance,GlobalClusters.clusters[i].total_gain,GlobalClusters.clusters[i].unexplored_nodes.size());
          if(reward_avg>best_reward){
            best_cluster_id = i;
            best_path = path;
            best_reward = reward_avg;
            
          }
        }
        
      
    }

    RCLCPP_INFO(this->get_logger(),"BEST CLUSTER: %d - average reward: %f - %d clusters away",best_cluster_id, best_reward, best_path.size());

    std::string path_str;
    for (int node : best_path) {
        path_str += std::to_string(node) + "-";
    }

    RCLCPP_INFO(this->get_logger(), "BEST CLUSTER: Path computed with adj is: %s", path_str.c_str());


    //   if(std::find(occupied_clusters.begin(),occupied_clusters.end(),i) != occupied_clusters.end()){
    //     continue;
    //   }else{
    
    // }

    // Get current cluster of other robots

    // Verify if it exists any unexplored nodes in the current cluster 

    // If yes, verify if is there a better cluster to explore, based on average and total gain
    // avoiding clusters occupied by other

    return best_cluster_id;

  }

  void navigationCallback(){
    // Exploration based on second level Map taking into account also clusters of others robot
    
    bool AreaDivision_isrequired = false;

    if(last_selected_cluster_id<0){
      // Node is just started
      if(!GlobalClusters.clusters.empty()){
        if(!stored_Graph[name_space_id]->cluster_graph.clusters.empty()){

          RCLCPP_INFO(this->get_logger()," I would select the first cluster among mine");
          last_selected_cluster_id = cluster_l2g_index[name_space_id][0];
          last_selected_cluster = GlobalClusters.clusters[last_selected_cluster_id];
          current_cluster_pub_->publish(last_selected_cluster);
          return;

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

    if(last_selected_cluster_id>=0 && !AreaDivision_isrequired){
      //
      // Get updated id of the last selected cluster, since it changes during local copy of the global map
      last_selected_cluster_id = cluster_l2g_index[last_selected_cluster.belong_to][last_selected_cluster.cluster_id];
      //int best_cl_id = last_selected_cluster_id;
      int best_cl_id = getBestCluster(last_selected_cluster_id);

      // is the current cluster the best? 
      // BEST CLUSTER SEARCH 
      //RCLCPP_INFO(this->get_logger(),"exit from getBestCluster function");
      if(last_selected_cluster_id!=best_cl_id){
        RCLCPP_INFO(this->get_logger(),"last selected cluster: %d ",last_selected_cluster_id);
        auto best_cl = GlobalClusters.clusters[last_selected_cluster_id];
        //RCLCPP_INFO(this->get_logger(),"Global cluster array is ok:");
        if(best_cl.belong_to!=name_space_id){
          last_selected_cluster_id = best_cl_id;
          AreaDivision_isrequired = true;
          RCLCPP_INFO(this->get_logger(),"AREA DIVISION REQUIRED for selected cluster of robot%d with id: %d",GlobalClusters.clusters[last_selected_cluster_id].belong_to, last_selected_cluster_id);
        }else{
          RCLCPP_INFO(this->get_logger(),"Select cluster of mine with id: %d",last_selected_cluster_id);
        }
      }else{
        RCLCPP_INFO(this->get_logger(),"Mantain last selected id: %d",last_selected_cluster_id);
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
