//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <utility>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <thread>
#include <optional>

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

#include "gbeam2_interfaces/action/assigned_task.hpp"
#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/graph_cluster_node.hpp"
#include "gbeam2_interfaces/msg/global_map.hpp"
#include "library_fcn.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // For easier point cloud population


#define INF 100000

class CooperationNode : public rclcpp::Node
{
public:

  using Task = gbeam2_interfaces::action::AssignedTask;
  explicit CooperationNode() : Node("coop_manager"){

    //SUBSCRIBED TOPICS
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            name_space+ "odom", 1, std::bind(&CooperationNode::odomCallback, this, std::placeholders::_1));

    merged_graph_sub_= this->create_subscription<gbeam2_interfaces::msg::GlobalMap>(
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
      "coop/current_cluster",1);

    start_coop_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_coop",std::bind(&CooperationNode::startCooperation,this, std::placeholders::_1, std::placeholders::_2));

    try_clustering_service_ = this->create_client<std_srvs::srv::SetBool>("gbeam/start_cluster");

    //timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&CooperationNode::navigationCallback, this));
    
    // Create Action client for exploration node
    this->action_client_ = rclcpp_action::create_client<Task>(
      this,
      "Task"
    );
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
    RCLCPP_INFO(this->get_logger(),"2) Communication range: %.2f",wifi_range);

    // Initialize vectors with the correct size
    //stored_Graph.resize(N_robot);
    last_status.resize(N_robot);
    cluster_l2g_index.resize(N_robot);
    last_status[name_space_id].robot_id = name_space_id;
    for (int i = 0; i < N_robot; i++)
    {
      //stored_Graph->map[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
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
  gbeam2_interfaces::msg::GlobalMap::SharedPtr stored_Graph;
  gbeam2_interfaces::msg::GraphCluster GlobalClusters;
  std::vector<std::unordered_map<int,int>> cluster_l2g_index; 
  int last_updated_node;
  int last_updated_edge;
  int last_updated_cluster;
  bool start_coop = false;
  int prev_clusters_size = 0;
  bool execute_nav= false; 
  float tot_global_gain=-1;
  int min_unexpl_size=3;

  double gamma=0.5;

  bool recursive=false;


  //Navigation variables
  // Local indexes 
  int start=-1, curr=-1, target=-1;
  // Global clusters
  gbeam2_interfaces::msg::GraphClusterNode start_cl, curr_cl, target_cl; 


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
  rclcpp::Subscription<gbeam2_interfaces::msg::GlobalMap>::SharedPtr merged_graph_sub_;
  rclcpp::Subscription<gbeam2_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr assigned_graph_pub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::GraphCluster>::SharedPtr clusters_pub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::GraphClusterNode>::SharedPtr current_cluster_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  start_coop_service_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr  try_clustering_service_;
  rclcpp_action::Client<Task>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<Task>::SharedPtr current_goal_handle_;

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
      //RCLCPP_INFO(this->get_logger(),"odom received: x:%.2f y:%.2f",robot_odom_.pose.pose.position.x,robot_odom_.pose.pose.position.y);
  }
  
  void startCooperation(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
      start_coop=request->data; 
      navigationCallback(false);
  }

  void statusCallback(const gbeam2_interfaces::msg::Status::SharedPtr received_status){
    if(received_status->connection_status[name_space_id]!=1) return;
    last_status[received_status->robot_id]=*received_status;
    if(!start_coop) return;


  }

  void computeClusterProperties2(gbeam2_interfaces::msg::GraphClusterNode& it){
        // Compute centroid and total gain for the pointer to the cluster
        
        //std::vector<gbeam2_interfaces::msg::GraphClusterNode, std::allocator<gbeam2_interfaces::msg::GraphClusterNode>>::iterator it
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        double expl_sum_x = 0.0, expl_sum_y = 0.0, expl_sum_z = 0.0;
        double tot_gain=0.0;
        for(auto& node_id:it.nodes){
            auto node_gain = stored_Graph->map[it.belong_to].nodes[node_id].gain;

            // Geometrical centroid
            sum_x += stored_Graph->map[it.belong_to].nodes[node_id].x; //*node_gain;
            sum_y += stored_Graph->map[it.belong_to].nodes[node_id].y; //*node_gain;
            sum_z += stored_Graph->map[it.belong_to].nodes[node_id].z; //*node_gain;

            //  Exploration weighted centroid
            expl_sum_x += stored_Graph->map[it.belong_to].nodes[node_id].x*node_gain;
            expl_sum_y += stored_Graph->map[it.belong_to].nodes[node_id].y*node_gain;
            expl_sum_z += stored_Graph->map[it.belong_to].nodes[node_id].z*node_gain;

            tot_gain+=node_gain;
        }
        it.total_gain=tot_gain;
        it.neighbours_centroids.clear();

        int count = it.nodes.size();
        
        if (count>0){
            // Geometrical centroid
            if (tot_gain > 1e-6) {  // Avoid division by zero
                it.expl_centroid.x = expl_sum_x / tot_gain;
                it.expl_centroid.y = expl_sum_y / tot_gain;
                it.expl_centroid.z = expl_sum_z / tot_gain;
            } else {
                it.expl_centroid.x = it.centroid.x;
                it.expl_centroid.y = it.centroid.y;
                it.expl_centroid.z = it.centroid.z;
            }

        }
        
    }

  void mergedGraphCallback(const std::shared_ptr<gbeam2_interfaces::msg::GlobalMap> global_map_received){
    // This function creates the clusters layer graph considering all the cluster of all the map in global map
    // and it's executed every new updates of the global map.
    // To make easier processing all the cluster are added and id remapped from 0 to total size of clusters
    // in global map. cluster_l2g_index[n][i]=j is a vector of unordered map to obtain the j id of global cluster
    // of cluster i of robot n. 
    // It enables the decision making in NavigationCallback()   

    stored_Graph = global_map_received;
    int req_robot_id = stored_Graph->last_updater;
    auto& graph_received = stored_Graph->map[req_robot_id];
    //RCLCPP_INFO(this->get_logger(), "Received global Map last updated by robot%d",req_robot_id);
    bool new_nodes=false;
    int curr_tot_global_gain=0;
       
    GlobalClusters.clusters.clear();

    // Calculate total size needed for global array
    size_t totalSize = 0;
    int offset_index=0;
    for (int i = 0; i < N_robot; i++) {
        totalSize += stored_Graph->map[i].cluster_graph.clusters.size();
        if(i<req_robot_id){
          offset_index += totalSize;
        } 

        auto to_add = (i!=req_robot_id) ? stored_Graph->map[i].cluster_graph.clusters :graph_received.cluster_graph.clusters;

        GlobalClusters.clusters.insert( GlobalClusters.clusters.end(), to_add.begin(), to_add.end() );  
        cluster_l2g_index[i].clear();   
    }

    // RCLCPP_INFO(this->get_logger(),"Global Cluster size: %d - Clusters of 0 = %d - Clusters of 1 = %d",GlobalClusters.clusters.size()
    //   ,stored_Graph->map[0].cluster_graph.clusters.size(),stored_Graph->map[1].cluster_graph.clusters.size());

    std::vector<std::vector<float>> global_adj_matrix(GlobalClusters.clusters.size(), std::vector<float>(GlobalClusters.clusters.size(), -1.0));
    std::vector<std::vector<int>> number_of_bridges(GlobalClusters.clusters.size(), std::vector<int>(GlobalClusters.clusters.size(), 0));

    bool should_decrease = false;  // Flag to start decreasing min_unexpl_size

    for (int i = 0; i < GlobalClusters.clusters.size(); i++) {
        auto& cl_i = GlobalClusters.clusters[i];
        computeClusterProperties2(cl_i);  // Compute properties of the cluster

        // if (cl_i.total_gain > 0.0) {
        //     // Check if any unexplored clusters remain larger than min_unexpl_size
        //     if (cl_i.nodes.size() > min_unexpl_size) {
        //         should_decrease = false;  // There's still a larger unexplored cluster
        //     } else {
        //         should_decrease = true;   // All unexplored clusters are now <= min_unexpl_size
        //     }

        //     // Update min_unexpl_size only when larger clusters still exist
        //     if (!should_decrease) {
        //         min_unexpl_size = std::min(min_unexpl_size, static_cast<int>(cl_i.nodes.size()));
        //     }
        // }

        curr_tot_global_gain += cl_i.total_gain;
        cluster_l2g_index[cl_i.belong_to][cl_i.cluster_id] = i;
    }

    // // If all unexplored clusters are now <= min_unexpl_size, allow it to decrease
    // if (should_decrease) {
    //     min_unexpl_size--;  // Reduce minimum unexplored size
    // }

    //for (int i = 0; i < N_robot; i++) {printUnorderedMap(this->get_logger(),cluster_l2g_index[i],"mapped for robot"+std::to_string(i));}
    
    // Edges between clusters of the same robots
    for(int i=0; i<GlobalClusters.clusters.size(); i++){
      auto& cl_i = GlobalClusters.clusters[i];
      int N = stored_Graph->map[cl_i.belong_to].cluster_graph.clusters.size();

      for(int j=i;j<GlobalClusters.clusters.size(); j++){
        auto& cl_j = GlobalClusters.clusters[j];
        if(cl_i.belong_to==cl_j.belong_to){
          auto local_index = cl_i.cluster_id*N+cl_j.cluster_id;
          if (local_index >= stored_Graph->map[cl_i.belong_to].cluster_graph.length_matrix.data.size()) {
              // RCLCPP_ERROR(this->get_logger(), "Out-of-bounds access in length_matrix.data at index %d", local_index);
          }

          auto el = stored_Graph->map[cl_i.belong_to].cluster_graph.length_matrix.data[local_index];

          // Compute euclidean distance between expl_centroid
          float dist_ij = dist(cl_i.expl_centroid,cl_j.expl_centroid);

          global_adj_matrix[i][j] = (el>1e-4) ? dist_ij : -1.0;
         
          global_adj_matrix[j][i] = global_adj_matrix[i][j];  
        }else{
          // Bridges
          global_adj_matrix[j][i] = -1.0;
        }
      }
    }


    // RCLCPP_INFO(this->get_logger(),"Starting processing bridges...");
    // Edges between clusters of the different robots
    for(auto& bridge : stored_Graph->map[req_robot_id].cluster_graph.bridges){
      int i_gl = cluster_l2g_index[bridge.r1][bridge.c1]; 
      int j_gl = cluster_l2g_index[bridge.r2][bridge.c2];
      auto& cl_i = GlobalClusters.clusters[i_gl];
      auto& cl_j = GlobalClusters.clusters[j_gl];

      number_of_bridges[i_gl][j_gl] +=1;

      // RCLCPP_INFO(this->get_logger(),"Checking connection %d - %d n_bridges: %d",i_gl,j_gl, number_of_bridges[i_gl][j_gl]);

      auto& bridge_end_i = stored_Graph->map[bridge.r1].nodes[bridge.v1];
      auto& bridge_end_j = stored_Graph->map[bridge.r2].nodes[bridge.v2];
      // Compute euclidean distance between expl_centroid considering the bridges

      float dist_ij = dist(cl_i.expl_centroid,bridge_end_i) + bridge.length + dist(cl_j.expl_centroid,bridge_end_j);

      auto prev_dist = (global_adj_matrix[i_gl][j_gl]<0.0) ? INF : global_adj_matrix[i_gl][j_gl];

      // RCLCPP_INFO(this->get_logger(), "prev_dist: %.2f, dist_ij: %.2f", prev_dist, dist_ij);

      
      global_adj_matrix[i_gl][j_gl] = (dist_ij < prev_dist) ? dist_ij : prev_dist;
      global_adj_matrix[j_gl][i_gl] = global_adj_matrix[i_gl][j_gl];

      if(global_adj_matrix[i_gl][j_gl]==INF){
        RCLCPP_WARN(this->get_logger(),"BRIDGE id: %d from (n: %d cl: %d of R%d ) to (n: %d cl: %d of R%d) length %.2f ", 
                                                            bridge.id,
                                                            bridge.v1, bridge.c1, bridge.r1,
                                                            bridge.v2, bridge.c2, bridge.r2, bridge.length);
      }
      
    }

    GlobalClusters.bridges = stored_Graph->map[req_robot_id].cluster_graph.bridges;
    // RCLCPP_INFO(this->get_logger(),"Size of bridges in Global Clusters: %d",GlobalClusters.bridges.size());
    GlobalClusters.adj_matrix = matrix2GraphAdj(global_adj_matrix);

    //printMatrix(this->get_logger(),global_adj_matrix);

    clusters_pub_->publish(GlobalClusters);

    if(curr_tot_global_gain>0.0){
      prev_clusters_size = GlobalClusters.clusters.size();
      tot_global_gain = curr_tot_global_gain;
      // Check if a goal is currently executing
      if (current_goal_handle_)
      {
        auto status = current_goal_handle_->get_status();
        if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
            status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
        {
            RCLCPP_WARN(this->get_logger(), "A goal is already executing. Not sending a new goal.");
            return;
        }else{
          RCLCPP_WARN(this->get_logger(), "No goal is executing, recompute a new target cluster");
          navigationCallback(false);
        }
      }

      
    }else{
      RCLCPP_INFO_ONCE(this->get_logger(),"ZERO TOT GLOBAL GAIN, exploration completed");
    }


  }

  gbeam2_interfaces::msg::Graph getAssignedGraph(std::vector<int> clusters_ids){
      // we process cluster by cluster the path to get to the objective 
      // and all the useful adjacency matrix for navigation
      // Also shortest bridges are selected if needed

      // Check if the curr cluster is included 

      if(std::find(clusters_ids.begin(),clusters_ids.end(),curr) == clusters_ids.end()){
        clusters_ids.insert(clusters_ids.begin(),curr);
        RCLCPP_WARN(this->get_logger(),"getAssignedGraph:: current cluster %d was not included in assigned graph!",curr);
      }

      gbeam2_interfaces::msg::Graph result;
      result.cluster_graph.bridges.clear();
      // TODO: use clusters_ids.size() - 1 and remove condition
      for(int i=0; i<clusters_ids.size(); i++){
        auto cl = GlobalClusters.clusters[clusters_ids[i]];
        RCLCPP_INFO(this->get_logger(),"getAssignedGraph:: Add all the %d nodes of cluster %d",cl.nodes.size(),cl.cluster_id);
        result.cluster_graph.clusters.push_back(cl);
        for(auto& node_id:cl.nodes){
          result.nodes.push_back(stored_Graph->map[cl.belong_to].nodes[node_id]);
        }

        if(i+1 <clusters_ids.size()){
          auto next = GlobalClusters.clusters[clusters_ids[i+1]];
          if(next.belong_to!=cl.belong_to){
            gbeam2_interfaces::msg::Bridge sel_bridge;
            sel_bridge.length =INF;
            // Select a bridge to pass from cl to next
            for(auto& bridge:GlobalClusters.bridges){
              if(bridge.r1 == cl.belong_to && bridge.r2 == next.belong_to && 
                bridge.c1 == cl.cluster_id && bridge.c2 == next.cluster_id){
                  
                  if(bridge.length < sel_bridge.length) sel_bridge=bridge;


              }else if (bridge.r2 == cl.belong_to && bridge.r1 == next.belong_to &&
                        bridge.c2 == cl.cluster_id && bridge.c1 == next.cluster_id)
              { 
                if(bridge.length < sel_bridge.length) sel_bridge=bridge;
            
              }
            }

            result.cluster_graph.bridges.push_back(sel_bridge);
          }
        }
      }
      //RCLCPP_INFO(this->get_logger(),"getAssignedGraph:: Total nodes: %d",result.nodes.size());
      auto res_robot_id = GlobalClusters.clusters[clusters_ids.back()].belong_to;
      result.adj_matrix = stored_Graph->map[res_robot_id].adj_matrix;
      result.length_matrix = stored_Graph->map[res_robot_id].length_matrix;
      // Assign the robot_id of the last cluster in the path
      result.robot_id = res_robot_id;

      return result;
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
      //RCLCPP_INFO(this->get_logger(),"cluster %d --> id: %d dist: %.2f",i,cl.cluster_id,d);
      i++;
    }

    return id_min;
  }

  std::pair<std::vector<int>,double> dijkstraWithAdjandPath(gbeam2_interfaces::msg::GraphCluster graph, int s, int t){
    int N = graph.adj_matrix.size; //graph.clusters.size();
    auto adjMatrix = graph.adj_matrix.data;
    ////RCLCPP_INFO(this->get_logger(),"Size of adjacency matrix: %d",N);

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
              double weight = adjMatrix[u*N + v]; // adj.data[i * N + j] = matrix[i][j];
              //RCLCPP_INFO(this->get_logger(),"DIJSKTRA:  %d belong to robot%d and %d belong to robot%d", v,cl_v.belong_to,u,cl_u.belong_to);
              // if(cl_u.belong_to==cl_v.belong_to){
                
              //   weight = adjMatrix[u*N + v]; // adj.data[i * N + j] = matrix[i][j];
              //   //RCLCPP_INFO(this->get_logger(),"DIJSKTRA: weight matrix i:%d j%d %.2f",u,v, weight);
              // }else{
              //   // Get access to bridges
              //   //RCLCPP_INFO(this->get_logger(),"DIJSKTRA: bridges");
              //   weight = graph.bridges[adjMatrix[u*N + v]].length;
              // }

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

  double getInnerDistance(gbeam2_interfaces::msg::GraphClusterNode cl){
    double max_dist = 0.25;
    
    return cl.unexplored_nodes.size()*(max_dist/cl.nodes.size());

  }

  std::pair<int, std::vector<int>> getBestCluster(const int start,std::vector<int> occupied_clusters){
    // Get the best cluster to explored based on the one in which i am
    // Here we should avoid to get access to occupied cluster from "status" topic
    
    float exp_distance = 1.0;
    double dist_scaling = 10.0; 
    double best_reward =-1.0;
    int best_cluster_id=-1;
    std::vector<int> best_path;
    
    RCLCPP_INFO(this->get_logger(),"BEST CLUSTER: Start searching for best cluster - min unexpl size: %d", min_unexpl_size);

    for(int i=0; i<GlobalClusters.clusters.size(); i++){
      auto& cl_i = GlobalClusters.clusters[i];
      if(std::find(occupied_clusters.begin(),occupied_clusters.end(),i) != occupied_clusters.end()){
        RCLCPP_INFO(this->get_logger(),"->%d: (C%d R%d) - Can't select it because is occupied", i,cl_i.cluster_id,cl_i.belong_to);
        continue;
      }else{        
        if(cl_i.nodes.size()<min_unexpl_size || cl_i.total_gain==0.0) continue; // Skip small clusters that can be merged TODO
          
          //RCLCPP_INFO(this->get_logger(),"cluster: %d Start searching for best cluster",i);
    
          auto [path, distance] = (i!=start) ? dijkstraWithAdjandPath(GlobalClusters,start,i) : std::make_pair(std::vector<int>{start}, getInnerDistance(start_cl));
          RCLCPP_INFO(this->get_logger(),"->%d: (C%d R%d) - path: %d - dist %.2f - tot_gain: %.2f - unexpl nodes: %d",
                                                     i,cl_i.cluster_id,cl_i.belong_to, path.size(),distance,cl_i.total_gain,cl_i.unexplored_nodes.size());
          
          if(path.size()>0){

            double dist_from_agent = std::accumulate(last_status.begin(), last_status.end(), 0.0,
                                      [cl_i](double sum, const gbeam2_interfaces::msg::Status& status) {
                                          geometry_msgs::msg::PointStamped p;
                                          p.point.x = status.last_known_target.pose.position.x;
                                          p.point.y = status.last_known_target.pose.position.y;
                                          p.point.z = status.last_known_target.pose.position.z;
                                          //status.connection_status[name_space_id]*
                                          return sum + dist(cl_i.centroid,p); 
                                      });


            double reward_avg = (cl_i.total_gain/cl_i.unexplored_nodes.size()) / pow(distance,exp_distance);
            
            //RCLCPP_INFO(this->get_logger(),"BEST CLUSTER: CLUSTER: %d - average reward: %.2f - distance %.2f - total_gain: %.2f - number of unexplored nodes: %d",
                                                    // i, reward_avg,distance,cl_i.total_gain,cl_i.unexplored_nodes.size());
            double cost = gamma*reward_avg + (1-gamma)*dist_scaling*dist_from_agent;

            RCLCPP_INFO(this->get_logger(),"->%d: (C%d R%d) - agent_dist: %.2f - reward_avg: %.2f - cost: %.2f - gamma: %.2f",
                                                    i,cl_i.cluster_id,cl_i.belong_to,dist_from_agent,reward_avg,cost,gamma);
            if(cost>best_reward){
              best_cluster_id = i;
              best_path = path;
              best_reward = cost;
              
            }
          }
          


        }
        
      
    }

    //RCLCPP_INFO(this->get_logger(),"BEST CLUSTER: %d - average reward: %.2f - %d clusters away",best_cluster_id, best_reward, best_path.size());

  

    // Get current cluster of other robots

    // Verify if it exists any unexplored nodes in the current cluster 

    // If yes, verify if is there a better cluster to explore, based on average and total gain
    // avoiding clusters occupied by other

    return std::make_pair(best_cluster_id, best_path);

  }

  void getOccupiedClusters(std::vector<int>& occupied_clusters){
    for(int z=0; z<N_robot;z++){
        if(z!=name_space_id){
          auto it = (cluster_l2g_index[last_status[z].current_cluster.belong_to].find(last_status[z].current_cluster.cluster_id));
          if(it!=cluster_l2g_index[last_status[z].current_cluster.belong_to].end()){
            RCLCPP_INFO(this->get_logger(),"Occupied Cluster || C%d R%d || by %d global index: %d",
                                            last_status[z].current_cluster.cluster_id,last_status[z].current_cluster.belong_to,z,it->second);

            occupied_clusters.push_back(it->second);
          }else{
            // What if i'm trying to map an external cluster that I don't have?
            RCLCPP_ERROR(this->get_logger(),"Robot %d is occupying a cluster that i cannot map");
          }
          
          
        }
      }
  }
  
  void navigationCallback(bool avoid_last_cluster){
    // Exploration based on second level Map taking into account also clusters of others robot

    std::vector<int> clusters_to_send;
    
    bool AreaDivision_isrequired = false;

    RCLCPP_INFO(this->get_logger(),"--------------------------------------------------");
    RCLCPP_INFO(this->get_logger(),"############## NEW CLUSTER EXPLORATION ##############");

    //printMatrix(this->get_logger(),GraphAdj2matrix(GlobalClusters.adj_matrix));
    // INITIALIZATION 
    if(start<0){
      // Node is just started
      if(!GlobalClusters.clusters.empty()){
        if(!stored_Graph->map[name_space_id].cluster_graph.clusters.empty()){

          RCLCPP_INFO(this->get_logger()," I would select the first cluster among mine");
          start = cluster_l2g_index[name_space_id][0];
          start_cl = GlobalClusters.clusters[start];
          // Since I'm already inside and i select it as the best target (the only one available)
          curr = start; target = start;
          curr_cl = start_cl; target_cl = start_cl;

          clusters_to_send.push_back(start);
          
          // Send GOAL
          send_goal(clusters_to_send,target_cl);

          target_cl.is_target = true;
          current_cluster_pub_->publish(target_cl);
          return;

        }else{
          // I have no cluster at all or I didn't already compute one
          RCLCPP_INFO(this->get_logger(),"++ I have no cluster at all or I didn't already compute one");
          if(stored_Graph->map[name_space_id].nodes.size()>4){
            RCLCPP_INFO(this->get_logger(),"+++I have some nodes but they don't compose a cluster yet");
          }
          else{
            curr = getCurrentCluster(robot_pos);
            if(curr!=-1){
              curr_cl = GlobalClusters.clusters[curr];

              RCLCPP_INFO(this->get_logger(),"+++ I'm in global cluster %d --> id: %d belong to: %d",curr,curr_cl.cluster_id,curr_cl.belong_to);
              AreaDivision_isrequired=true;
            }

          }
          
        }

      }else{
        RCLCPP_WARN(this->get_logger(),"+ Can't select any cluster because there aren't any");
      }     
      

      
    }

    // MAIN ROUTINE
    if(start>=0 && !AreaDivision_isrequired){
      //
      // Get updated id of the last selected cluster, since it changes during local copy of the global map
      target  = cluster_l2g_index[target_cl.belong_to][target_cl.cluster_id];
      start   = cluster_l2g_index[start_cl.belong_to][start_cl.cluster_id];
      curr    = cluster_l2g_index[curr_cl.belong_to][curr_cl.cluster_id];

      RCLCPP_INFO(this->get_logger(),"LAST_START: %d: C%d R%d || CURR: %d: C%d R%d || LAST_TARGET: %d: C%d R%d ||", 
                                      start,start_cl.cluster_id,start_cl.belong_to,
                                      curr,curr_cl.cluster_id,curr_cl.belong_to,
                                      target,target_cl.cluster_id,target_cl.belong_to);

      if(curr==target){
        // If the curr index is the same of the last target this mean that in the last goal I reach the target cluster
        // Restart from the last target
        start=target; start_cl=target_cl;


      }else if(curr==start){
        // If the curr index is the same of the last start this mean that I didn't move from the last cluster
        RCLCPP_INFO(this->get_logger(),"The current cluster is not the last target. Last Goal has not be completed!");

      }else{
        // Last goal was canceled during its execution
        start=curr; start_cl=curr_cl;
        RCLCPP_INFO(this->get_logger(),"The current cluster is not the last target. Last Goal has not be completed!");

      }

      RCLCPP_INFO(this->get_logger(),"START: %d: C%d R%d || CURR: %d: C%d R%d || TARGET: %d: C%d R%d ||", 
                                      start,start_cl.cluster_id,start_cl.belong_to,
                                      curr,curr_cl.cluster_id,curr_cl.belong_to,
                                      target,target_cl.cluster_id,target_cl.belong_to);

      

      std::vector<int> occupied_clusters; getOccupiedClusters(occupied_clusters);

      if(avoid_last_cluster){
        occupied_clusters.push_back(target);
        RCLCPP_INFO(this->get_logger(),"Avoid last selected cluster: %d",target);
      } 

      auto [best_cl_id,best_path] = getBestCluster(start,occupied_clusters);

      if(best_cl_id==-1){
        RCLCPP_WARN(this->get_logger(),"Didn't find any best cluster, try again after call a clustering");
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        try_clustering_service_->async_send_request(request);

        min_unexpl_size--;
        if(min_unexpl_size>0){
          navigationCallback(false);
        }else{
          RCLCPP_WARN(this->get_logger(),"No more unexplored clusters!");
        } 
      

        return;
      }

      auto best_cl = GlobalClusters.clusters[best_cl_id];

      RCLCPP_INFO(this->get_logger(),"+ Best Cluster selected: %d C%d R%d || - %d clusters away",
                                        best_cl_id, best_cl.cluster_id,best_cl.belong_to, best_path.size());

      gbeam2_interfaces::msg::Bridge bridge_to_send;

      clusters_to_send = best_path;
      std::string path_str;
      std::ostringstream owners; 
      for (auto id : clusters_to_send) {
          path_str += std::to_string(id) + "-";
          owners << " ( C" << GlobalClusters.clusters[id].cluster_id << " - R" <<GlobalClusters.clusters[id].belong_to << " ) -";
      }
      owners << "> STOP";
      RCLCPP_INFO(this->get_logger(),"+ Path: %s",owners.str().c_str());

      // is the current cluster the best? 
      // BEST CLUSTER SEARCH 
      //RCLCPP_INFO(this->get_logger(),"exit from getBestCluster function");
      if(target!=best_cl_id){
        // NEW TARGET
        if(best_cl.belong_to!=name_space_id){
          
          target = best_cl_id;
          target_cl = best_cl;
          
          RCLCPP_INFO(this->get_logger(),"+++ AREA DIVISION REQUIRED for selected cluster of robot%d with id: %d",GlobalClusters.clusters[target].belong_to, target);
          
          AreaDivision_isrequired = true;

          // Here I should ask for area Division client and validate target

          send_goal(clusters_to_send,target_cl);
          
        }else{          
          target = best_cl_id;
          target_cl = best_cl;
          RCLCPP_INFO(this->get_logger(),"+++ Select cluster of mine with id: %d",target);
          

          send_goal(clusters_to_send,target_cl);
          
        }
      }else{
        RCLCPP_INFO(this->get_logger(),"++ Mantain last selected id: %d",target);
        send_goal(clusters_to_send,target_cl);
      }
      
      
    }

    // If the selected current belong to an other robot:
    // PAIRWISE AREA DIVISION 
    // Client and service call to be pairwise syncronized 

    if(AreaDivision_isrequired){

    }


    // Global update of variables
    curr_cl = GlobalClusters.clusters[curr];
    target_cl = GlobalClusters.clusters[target];
    start_cl = GlobalClusters.clusters[start];

    RCLCPP_INFO(this->get_logger(),"START: %d: C%d R%d || CURR: %d: C%d R%d || TARGET: %d: C%d R%d ||", 
                                      start,start_cl.cluster_id,start_cl.belong_to,
                                      curr,curr_cl.cluster_id,curr_cl.belong_to,
                                      target,target_cl.cluster_id,target_cl.belong_to);




    // Publish to status the current cluster
    target_cl.is_target = true;
    current_cluster_pub_->publish(target_cl);
    RCLCPP_INFO(this->get_logger(),"####################### END #######################");
    RCLCPP_INFO(this->get_logger(),"--------------------------------------------------");

  }

  

  // Action routines

  void goal_response_callback(const rclcpp_action::ClientGoalHandle<Task>::SharedPtr &goal_handle)
  {
      if (!goal_handle)
      {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
      }
      else
      {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result...");
      }
  }

  void feedback_callback(rclcpp_action::ClientGoalHandle<Task>::SharedPtr, const std::shared_ptr<const Task::Feedback> feedback) {
      static int prev_id = -1;
      static int prev_cluster_id = -1;

      if (feedback->curr_vert.id != prev_id || feedback->curr_vert.cluster_id != prev_cluster_id) {
          RCLCPP_INFO(this->get_logger(), "Current node visited id: %d C%d R%d",
                      feedback->curr_vert.id, feedback->curr_vert.cluster_id,feedback->curr_vert.belong_to);
          prev_id = feedback->curr_vert.id;
          prev_cluster_id = feedback->curr_vert.cluster_id;
      }
      curr = cluster_l2g_index[feedback->curr_vert.belong_to][feedback->curr_vert.cluster_id];
      curr_cl = GlobalClusters.clusters[curr];
      curr_cl.is_target = false;
      current_cluster_pub_->publish(curr_cl);
  }


  void result_callback(const rclcpp_action::ClientGoalHandle<Task>::WrappedResult &result){

      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal succeeded! Result: %d", result.result->task_completed);
          break;
      case rclcpp_action::ResultCode::ABORTED:
          //last_selected_cluster =curr_cluster;
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          break;
      case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          break;
      default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }

      // Reset current goal handle after result
      current_goal_handle_.reset();

      navigationCallback(!result.result->task_completed);
  }

  void send_goal(std::vector<int> clusters_ids, gbeam2_interfaces::msg::GraphClusterNode cluster_task){

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }

    // Check if a goal is currently executing
    if (current_goal_handle_)
    {
      auto status = current_goal_handle_->get_status();
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
          RCLCPP_WARN(this->get_logger(), "A goal is already executing. Not sending a new goal.");
          return;
      }
    }

    gbeam2_interfaces::msg::Graph assigned_graph = getAssignedGraph(clusters_ids);

    //printMatrix(this->get_logger(),GraphAdj2matrix(assigned_graph.length_matrix));
      
    // Create a goal message
    auto goal_msg = Task::Goal();
    goal_msg.cluster_id_task = cluster_task.cluster_id;
    goal_msg.belong_to = cluster_task.belong_to; 
    goal_msg.assigned_graph = assigned_graph;
    goal_msg.has_target_bridge = (!assigned_graph.cluster_graph.bridges.empty());
    goal_msg.target_bridge = assigned_graph.cluster_graph.bridges;

    for(int z=0; z<stored_Graph->map.size();z++){
      goal_msg.adj_matrix.push_back(stored_Graph->map[z].length_matrix);
    }
    
    
    

    //RCLCPP_INFO(this->get_logger(), "Sending goal to cluster ID: %d", goal_msg.cluster_id_task);

    // Send the goal asynchronously
    auto send_goal_options = rclcpp_action::Client<Task>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&CooperationNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&CooperationNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&CooperationNode::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
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
