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

class Ellipse {
  private:
      gbeam2_interfaces::msg::Vertex focus1;
      gbeam2_interfaces::msg::Vertex focus2;
      double a;  // Semi-major axis
      double b; // Semi-minor axis

      gbeam2_interfaces::msg::Vertex pointToVertex(const geometry_msgs::msg::Point& point) const {
        gbeam2_interfaces::msg::Vertex vert;
        vert.x = point.x;
        vert.y = point.y;
        vert.z = point.z;
        // Initialize other Vertex fields with default values if needed
        return vert;
      }
      void calculateSemiMajorAxis() {
          double c = dist(focus1, focus2) / 2.0;  // Half the distance between foci 
          a = std::sqrt(b*b + c*c);
      }

  public:
      // Constructor
      Ellipse(const gbeam2_interfaces::msg::Vertex& f1, const gbeam2_interfaces::msg::Vertex& f2, double semi_minor_axis)
          : focus1(f1), focus2(f2), b(semi_minor_axis) {
            calculateSemiMajorAxis();
          }

           // Constructor with Point objects
      Ellipse(const geometry_msgs::msg::Point& f1, const geometry_msgs::msg::Point& f2, double semi_minor_axis)
        : focus1(pointToVertex(f1)), focus2(pointToVertex(f2)), b(semi_minor_axis) {
          calculateSemiMajorAxis();
        }

      // Check if a Vertex is inside the ellipse
      bool isInside(const gbeam2_interfaces::msg::Vertex& vert) const {
          double d1 = dist(vert, focus1);
          double d2 = dist(vert, focus2);
          return (d1 + d2) <= 2 * a;
      }

      // Check if a point is inside the ellipse
      bool isInside(const geometry_msgs::msg::Point point) const {
          gbeam2_interfaces::msg::Vertex vert;
          vert.x = point.x;
          vert.y = point.y;
          //vert.z = z;
          return isInside(vert);
      }

      // Getters
      const gbeam2_interfaces::msg::Vertex& getFocus1() const { return focus1; }
      const gbeam2_interfaces::msg::Vertex& getFocus2() const { return focus2; }
      double getSemiMajorAxis() const { return a; }
      double getSemiMinorAxis() const { return b; }

      // Setters
      void setFocus1(const gbeam2_interfaces::msg::Vertex& f1) { focus1 = f1; }
      void setFocus2(const gbeam2_interfaces::msg::Vertex& f2) { focus2 = f2; }
      void setSemiMinorAxis(double semi_minor_axis) { b = semi_minor_axis; }
  };



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
      "gbeam/assigned_graph",1);

    start_frontiers_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_frontier",std::bind(&CooperationNode::startFrontier,this, std::placeholders::_1, std::placeholders::_2));


    //cluster_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&CooperationNode::CreateClusterGraph, this));

     // Initialize parameters
    this->declare_parameter<int>("N_robot",0);
    this->declare_parameter<double>("communication_range",0.0);
    this->declare_parameter<double>("elipse_scaling_obs",0.0);
    this->declare_parameter<double>("elipse_scaling_robot",0.0);

    // Get parameters
    N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
    wifi_range = this->get_parameter("communication_range").get_parameter_value().get<double>();
    elipse_scaling_obs = this->get_parameter("elipse_scaling_obs").get_parameter_value().get<double>();
    elipse_scaling_robot = this->get_parameter("elipse_scaling_robot").get_parameter_value().get<double>();

    RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF COOPERATION: ############# ");
    RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
    RCLCPP_INFO(this->get_logger(),"1) Number of robots: %d",N_robot);
    RCLCPP_INFO(this->get_logger(),"2) Communication range: %f",wifi_range);
    RCLCPP_INFO(this->get_logger(),"3) Elipse scaling for frontier obstacle node: %f",elipse_scaling_obs);
    RCLCPP_INFO(this->get_logger(),"4) Elipse scaling between robots: %f",elipse_scaling_robot);

    // Initialize vectors with the correct size
    stored_Graph.resize(N_robot);
    last_status.resize(N_robot);
    last_status[name_space_id].robot_id = name_space_id;
    for (int i = 0; i < N_robot; i++)
    {
      stored_Graph[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
      last_status[i].connection_status.resize(N_robot);
      last_status[i].joint_vector.resize(N_robot);
      last_status[i].normal_joint_vector.resize(N_robot);
    }

  }

private:
  std::string name_space;
  int name_space_id;

  // Parameters variables
  int N_robot;
  double wifi_range;
  double elipse_scaling_obs;
  double elipse_scaling_robot;

  std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> stored_Graph;
  int last_updated_node;
  int last_updated_edge;
  bool start_frontier = false;

  //
  int updates_iter = 0;
  bool eval_updates = false;
  std::vector<gbeam2_interfaces::msg::Vertex> first_batch;
  std::vector<gbeam2_interfaces::msg::Vertex> second_batch;

  //Frontiers variables
  int N_my_frontiers = 0;
  std::pair<gbeam2_interfaces::msg::Vertex, gbeam2_interfaces::msg::Vertex> obs_min_pair;
  bool has_bridge = false;
  double dist_ij;
  std::vector<gbeam2_interfaces::msg::Vertex> candidates_reach_nodes;
  std::vector<gbeam2_interfaces::msg::Vertex> merged_obstacles;
  std::vector<gbeam2_interfaces::msg::Vertex> obstacles_left;
  std::vector<gbeam2_interfaces::msg::Vertex> obstacles_right;
  std::vector<gbeam2_interfaces::msg::Vertex> merged_reachables;
  std::vector<gbeam2_interfaces::msg::Vertex> inside_reachables;
  std::vector<gbeam2_interfaces::msg::Vertex> reachables_left;
  std::vector<gbeam2_interfaces::msg::Vertex> reachables_right;
  std::vector<gbeam2_interfaces::msg::Vertex> obstacles_to_evaluate;

  std::vector<int> cluster_ids;
  std::vector<int> reach_node_label;
  std::vector<float> fieldler_values;
  std::vector<gbeam2_interfaces::msg::GraphClusterNode> clusterGraph; 


  std::vector<gbeam2_interfaces::msg::Status> last_status;
  nav_msgs::msg::Odometry robot_odom_;


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

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  start_frontiers_service_;

  //rclcpp::TimerBase::SharedPtr cluster_timer_;

  double deg_90 = M_PI / 2.0;
  double deg_30 = M_PI / 6.0;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
  {
      robot_odom_ = *odom_ptr;
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

    for(gbeam2_interfaces::msg::Vertex node: graph_received->nodes){

      if(node.id<last_updated_node){
        //update old ones
        stored_Graph[req_robot_id]->nodes[node.id]=node;          
      }
      else{
        // adding new nodes

        RCLCPP_INFO(this->get_logger(), "Add new node %d", node.id);
        stored_Graph[req_robot_id]->nodes.push_back(node);
        if(!node.is_obstacle){
          new_nodes = true;
          switch (updates_iter)
          {
          case 0:
            first_batch.push_back(node);
            eval_updates=false;
            break;

          case 1:
            second_batch.push_back(node);
            eval_updates=true;
            break;
          
          default:
            break;
          }
        }
        
      }

    }
    if(new_nodes){
      if(!eval_updates){
          updates_iter++; 
        } 
        else{

          first_batch.clear();
          second_batch.clear();
          updates_iter=0;
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




  }
  

  

  void startFrontier(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
      start_frontier=request->data; 

  }

  void statusCallback(const gbeam2_interfaces::msg::Status::SharedPtr received_status){
    last_status[received_status->robot_id]=*received_status;
    if(!start_frontier) return;

    start_frontier = false;

  

    // res_frontier_array.frontiers.clear(); 

    // auto my_pos = robot_odom_.pose.pose.position; // Odom position of the robot itself
    // auto rec_pos = received_status->current_position.pose.pose.position; // Received position of the other robot

    // if(stored_Graph[name_space_id]->nodes.size()==0) return;
    

    // if(received_status->connection_status[name_space_id]){


    // }
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
