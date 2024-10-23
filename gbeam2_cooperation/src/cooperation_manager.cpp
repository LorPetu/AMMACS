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
#include "gbeam2_interfaces/msg/frontier_stamped.hpp"
#include "gbeam2_interfaces/msg/frontier_stamped_array.hpp"
#include "gbeam2_interfaces/msg/fieldler_info.hpp"
#include "gbeam2_interfaces/msg/graph_cluster_node.hpp"
#include "library_fcn.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // For easier point cloud population

#include <Eigen/Dense>
#include <Spectra/SymEigsSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>

using namespace Eigen;
using namespace Spectra;
using namespace std;

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

    frontier_pub_ = this->create_publisher<gbeam2_interfaces::msg::FrontierStampedArray>(
      "frontier",1);

    start_frontiers_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_frontier",std::bind(&CooperationNode::startFrontier,this, std::placeholders::_1, std::placeholders::_2));

    point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "merged_obstacles",1);

    fiedler_vector_pub_ = this->create_publisher<gbeam2_interfaces::msg::FieldlerInfo>("fieldler_vector",1);

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

  //Frontiers variables
  int N_my_frontiers = 0;
  std::pair<gbeam2_interfaces::msg::Vertex, gbeam2_interfaces::msg::Vertex> obs_min_pair;
  gbeam2_interfaces::msg::FrontierStampedArray res_frontier_array;
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
  rclcpp::Publisher<gbeam2_interfaces::msg::FrontierStampedArray>::SharedPtr frontier_pub_;
  rclcpp::Subscription<gbeam2_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr assigned_graph_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  start_frontiers_service_;
  rclcpp::Publisher<gbeam2_interfaces::msg::FieldlerInfo>::SharedPtr fiedler_vector_pub_;

  //rclcpp::TimerBase::SharedPtr cluster_timer_;

  double deg_90 = M_PI / 2.0;
  double deg_30 = M_PI / 6.0;


  bool checkIntersection(gbeam2_interfaces::msg::Vertex p1,gbeam2_interfaces::msg::Vertex p2,gbeam2_interfaces::msg::Vertex p3,gbeam2_interfaces::msg::Vertex p4){
      // Check it there's an intersection between two segment: 
      // p1 - rec_pos: the segment between the 2 robot
      // p3   - p4:    the segment between the two obstacles boundary vertex.  

      double t = ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x)==0) ? -1 : 
                  ((p1.x - p3.x)*(p3.y - p4.y) - (p1.y - p3.y)*(p3.x - p4.x))
                / ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x));

      double u = ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x)==0) ? -1 : 
                -((p1.x - p2.x)*(p1.y - p3.y) - (p1.y-p2.y)*(p1.x - p3.x))
                / ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));

      return (t>=0.0 && t<=1.0 && u>=0.0 && u<=1.0) ? true : false;

  }

  bool checkIntersection(gbeam2_interfaces::msg::Vertex p1,geometry_msgs::msg::Point p2,gbeam2_interfaces::msg::Vertex p3,gbeam2_interfaces::msg::Vertex p4){
      // Check it there's an intersection between two segment: 
      // p1 - rec_pos: the segment between the 2 robot
      // p3   - p4:    the segment between the two obstacles boundary vertex.  

      double t = ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x)==0) ? -1 : 
                  ((p1.x - p3.x)*(p3.y - p4.y) - (p1.y - p3.y)*(p3.x - p4.x))
                / ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x));

      double u = ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x)==0) ? -1 : 
                -((p1.x - p2.x)*(p1.y - p3.y) - (p1.y-p2.y)*(p1.x - p3.x))
                / ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));

      return (t>=0.0 && t<=1.0 && u>=0.0 && u<=1.0) ? true : false;

  }

  bool checkIntersection(geometry_msgs::msg::Point p1,geometry_msgs::msg::Point p2,gbeam2_interfaces::msg::Vertex p3,gbeam2_interfaces::msg::Vertex p4){
    // Check it there's an intersection between two segment: 
    // p1 - rec_pos: the segment between the 2 robot
    // p3   - p4:    the segment between the two obstacles boundary vertex.  

    double t = ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x)==0) ? -1 : 
                ((p1.x - p3.x)*(p3.y - p4.y) - (p1.y - p3.y)*(p3.x - p4.x))
              / ((p1.x -p2.x)*(p3.y - p4.y) - (p1.y - p2.y)*(p3.x - p4.x));

    double u = ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x)==0) ? -1 : 
              -((p1.x - p2.x)*(p1.y - p3.y) - (p1.y-p2.y)*(p1.x - p3.x))
              / ((p1.x - p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));

    return (t>=0.0 && t<=1.0 && u>=0.0 && u<=1.0) ? true : false;

  }

  std::pair<double, bool> sideOfLine(geometry_msgs::msg::Point lineStart, geometry_msgs::msg::Point lineEnd, gbeam2_interfaces::msg::Vertex point) {
      // 1. Compute the cross product to determine which side of the main line the point is on.
      double cross_product = (lineEnd.x - lineStart.x) * (point.y - lineStart.y) - 
                            (lineEnd.y - lineStart.y) * (point.x - lineStart.x);

      // The value will be positive if the point is on one side, negative if on the other.
      double value = cross_product;

      // 2. Check if the point is within the perpendicular bounds defined by the start and end points.
      // This can be done by projecting the point onto the main line, and seeing if the projected point lies between the start and end.

      // Vector from lineStart to lineEnd
      double dx = lineEnd.x - lineStart.x;
      double dy = lineEnd.y - lineStart.y;

      // Vector from lineStart to the point
      double px = point.x - lineStart.x;
      double py = point.y - lineStart.y;

      // Project the point onto the line (calculate the projection scalar)
      double dot_product = px * dx + py * dy;
      double line_length_squared = dx * dx + dy * dy;
      double projection = dot_product / line_length_squared;

      // Check if the projection is between 0 and 1 (i.e., the point is between lineStart and lineEnd in the direction of the line)
      bool is_inside = (projection >= 0 && projection <= 1);

      return std::make_pair(value, is_inside);
  }

  std::pair<double, bool> sideOfLine(gbeam2_interfaces::msg::Vertex lineStart, gbeam2_interfaces::msg::Vertex lineEnd, gbeam2_interfaces::msg::Vertex point) {
    // 1. Compute the cross product to determine which side of the main line the point is on.
    double cross_product = (lineEnd.x - lineStart.x) * (point.y - lineStart.y) - 
                           (lineEnd.y - lineStart.y) * (point.x - lineStart.x);

    // The value will be positive if the point is on one side, negative if on the other.
    double value = cross_product;

    // 2. Check if the point is within the perpendicular bounds defined by the start and end points.
    // This can be done by projecting the point onto the main line, and seeing if the projected point lies between the start and end.

    // Vector from lineStart to lineEnd
    double dx = lineEnd.x - lineStart.x;
    double dy = lineEnd.y - lineStart.y;

    // Vector from lineStart to the point
    double px = point.x - lineStart.x;
    double py = point.y - lineStart.y;

    // Project the point onto the line (calculate the projection scalar)
    double dot_product = px * dx + py * dy;
    double line_length_squared = dx * dx + dy * dy;
    double projection = dot_product / line_length_squared;

    // Check if the projection is between 0 and 1 (i.e., the point is between lineStart and lineEnd in the direction of the line)
    bool is_inside = (projection >= 0 && projection <= 1);

    return std::make_pair(value, is_inside);
}

  std::pair<double, bool> sideOfLine(gbeam2_interfaces::msg::Vertex lineStart,  geometry_msgs::msg::Point lineEnd, gbeam2_interfaces::msg::Vertex point) {
    // 1. Compute the cross product to determine which side of the main line the point is on.
    double cross_product = (lineEnd.x - lineStart.x) * (point.y - lineStart.y) - 
                           (lineEnd.y - lineStart.y) * (point.x - lineStart.x);

    // The value will be positive if the point is on one side, negative if on the other.
    double value = cross_product;

    // 2. Check if the point is within the perpendicular bounds defined by the start and end points.
    // This can be done by projecting the point onto the main line, and seeing if the projected point lies between the start and end.

    // Vector from lineStart to lineEnd
    double dx = lineEnd.x - lineStart.x;
    double dy = lineEnd.y - lineStart.y;

    // Vector from lineStart to the point
    double px = point.x - lineStart.x;
    double py = point.y - lineStart.y;

    // Project the point onto the line (calculate the projection scalar)
    double dot_product = px * dx + py * dy;
    double line_length_squared = dx * dx + dy * dy;
    double projection = dot_product / line_length_squared;

    // Check if the projection is between 0 and 1 (i.e., the point is between lineStart and lineEnd in the direction of the line)
    bool is_inside = (projection >= 0 && projection <= 1);

    return std::make_pair(value, is_inside);
}


  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
  {
      robot_odom_ = *odom_ptr;
      //RCLCPP_INFO(this->get_logger(),"odom received: x:%f y:%f",robot_odom_.pose.pose.position.x,robot_odom_.pose.pose.position.y);
  }

  void mergedGraphCallback(const std::shared_ptr<gbeam2_interfaces::msg::Graph> graph_received){
  

    if(graph_received->robot_id!=name_space_id){
     
      int req_robot_id = graph_received->robot_id;
      last_updated_node = (stored_Graph[req_robot_id]->nodes.empty()) ? -1 : stored_Graph[req_robot_id]->nodes.back().id;
      last_updated_edge = (stored_Graph[req_robot_id]->edges.empty()) ? -1 : stored_Graph[req_robot_id]->edges.back().id;

      for(gbeam2_interfaces::msg::Vertex node: graph_received->nodes){

        if(node.id<last_updated_node){
          stored_Graph[req_robot_id]->nodes[node.id]=node;          
        }
        else{
          stored_Graph[req_robot_id]->nodes.push_back(node);
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

    } else{
      stored_Graph[name_space_id] = graph_received;
    }


  }
  
  void computeSecondSmallestEigen(MatrixXd& laplacianMatrix, VectorXd& eigenvalues, VectorXd& fiedler_vector, VectorXd& fiedler_vector2, int N)
{

    // We are going to calculate the eigenvalues of M

    // Construct matrix operation object using the wrapper class DenseSymMatProd
    DenseSymMatProd<double> op(laplacianMatrix);

    // Construct eigen solver object, requesting the largest three eigenvalues
    SymEigsSolver<DenseSymMatProd<double>> eigs(op, 3, N);

    // Initialize and compute
    eigs.init();
    eigs.compute(SortRule::SmallestAlge);
    std::ostringstream oss;

    // Retrieve results
  
    if(eigs.info() == CompInfo::Successful){
        eigenvalues = eigs.eigenvalues();

        Eigen::MatrixXd evectors = eigs.eigenvectors();
       
        fiedler_vector = evectors.col(1);
        fiedler_vector2 = evectors.col(2);


         // Convert the eigenvalues to a string
        std::stringstream eigenvalues_stream;
        eigenvalues_stream << eigenvalues;
        std::string eigenvalues_str = eigenvalues_stream.str();

        // Log the eigenvalues using RCLCPP
        RCLCPP_INFO(this->get_logger(), "Eigenvalues found:\n%s", eigenvalues_str.c_str());
    } else{
      RCLCPP_INFO(this->get_logger(),"Something went wrong computing eigenvalues!");      
    }
    

    return;
}
 
  Eigen::MatrixXd getGraphMatrixes(){
    
    reach_node_label.clear();

    std::vector<gbeam2_interfaces::msg::Vertex> nodes = stored_Graph[name_space_id]->nodes;

    for(auto& node:nodes){
        if(!node.is_obstacle){
            reach_node_label.push_back(node.id);
        }
    }
    int N =  stored_Graph[name_space_id]->adj_matrix.size;
    int N_reach = reach_node_label.size();
    Eigen::MatrixXd reach_mat(N_reach,N_reach);

    int row = 0;
    int col = 0;

    for (int i:reach_node_label) {
        for (int j:reach_node_label) {
            int el = stored_Graph[name_space_id]->adj_matrix.data[i * N + j];
            
            if(i==j){
                int degree = nodes[i].neighbors.size();  // Degree is the number of neighbors
                reach_mat(row,col) = degree;
            }else{
                if(el>-1){
                    reach_mat(row,col) = -1;
                }else{
                    reach_mat(row,col) = 0;
                }
                } 
                col++;
        }
        row++;
        col=0;
    }

    return reach_mat;
  }

  void startFrontier(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
      start_frontier=request->data; 

      gbeam2_interfaces::msg::FieldlerInfo msg;

      //Eigen::MatrixXd laplacianMatrix;
      Eigen::MatrixXd laplacianMatrix = getGraphMatrixes();

      //printMatrix(laplacianMatrix, reach_node_label);  // Assuming reach_node_label is in scope
      
      // Variables to store the second smallest eigenvalue and the Fiedler vector
      VectorXd eigenvalues;
      VectorXd fiedler_vector;
      VectorXd fiedler_vector2;

      // Compute the second smallest eigenvalue and its corresponding eigenvector
      computeSecondSmallestEigen(laplacianMatrix, eigenvalues, fiedler_vector, fiedler_vector2, reach_node_label.size()*3/4 );

      RCLCPP_INFO(this->get_logger(),"Fieldler vector has been computed!");
      //plotEigenvector(eigenvalues, reach_node_label);


       // DEBUG CLOUDPOINT 
      // Prepare the PointCloud2 message
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_msg.header.stamp = this->now();  // Set timestamp
      cloud_msg.header.frame_id = "world";     // Set frame ID (adjust if necessary)

      // Reserve space for the points and the additional "side" field
      cloud_msg.height = 1;                  // Unordered point cloud (1D array)
      cloud_msg.is_dense = false;            // Allow for possible invalid points
      int total_points = reach_node_label.size();
      cloud_msg.width = total_points;        // Number of points

      // Define the PointCloud2 fields
      sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
      modifier.setPointCloud2Fields(4,  // Number of fields: x, y, z, and side
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "comp", 1, sensor_msgs::msg::PointField::FLOAT32);  // Custom field for side (0 = left, 1 = right)

      modifier.resize(total_points);  // Resize the point cloud to accommodate all points

      // Use iterators for better handling of PointCloud2
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_comp(cloud_msg, "comp");

      // Fill the data in row-major order (first all values of reach_node_label, then all values of field_vector)
      int i=0;
      for (const auto& id : reach_node_label) {
          msg.label_vector.push_back(static_cast<float>(id)); // First row: node labels
          float comp = static_cast<float>(fiedler_vector(i));
          msg.fieldler_vector.push_back(comp); // Second row: Eigen vector values

          auto node = stored_Graph[name_space_id]->nodes[id];
          *iter_x = node.x;
          *iter_y = node.y;
          *iter_z = node.z;
          *iter_comp = comp;  // Right side
        
          ++iter_x;
          ++iter_y;
          ++iter_z;
          ++iter_comp;

          i++;
      }
      // Process obstacles and reachables

      // Publish the point cloud
      point_cloud_publisher_->publish(cloud_msg);

      for (int i = 0; i < fiedler_vector2.size(); ++i) {
          msg.fieldler_vector2.push_back(static_cast<float>(fiedler_vector2(i))); // Second row: Eigen vector values
      }

      // Publish the message
      fiedler_vector_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Published matrix with %ld columns.", reach_node_label.size());    
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

 float computeClusterTresh(const std::map<int,float> cluster_values){
     // Extract the centroids in sorted order by their values
    std::vector<std::pair<int, float>> sorted_clusters(cluster_values.begin(), cluster_values.end());
    std::sort(sorted_clusters.begin(), sorted_clusters.end(), [](auto& a, auto& b) {
        return a.second < b.second;
    });

    // Calculate distances between contiguous centroids
    std::vector<float> distances;
    for (int i = 0; i < sorted_clusters.size() - 1; ++i) {
        float distance = std::fabs(sorted_clusters[i + 1].second - sorted_clusters[i].second);
        distances.push_back(distance);
    }

    // Compute the mean of the contiguous distances
    float sum_of_distances = 0;
    for (const auto& dist : distances) {
        sum_of_distances += dist;
    }
    float mean_distance = sum_of_distances / distances.size();
    float threshold = mean_distance / 2;

   
    return threshold;
  }

  void CreateClusterGraph(const VectorXd& fiedler_vector){
    std::map<int,float> cluster_values;
    std::map<int,float> cluster_centroids;

    // If there's no cluster first nodes compose the first cluster
    if(clusterGraph.empty()){
      int n=0;

      for (int i = 0; i < fiedler_vector.size(); ++i) {
      float value = static_cast<float>(fiedler_vector(i));
      fieldler_values.push_back(value);
      cluster_ids.push_back(0);   
      n ++;
      cluster_values[0] +=(cluster_values[0] -value)/n;

      gbeam2_interfaces::msg::GraphClusterNode new_cluster;
      new_cluster.cluster_id = 0;
      clusterGraph.push_back(new_cluster);
    
    }

      return;
    } 

    // Re-assign new fieldler vector components to each nodes
    for (int i = 0; i < fiedler_vector.size(); ++i) {
      float value = static_cast<float>(fiedler_vector(i));
      fieldler_values.push_back(value);
      if(i>cluster_ids.size()){cluster_ids.push_back(-1);}
      else{
        int n = std::count(cluster_ids.begin(),cluster_ids.end(),cluster_ids[i]);
        cluster_values[cluster_ids[i]] += value/n;
        //if(abs(cluster_values[cluster_ids[i]] - value))
      }
    }

    float max_treshold = computeClusterTresh(cluster_values);

    // reverse loop because i want to process first the last added nodes
    for (int i=reach_node_label.size(); i>=0; --i ){
      if(cluster_ids[i]==-1){
        float min_dist =INF;
        int min_cluster =-1;
        for(auto& cluster_map:cluster_values){
          float distance = std::fabs(fiedler_vector[i] - cluster_map.second);
          if(distance<min_dist) {min_dist = distance; min_cluster=cluster_map.first;}
        }
        if(min_dist<max_treshold){// assign node to a cluster
          cluster_ids[i]=min_cluster;
        } else{// create a new cluster
          int N_cluster = cluster_values.size();
          cluster_values[N_cluster] = fieldler_values[i];
          gbeam2_interfaces::msg::GraphClusterNode new_cluster;

          new_cluster.cluster_id = N_cluster;
          clusterGraph.push_back(new_cluster);
        }
      }
    }

    // Compute bridges and connect to other clusters
    for(auto& cluster:clusterGraph){
      //Check which node has at least one neighbour that doesn't belong to its own cluster
    }
   
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
