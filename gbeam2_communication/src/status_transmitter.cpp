//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
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

#include "gbeam2_interfaces/msg/status.hpp"
#include "library_fcn.hpp"

class StatusTXNode : public rclcpp::Node
{
public:
  StatusTXNode() : Node("status_TX"){

  // Get namespace
  name_space = this->get_namespace();
  name_space_id = name_space.back()- '0';

  //SUBSCRIBED TOPICS
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 1, std::bind(&StatusTXNode::odomCallback, this, std::placeholders::_1));

  ref_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "gbeam/gbeam_pos_ref", 1, std::bind(&StatusTXNode::refPosCallback, this, std::placeholders::_1));

  
  current_cluster_sub = this->create_subscription<gbeam2_interfaces::msg::GraphClusterNode>(
      "current_cluster",1 ,std::bind(&StatusTXNode::currentClusterCallback, this, std::placeholders::_1));

  

  std::string filter_string = "robot_id!=%0";          
  rclcpp::SubscriptionOptions options;
  options.content_filter_options.filter_expression = filter_string;
  options.content_filter_options.expression_parameters = {std::to_string(name_space_id)};
  
  status_sub_ = this->create_subscription<gbeam2_interfaces::msg::Status>("/status", 1,
            std::bind(&StatusTXNode::statusCallback, this, std::placeholders::_1),//);
            options);
  

  //PUBLISHING TOPICS
  status_pub_= this->create_publisher<gbeam2_interfaces::msg::Status>("/status",1);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StatusTXNode::statusloop, this));


  // Initialize parameters
  this->declare_parameter<int>("N_robot",0);
  this->declare_parameter<double>("communication_range",0.0);

  // Get parameters
  N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
  wifi_range = this->get_parameter("communication_range").get_parameter_value().get<double>();

  RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF STATUS_TRANSMITTER: ############# ");
  RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
  RCLCPP_INFO(this->get_logger(),"1) Number of robots: %d",N_robot);
  RCLCPP_INFO(this->get_logger(),"1) Communication range: %f",wifi_range);
  curr_status.resize(N_robot);
  curr_status[name_space_id].robot_id = name_space_id;
  for (int i = 0; i < N_robot; i++)
  {
     curr_status[i].connection_status.resize(N_robot);
     curr_status[i].joint_vector.resize(N_robot);
     curr_status[i].normal_joint_vector.resize(N_robot);
  }

 

  };

private:
  std::string name_space;
  int name_space_id;

  // Parameters variables
  int N_robot;
  double wifi_range;

  rclcpp::Publisher<gbeam2_interfaces::msg::Status>::SharedPtr status_pub_; 
  rclcpp::Subscription<gbeam2_interfaces::msg::Status>::SharedPtr status_sub_; 
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pos_subscriber_;
  rclcpp::Subscription<gbeam2_interfaces::msg::GraphClusterNode>::SharedPtr current_cluster_sub;

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry robot_odom_;
  geometry_msgs::msg::PoseStamped target_pos_;
  std::vector<gbeam2_interfaces::msg::Status> curr_status;
  double deg_90 = M_PI / 2.0;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
  {
      robot_odom_ = *odom_ptr;
      //RCLCPP_INFO(this->get_logger(),"odom received: x:%f y:%f",robot_odom_.pose.pose.position.x,robot_odom_.pose.pose.position.y);
  }

  void refPosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr tar_pos_ptr)
  {
      //RCLCPP_INFO(this->get_logger(), "refCallback executed. Point received x: %f y: %f",tar_pos_ptr->pose.position.x, tar_pos_ptr->pose.position.x);
      target_pos_ = *tar_pos_ptr;
      //pos_ref_received=true;

  }

  void statusCallback(const gbeam2_interfaces::msg::Status::SharedPtr received_status){
    curr_status[received_status->robot_id]=*received_status;
  }

  void currentClusterCallback(const gbeam2_interfaces::msg::GraphClusterNode::SharedPtr received_cluster){
    curr_status[name_space_id].current_cluster=*received_cluster;
  }


  void statusloop(){
    curr_status[name_space_id].current_position.header = robot_odom_.header;
    curr_status[name_space_id].current_position.pose = robot_odom_.pose;

    curr_status[name_space_id].last_known_target.header = target_pos_.header;
    curr_status[name_space_id].last_known_target.pose = target_pos_.pose;

    geometry_msgs::msg::Point32 pos_point;

    pos_point.x = robot_odom_.pose.pose.position.x;
    pos_point.y = robot_odom_.pose.pose.position.y;

    for (int i = 0; i < N_robot; i++)
    {
      if(i != name_space_id){
        geometry_msgs::msg::Point32 point_i;
        point_i.x = curr_status[i].current_position.pose.pose.position.x;
        point_i.y = curr_status[i].current_position.pose.pose.position.y;
        
        if(sqrt(distSq(pos_point,point_i)) < wifi_range)
        {
          curr_status[name_space_id].connection_status[i]=1; // 1 if connection is established, 0 otherwhise

          //Compute vector between the two points
          tf2::Vector3 p1(pos_point.x,pos_point.y,pos_point.z); 
          tf2::Vector3 p2(point_i.x,point_i.y,point_i.z);
          tf2::Vector3 z_axiz(0.0,0.0,1.0);

          curr_status[name_space_id].joint_vector[i].x = (p1-p2).x();
          curr_status[name_space_id].joint_vector[i].y = (p1-p2).y();
          curr_status[name_space_id].joint_vector[i].z = (p1-p2).z();

          //compute the normal
          curr_status[name_space_id].normal_joint_vector[i].x = (p1-p2).rotate(z_axiz,deg_90).x();
          curr_status[name_space_id].normal_joint_vector[i].y = (p1-p2).rotate(z_axiz,deg_90).y();
          curr_status[name_space_id].normal_joint_vector[i].z = (p1-p2).rotate(z_axiz,deg_90).z();
        }
        else{
          curr_status[name_space_id].connection_status[i]=0;
        }        
      }
      

    }
        

    status_pub_->publish(curr_status[name_space_id]);
  }
  


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatusTXNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
