//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <queue>


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

#include "gbeam2_interfaces/srv/set_mapping_status.hpp"
#include "gbeam2_interfaces/action/assigned_task.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "library_fcn.hpp"

#define INF 100000

class ExplorationNode : public rclcpp::Node
{
public:
    
    using Task = gbeam2_interfaces::action::AssignedTask;
    explicit ExplorationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("graph_expl",options), active_goal_handle_(nullptr)
    {   
        name_space = this->get_namespace();
        graph_subscriber_ = this->create_subscription<gbeam2_interfaces::msg::Graph>(
            "gbeam/reachability_graph", 1, std::bind(&ExplorationNode::graphCallback, this, std::placeholders::_1));

        //"coop/assigned_graph"   OR "gbeam/reachability_graph" 

        pos_ref_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "gbeam/gbeam_pos_ref", 1);
    
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



      // DECLARATION OF PARAMETERS FROM YAML FILE
        
        this-> declare_parameter<float>("reached_tol",0.0);
        this-> declare_parameter<float>("distance_exp",0.0);
        
        //Exploration limits
        this->declare_parameter<double>("limit_xi",0.0);
        this->declare_parameter<double>("limit_xs",0.0);
        this->declare_parameter<double>("limit_yi",0.0);
        this->declare_parameter<double>("limit_ys",0.0);

        // Get parameter from yaml file
        reached_tol = this->get_parameter("reached_tol").get_parameter_value().get<float>();
        distance_exp = this->get_parameter("distance_exp").get_parameter_value().get<float>();
        limit_xi = this->get_parameter("limit_xi").get_parameter_value().get<double>();
        limit_xs = this->get_parameter("limit_xs").get_parameter_value().get<double>();
        limit_yi = this->get_parameter("limit_yi").get_parameter_value().get<double>();
        limit_ys = this->get_parameter("limit_ys").get_parameter_value().get<double>();

        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF EXPLORATION NODE: ############# ");
        RCLCPP_INFO(this->get_logger(),"1) REACHED_TOL: %f", reached_tol);
        RCLCPP_INFO(this->get_logger(),"2) DISTANCE_EXP: %f", distance_exp);
        RCLCPP_INFO(this->get_logger(),"3) LIMIT_XI: %f", limit_xi);
        RCLCPP_INFO(this->get_logger(),"4) LIMIT_XS: %f", limit_xs);
        RCLCPP_INFO(this->get_logger(),"5) LIMIT_YI: %f", limit_yi);
        RCLCPP_INFO(this->get_logger(),"6) LIMIT_YS: %f", limit_ys);
        
      //---------------------------------------------

        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<Task>(
            this,
            "Task", // Action name
            std::bind(&ExplorationNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ExplorationNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ExplorationNode::handle_accepted, this, std::placeholders::_1)
        );

    }

    
    
private:

    

    double distance_exp;
    double reached_tol;
    double limit_xi, limit_xs, limit_yi, limit_ys;

    int N=0, E=0; 
    int last_target =-1;
    int mapping_z = 5;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string name_space;
    
    gbeam2_interfaces::msg::Graph graph;
    gbeam2_interfaces::msg::Vertex last_target_vertex;
    gbeam2_interfaces::msg::Vertex intermediate_target_vertex;
    gbeam2_interfaces::msg::Vertex curr_vertex;
    std::vector<int> path;

    // Action
    rclcpp_action::Server<Task>::SharedPtr action_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Task>> active_goal_handle_;
    bool task_completed = false;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pos_ref_publisher_;
    rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr graph_subscriber_;


    std::vector<int> dijkstraWithAdj(gbeam2_interfaces::msg::Graph graph, int s, int t)
    {
    int N = graph.nodes.size();
    int E = graph.adj_matrix.size;
    auto adjMatrix = graph.adj_matrix.data;

    // Since we're considering only reachable node we skip the filtering part.

    std::vector<double> dist(N, INF);
    std::vector<int> parent(N, -1); // To store the shortest path tree
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    dist[s] = 0;
    pq.push({0, s});


    while (!pq.empty()) {
            auto [currentDist, u] = pq.top();
            pq.pop();

            // If the distance is already larger, skip
            if (currentDist > dist[u]) continue;

            int u_id = graph.nodes[u].id;

            // Explore neighbors
            for (int v = 0; v < N; v++) {
            int v_id = graph.nodes[v].id;
                double weight = adjMatrix[u_id*N + v_id]; // adj.data[i * N + j] = matrix[i][j];
                if (weight > 0 && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        std::vector<int> path;
        for (int at = t; at != -1; at = parent[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end()); // Reverse to get the path from source to target

        // Check if the path starts with the source
        if (!path.empty() && path[0] == s) {
            return path;
        }
        return {}; // Return empty if there's no valid path


        return path;
    }

        // Actions routines

     // Handle incoming goal requests
    rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const Task::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");

        // Reject the goal if another goal is active
        if (active_goal_handle_)
        {
            RCLCPP_WARN(this->get_logger(), "Another goal is already active. Rejecting this goal.");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Handle cancellation requests
    rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<rclcpp_action::ServerGoalHandle<Task>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Handle accepted goals
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Task>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Accepted goal");

        // Store the active goal handle
        active_goal_handle_ = goal_handle;

        // Start the goal execution in a separate thread
        std::thread([this, goal_handle]() { this->execute(goal_handle); }).detach();
    }
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Task>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(4); //4Hz - every 250ms

        const auto goal = goal_handle->get_goal();
        graph = goal->assigned_graph;
        task_completed = false;
        int local_target =-1; // Local enumeration of nodes using index (last_reached follow the same logic)
        geometry_msgs::msg::PoseStamped pos_ref;
        
        auto feedback = std::make_shared<Task::Feedback>();        
        auto result = std::make_shared<Task::Result>();

        if(last_target<0){
            // Inizialization, node is just started
            // Get the node on which i am
            auto [dist, id] = vert_graph_distance_noobstacle(graph,getCurrPos()); 
            curr_vertex = graph.nodes[id];
            last_target_vertex = curr_vertex;
            last_target = id;
            RCLCPP_INFO(this->get_logger(), "I am on the node with id: global:%d", id);        
        }

        if(goal->has_target_bridge){
            // I need to go to an external cluster and get to a bridge end to traspass it

            int bridge_end_id = (graph.robot_id == goal->target_bridge.r1) ? goal->target_bridge.v1 : goal->target_bridge.v2;
             
            for(int i=0; i<graph.nodes.size();i++){
                if(graph.nodes[i].id==bridge_end_id) {
                    local_target = i; 
                    break;
                }
            }

            path = dijkstraWithAdj(graph, last_target, local_target);

        }else{
            // Client doesn't specify any particular target node, just need to explore a cluster
            // At start select the best node

            local_target = getBestNode(goal->cluster_id_task);

            // Compute the path to it 

            path = dijkstraWithAdj(graph, last_target, local_target);
            
        }

        std::string local_path_str, global_path_str;
        for (int node : path) {
            local_path_str += std::to_string(node) + "-";
            global_path_str += std::to_string(graph.nodes[node].id) + "-";
        }

        RCLCPP_INFO(this->get_logger(), "Local Path : %s", local_path_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Global Path: %s", global_path_str.c_str());

        last_target_vertex = graph.nodes[local_target];
        intermediate_target_vertex = (path.size()>1) ? graph.nodes[path[1]]: last_target_vertex;

        

        if (last_target < N) {
            pos_ref.pose.position = vertex2point(intermediate_target_vertex);
            pos_ref.pose.position.z = mapping_z;
            pos_ref.header.frame_id = name_space.substr(1, name_space.length()-1) + "/odom";
            pos_ref_publisher_->publish(pos_ref);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid last_target after path computation: %d", last_target);
        }

        // Exploration Callback

        while(!task_completed && rclcpp::ok()){

            if (goal_handle->is_canceling()) {
                result->last_reached = curr_vertex;
                result->task_completed = false;               
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Explore the assigned cluster until there's no more unexplored node
            // or current action is aborted
            // OR IF I HAVE A TARGET BRIDGE
            // once the bridge end is reached
          
            auto [last_reached,is_local_target] = pathHandle(getCurrPos());

            if(is_local_target){
                curr_vertex = last_target_vertex;
                if(goal->has_target_bridge){
                    // End task
                    task_completed = true;
                }else{ 
                    // ##########################################
                    // ##### Assigned Cluster Exploration #######
                    // ##########################################

                    local_target = getBestNode(goal->cluster_id_task);
                    path = dijkstraWithAdj(graph, last_target, local_target);

                    std::string local_path_str, global_path_str;
                    for (int node : path) {
                        local_path_str += std::to_string(node) + "-";
                        global_path_str += std::to_string(graph.nodes[node].id) + "-";
                    }

                    RCLCPP_INFO(this->get_logger(), "Local Path : %s", local_path_str.c_str());
                    RCLCPP_INFO(this->get_logger(), "Global Path: %s", global_path_str.c_str());
   

                    task_completed=true; // TODO: just when there are no more unexplored nodes 
                }
            }else{
                curr_vertex = graph.nodes[last_reached];

                // Publish position reference to controller
                pos_ref.pose.position = vertex2point(intermediate_target_vertex);
                pos_ref.pose.position.z = mapping_z;
                pos_ref.header.frame_id = name_space.substr(1, name_space.length()-1) + "/odom";
                pos_ref_publisher_->publish(pos_ref);
            }
            
            feedback->curr_vert = curr_vertex;
            goal_handle->publish_feedback(feedback);
            
            loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            // Modify here the result information
            result->last_reached = curr_vertex;
            result->task_completed = task_completed;
            goal_handle->succeed(result);
            active_goal_handle_ = nullptr; // Clear the active goal
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        
    }


    void graphCallback(const gbeam2_interfaces::msg::Graph::SharedPtr graph_ptr)
    {
        graph = *graph_ptr;
        N = graph.nodes.size();
        E = graph.edges.size();
    }

    int getBestNode(int cluster_id){
        // Get the Best node among the one inside the specified cluster
        float max_reward = 0;
        int best_node = 0;
        std::vector<float> dist(N, INF);

        shortestDistancesWithAdjMatrix(graph, dist.data(), last_target);


        for(size_t n = 0; n < N; n++)
        {
            if(graph.nodes[n].is_reachable)
            {
                if (dist[n] == INF || graph.nodes[n].cluster_id!=cluster_id) {
                    // Skip unreachable nodes and the ones that doesn't belong to the specified cluster
                    continue;  
                }
                float reward = graph.nodes[n].gain / std::pow(dist[n], distance_exp);
                if(reward > max_reward)
                {
                    max_reward = reward;
                    best_node = n;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Best node -> local id: %d global id: %d", best_node, graph.nodes[best_node].id);

        return best_node;
    }

    std::pair<int, bool> pathHandle(gbeam2_interfaces::msg::Vertex pos) {
        int last_reached; // Local enumeration
        bool is_target = false;

        // Check if path is not empty
        if (path.empty()) {
            RCLCPP_WARN(this->get_logger(), "Path is empty, cannot update.");
            return std::make_pair(-1, is_target); // Return invalid value if path is empty
        }

        // Check if the current position has reached the next vertex in the path
        if (dist(intermediate_target_vertex, pos) <= reached_tol) {
            // We've reached the next node in the path
            RCLCPP_INFO(this->get_logger(),"pathHandle::Reached intermediate target!");
            last_reached = path[1];  // The node we just reached

            // Modify last_target and reduce path
            path.erase(path.begin()); // Remove the first element of the path

            // If we reach the end of the path, set is_target to true
            if (path.empty()) {
                RCLCPP_INFO(this->get_logger(),"pathHandle::Reached end of the path!");
                is_target = true;
                curr_vertex=last_target_vertex;
            } else {
                // Set the intermediate target as the next node in the path
                intermediate_target_vertex = graph.nodes[path[1]];
                curr_vertex = graph.nodes[path[0]];
            }
        } else {
            // We're still on the current path segment
            RCLCPP_INFO(this->get_logger(),"pathHandle::Still at previous step");
            last_reached = path[0];
            intermediate_target_vertex = graph.nodes[path[1]];
            curr_vertex = graph.nodes[path[0]];
        }

        return std::make_pair(last_reached, is_target);
    }

    
    gbeam2_interfaces::msg::Vertex getCurrPos(){
        geometry_msgs::msg::TransformStamped l2g_tf;

        std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom"; //becasue lookupTransform doesn't allow "/" as first character
        std::string source_frame = name_space.substr(1, name_space.length()-1) + "/base_scan";
        try {
            
            //RCLCPP_INFO(this->get_logger(), "lookupTransform -------> %s to %s", target_frame.c_str(), source_frame.c_str());
            l2g_tf = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(
                this->get_logger(), "GBEAM:graph_update:lookupTransform: Could not transform %s to %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return gbeam2_interfaces::msg::Vertex();
        } 

        //create dummy vertex corresponding to robot position
        gbeam2_interfaces::msg::Vertex position;
        position = vert_transform(position, l2g_tf);
        return position ;  // create temporary vertex at robot position

    }

        
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationNode>());
  rclcpp::shutdown();
  return 0;
}