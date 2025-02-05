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
        // graph_subscriber_ = this->create_subscription<gbeam2_interfaces::msg::Graph>(
        //     "gbeam/reachability_graph", 1, std::bind(&ExplorationNode::graphCallback, this, std::placeholders::_1));

        //"coop/assigned_graph"   OR "gbeam/reachability_graph" 

        pos_ref_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "gbeam/gbeam_pos_ref", 1);
        target_ref_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "gbeam/target_pos_ref", 1);
    
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
    std::vector<gbeam2_interfaces::msg::GraphAdjacency> getAdjMatrixof;
    gbeam2_interfaces::msg::Vertex last_target_vertex;
    gbeam2_interfaces::msg::Vertex intermediate_target_vertex;
    gbeam2_interfaces::msg::Vertex curr_vertex;
    std::vector<int> path;
    std::vector<int> globalpath;

    // Action
    rclcpp_action::Server<Task>::SharedPtr action_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Task>> active_goal_handle_;
    bool task_completed = false;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pos_ref_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_ref_publisher_;
    rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr graph_subscriber_;

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
    
    void logIntVector(rclcpp::Logger logger, const std::vector<int>& vec, const std::string& vec_name = "vector") {
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

    std::pair<float,std::vector<int>>  dijkstraWithAdj(gbeam2_interfaces::msg::Graph graph, int s, int t, int belong_to)
    {
    int N = graph.nodes.size();
    auto adjMatrix = graph.length_matrix.data;
    int N_matrix = graph.length_matrix.size;
    // In this way we consider only the nodes of the adj matrix to which 
    // the source s belong (and also the target t)
    //int belong_to = graph.nodes[s].belong_to;

    // Since we're considering only reachable node we skip the filtering part.

    std::vector<float> dist(N, INF);
    std::vector<int> parent(N, -1); // To store the shortest path tree
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    //printMatrix(this->get_logger(),GraphAdj2matrix(graph.length_matrix));

    dist[s] = 0.0;
    pq.push({0.0, s});

    //RCLCPP_INFO(this->get_logger(),"Dijkstra:: algorithm inizialization");

        while (!pq.empty()) {
            auto [currentDist, u] = pq.top();
            pq.pop();

            // If the distance is already larger, skip
            if (currentDist > dist[u] || graph.nodes[u].belong_to!=belong_to) continue;

            int u_id = graph.nodes[u].id;

            // Explore neighbors
            for (int v = 0; v < N; v++) {
                if(v==u || graph.nodes[v].belong_to!=belong_to) continue;
                int v_id = graph.nodes[v].id;
                float weight = adjMatrix[u_id*N_matrix + v_id]; // adj.data[i * N + j] = matrix[i][j];
                if (weight > 0.0 && dist[u] + weight < dist[v]) {
                    dist[v] = dist[u] + weight;
                    parent[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }

        // Reconstruct path with strict group validation
        std::vector<int> path;
        for (int at = t; at != -1; at = parent[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end()); // Reverse to get the path from source to target

        // Check if the path starts with the source
        if (!path.empty() && path[0] == s) {
            return std::make_pair(dist[t],path);
        }else{
            return std::make_pair(INF,path);
        } // Return empty if there's no valid path
    }
    
    void moveToBridge(gbeam2_interfaces::msg::Bridge target_bridge, int& local_target){
        // Modify the path to get to the target_bridge starting from last_target
        // Also modify local_target variable
        int bridge_end_id = (last_target_vertex.belong_to == target_bridge.r1) ? target_bridge.v1 : target_bridge.v2;
        int bridge_end_belong_to = (last_target_vertex.belong_to == target_bridge.r1) ? target_bridge.r1 : target_bridge.r2;
        //graph.adj_matrix = getAdjMatrixof[last_target_vertex.belong_to];

        RCLCPP_INFO(this->get_logger(),"MoveToBridge:: bridge end id: %d", bridge_end_id);
             
            for(int i=0; i<graph.nodes.size();i++){
                if(graph.nodes[i].belong_to==bridge_end_belong_to && graph.nodes[i].id==bridge_end_id) {
                    local_target = i; 
                    break;
                }
            }
        RCLCPP_INFO(this->get_logger(),"MoveToBridge:: bridge end id: %d local target: %d", bridge_end_id,local_target);

        path = dijkstraWithAdj(graph, last_target, local_target,bridge_end_belong_to).second;
        globalpath.clear();
        for(auto n :path){
            globalpath.push_back(graph.nodes[n].id);
        }

        logIntVector(this->get_logger(),path,"MoveToBridge:: Path to "+ std::to_string(local_target));
        logIntVector(this->get_logger(),globalpath,"MoveToBridge:: Global Path to "+ std::to_string(local_target));
    }
    // Actions routines

    // Handle incoming goal requests
    rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Task::Goal> goal)
    {
        //RCLCPP_INFO(this->get_logger(), "Received goal request");

        // Reject the goal if another goal is active
        if (active_goal_handle_)
        {
            //RCLCPP_WARN(this->get_logger(), "Another goal is already active. Rejecting this goal.");
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
        rclcpp::Rate loop_rate(4); //4Hz - every 250ms

        const auto goal = goal_handle->get_goal();
        // Same inizialization of graph callback global variables
        graph = goal->assigned_graph;
        N = graph.nodes.size();
        getAdjMatrixof = goal->adj_matrix;

        task_completed = false;
        int local_target =-1; // Local enumeration of nodes using index (last_reached follow the same logic)
        int trespassed_bridges = 0;
        bool has_target_bridge = goal->has_target_bridge;
        geometry_msgs::msg::PoseStamped pos_ref;
        geometry_msgs::msg::PoseStamped target_ref;
        
        auto feedback = std::make_shared<Task::Feedback>();        
        auto result = std::make_shared<Task::Result>();
        
        RCLCPP_INFO(this->get_logger(),"Execute goal: #################NEW GOAL################");
        RCLCPP_INFO(this->get_logger(),"Execute goal: explore cluster %d of robot%d",goal->cluster_id_task, goal->belong_to);
        RCLCPP_INFO(this->get_logger(),"Execute goal: ########################################");

        if(last_target<0){
            // Inizialization, node is just started or the previous action has been cancelled 
            // Get the node on which i am
            // Here we assume that at first the enumeration of local and global is the same
            auto [dist, curr_vertex] = vert_graph_distance_noobstacle(graph,getCurrPos()); 
            last_target_vertex = curr_vertex;
            last_target = curr_vertex.id;
            //RCLCPP_INFO(this->get_logger(), "INIT: I am on the node with id: global:%d", id);        
        }else{
            bool found_last_target = false;
            for (int n = 0; n < graph.nodes.size(); n++) {
                if (graph.nodes[n].id == curr_vertex.id && 
                    graph.nodes[n].belong_to == curr_vertex.belong_to) {
                    last_target = n;
                    found_last_target = true;
                    RCLCPP_INFO(this->get_logger(), "execute:: Last reached node was n: %d id: %d", last_target, last_target_vertex.id);
                    break;
                }
            }
            if (!found_last_target) {
                RCLCPP_WARN(this->get_logger(), "execute:: Last reached vertex not found in updated graph. Resetting to default.");
                auto [dist, curr_vertex] = vert_graph_distance_noobstacle(graph,getCurrPos()); 
                last_target_vertex = curr_vertex;
                last_target = curr_vertex.id;
                RCLCPP_WARN(this->get_logger(),"execute:: Selected node  n: %d id: %d", last_target, last_target_vertex.id);
                if(last_target_vertex.belong_to!=goal->belong_to) RCLCPP_INFO(this->get_logger(), "The closest node is not belonging to the goal");
            }

            
        }

        if(has_target_bridge){
            // I need to go to an external cluster and get to a bridge end to traspass it
            RCLCPP_INFO(this->get_logger(),"execute:: goal has a %d target bridge",goal->target_bridge.size());

            auto target_bridge = goal->target_bridge[trespassed_bridges];
            RCLCPP_INFO(this->get_logger(),"execute:: Target BRIDGE from (n: %d cl: %d of R%d ) to (n: %d cl: %d of R%d) length %f ", 
                                                            target_bridge.v1, target_bridge.c1, target_bridge.r1,
                                                            target_bridge.v2, target_bridge.c2, target_bridge.r2, target_bridge.length);
            moveToBridge(target_bridge, local_target);

        }else{
            // Client doesn't specify any particular target node, just need to explore a cluster
            // At start select the best node and compute the path to it
            //RCLCPP_INFO(this->get_logger(),"execute:: Search for the best Node");

            auto bestpair = getBestNode(goal->cluster_id_task, goal->belong_to);

            local_target    =   bestpair.first;
            path            =   bestpair.second;

            
            
        }

        // Publish position reference to controller
        target_ref.pose.position    = vertex2point(graph.nodes[local_target]);
        target_ref.pose.position.z  = mapping_z;
        target_ref.header.frame_id  = name_space.substr(1, name_space.length()-1) + "/odom";
        target_ref_publisher_->publish(target_ref);

        if(path.empty()){
            // Modify here the result information
            result->last_reached = curr_vertex;
            result->task_completed = task_completed;
            goal_handle->abort(result);
            active_goal_handle_ = nullptr; // Clear the active goal
            RCLCPP_INFO(this->get_logger(), "Goal cannot be completed - empty path to local node %d",local_target);
            return;
        
        }

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

        // ##########################################
        // ##### Assigned Cluster Exploration #######
        // ##########################################

        // Explore the assigned cluster 
        // or current action is aborted
        // OR IF I HAVE A TARGET BRIDGE
        // once the bridge end is reached

        while(!task_completed && rclcpp::ok()){

            if (goal_handle->is_canceling()) {
                result->last_reached = curr_vertex;
                result->task_completed = false;               
                goal_handle->canceled(result);
                //RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
          
            auto [last_reached,is_local_target] = pathHandle(getCurrPos());

            if(is_local_target){
                curr_vertex = last_target_vertex;
                if(has_target_bridge){
                    // Trespass bridge
                    // Maybe is not needed? 

                    // Update curr_vertex and last_target
                    RCLCPP_INFO(this->get_logger(),"execute:: one Bridge end is reached! ");

                    auto& trasp_bridge = goal->target_bridge[trespassed_bridges];

                    RCLCPP_INFO(this->get_logger(),"execute:: Last target Vertex belong to: %d",last_target_vertex.belong_to );

                    // Enumeration based on the owner of the end
                    int bridge_end_id = (last_target_vertex.belong_to == trasp_bridge.r1) ? trasp_bridge.v2 : trasp_bridge.v1;
                    int bridge_end_belong_to = (last_target_vertex.belong_to == trasp_bridge.r1) ? trasp_bridge.r2 : trasp_bridge.r1; 

                    RCLCPP_INFO(this->get_logger(),"execute:: Trespass to other end... node id: %d belong to: %d",bridge_end_id,bridge_end_belong_to);

                    //intermediate_target_vertex = graph.nodes[bridge_end_id];

                    
                    
                        
                    for(int i=0; i<graph.nodes.size();i++){
                        if(graph.nodes[i].belong_to==bridge_end_belong_to && graph.nodes[i].id==bridge_end_id) {
                            local_target = i; 
                            intermediate_target_vertex = graph.nodes[i];
                                                        
                            break;
                        }
                    }

                    RCLCPP_INFO(this->get_logger(),"execute:: Trespass to other end... node local id: %d",local_target);

                    // // Publish position reference to controller
                    // pos_ref.pose.position = vertex2point(intermediate_target_vertex);
                    // pos_ref.pose.position.z = mapping_z;
                    // pos_ref.header.frame_id = name_space.substr(1, name_space.length()-1) + "/odom";
                    // pos_ref_publisher_->publish(pos_ref);

                    // rclcpp::sleep_for(std::chrono::milliseconds(1500));

                    last_target = local_target;

                    graph.length_matrix = getAdjMatrixof[intermediate_target_vertex.belong_to];
                    graph.adj_matrix    = getAdjMatrixof[intermediate_target_vertex.belong_to];

                    //printMatrix(this->get_logger(),GraphAdj2matrix(getAdjMatrixof[intermediate_target_vertex.belong_to]),"Adj Matrix of "+std::to_string(intermediate_target_vertex.belong_to));

                    // Verify that all the bridges has been trespassed
                    trespassed_bridges++;
                    if(trespassed_bridges>=graph.cluster_graph.bridges.size()){
                        // If all the bridges are trespassed i need to compute the best node 
                        // among the one of the target cluster and switch adjacency matrix
                        //graph.adj_matrix = goal->adj_matrix[goal->belong_to];

                        RCLCPP_INFO(this->get_logger(),"size of the matrix of robot%d: %d",goal->belong_to,graph.length_matrix.size);

                        //printMatrix(this->get_logger(),graph.adj_matrix);

                        auto bestpair = getBestNode(goal->cluster_id_task,goal->belong_to);

                        local_target    =   bestpair.first;
                        path            =   bestpair.second;

                        RCLCPP_INFO(this->get_logger(),"I want to go to node id: %d of robot%d in cluster:: %d",graph.nodes[local_target].id,graph.nodes[local_target].belong_to, graph.nodes[local_target].cluster_id);
                        
                        // Add a dummy vertex at the begin of the path, in order to have the end bridge 
                        path.insert(path.begin(), 0);

                        globalpath.clear();
                        for(auto n :path){
                            globalpath.push_back(graph.nodes[n].id);
                        }

                        logIntVector(this->get_logger(),path,"execute:: After bridge Path to "+ std::to_string(local_target));
                        logIntVector(this->get_logger(),globalpath,"execute:: After bridge Global Path to "+ std::to_string(local_target));

                        has_target_bridge = false;

                        // Publish position reference to controller
                        target_ref.pose.position    = vertex2point(graph.nodes[local_target]);
                        target_ref.pose.position.z  = mapping_z;
                        target_ref.header.frame_id  = name_space.substr(1, name_space.length()-1) + "/odom";
                        target_ref_publisher_->publish(target_ref);

                        if(path.empty()){
                            // Modify here the result information
                            result->last_reached = curr_vertex;
                            result->task_completed = task_completed;
                            goal_handle->abort(result);
                            active_goal_handle_ = nullptr; // Clear the active goal
                            RCLCPP_INFO(this->get_logger(), "Goal cannot be completed - empty path to local node %d",local_target);
                            return;
                        
                        }

                        last_target_vertex = graph.nodes[local_target];
                        intermediate_target_vertex = (path.size()>1) ? graph.nodes[path[1]]: last_target_vertex;
                        //task_completed =true;
                    }else{
                        // Here i need to switch from the previous adj matrix to the one required after
                        // the bridge is trespassed
                        moveToBridge(goal->target_bridge[trespassed_bridges], local_target);
                    }
                }else{  

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
            //RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        
    }

    std::pair<int, std::vector<int>> getBestNode(int cluster_id, int belong_to){
        // Get the Best node among the one inside the specified cluster
        float max_reward = 0.0;
        int best_node = -1;
        std::vector<int> bestPath;


        for(size_t n = 0; n < N; n++)
        {
            if(n!=last_target)
            {
                if(graph.nodes[n].belong_to!=belong_to && graph.nodes[n].cluster_id!=cluster_id || graph.nodes[n].belong_to==belong_to && graph.nodes[n].cluster_id!=cluster_id ){ //&& graph.nodes[n].gain>0.0
                    RCLCPP_INFO(this->get_logger(),"Node %d: id: %d does not belong to cluster %d of R%d", n, graph.nodes[n].id,cluster_id, belong_to);
                    // Skip unreachable nodes and the ones that doesn't belong to the specified cluster
                    continue;  
                }else if (graph.nodes[n].gain==0.0)
                {
                    RCLCPP_INFO(this->get_logger(),"Node %d: id: %d has zero gain", n, graph.nodes[n].id);
                    // Skip unreachable nodes and the ones that doesn't belong to the specified cluster
                    continue; 
                }
                
                
                auto [distance, path] =  dijkstraWithAdj(graph,last_target,n,belong_to);
                logIntVector(this->get_logger(),path,"getBestNode:: Path to "+std::to_string(n));

                globalpath.clear();
                for(auto n :path){
                    globalpath.push_back(graph.nodes[n].id);
                }

                
                logIntVector(this->get_logger(),globalpath,"getBestNode::Global Path to "+ std::to_string(n));
                float reward = graph.nodes[n].gain / std::pow(distance, distance_exp);
                RCLCPP_INFO(this->get_logger(),"getBestNode:: Node %d: id: %d || R%d - C%d || reward: %f distance:%f", n, graph.nodes[n].id, graph.nodes[n].belong_to, graph.nodes[n].cluster_id,reward, distance);
                if(reward > max_reward)
                {
                    max_reward = reward;
                    best_node = n;
                    bestPath = path;
                }
            }
        }

        if(best_node<0){
            RCLCPP_WARN(this->get_logger(),"getBestNode:: No best node has been found");
            return std::make_pair(last_target,std::vector<int>());;
        }

        RCLCPP_INFO(this->get_logger(), "getBestNode:: Best node -> local id: %d global id: %d", best_node, graph.nodes[best_node].id);

        return std::make_pair(best_node,bestPath);
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
            //RCLCPP_INFO(this->get_logger(),"pathHandle::Reached intermediate target!");
            last_reached = path[1];  // The node we just reached

            // Modify last_target and reduce path
            path.erase(path.begin()); // Remove the first element of the path

            // If we reach the end of the path, set is_target to true
            if (path.empty()) {
                //RCLCPP_INFO(this->get_logger(),"pathHandle::Reached end of the path!");
                is_target = true;
                curr_vertex=last_target_vertex;
            } else {
                // Set the intermediate target as the next node in the path
                intermediate_target_vertex = graph.nodes[path[1]];
                curr_vertex = graph.nodes[path[0]];
            }
        } else {
            // We're still on the current path segment
            //RCLCPP_INFO(this->get_logger(),"pathHandle::Still at previous step");
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