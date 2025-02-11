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
        this->declare_parameter<int>("N_robot",0);

        // Get parameter from yaml file
        reached_tol = this->get_parameter("reached_tol").get_parameter_value().get<float>();
        distance_exp = this->get_parameter("distance_exp").get_parameter_value().get<float>();
        limit_xi = this->get_parameter("limit_xi").get_parameter_value().get<double>();
        limit_xs = this->get_parameter("limit_xs").get_parameter_value().get<double>();
        limit_yi = this->get_parameter("limit_yi").get_parameter_value().get<double>();
        limit_ys = this->get_parameter("limit_ys").get_parameter_value().get<double>();

        N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();

        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF EXPLORATION NODE: ############# ");
        RCLCPP_INFO(this->get_logger(),"1) REACHED_TOL: %.2f", reached_tol);
        RCLCPP_INFO(this->get_logger(),"2) DISTANCE_EXP: %.2f", distance_exp);
        RCLCPP_INFO(this->get_logger(),"3) LIMIT_XI: %.2f", limit_xi);
        RCLCPP_INFO(this->get_logger(),"4) LIMIT_XS: %.2f", limit_xs);
        RCLCPP_INFO(this->get_logger(),"5) LIMIT_YI: %.2f", limit_yi);
        RCLCPP_INFO(this->get_logger(),"6) LIMIT_YS: %.2f", limit_ys);
        RCLCPP_INFO(this->get_logger(),"7) N_robot: %d", N_robot);
        
        new_mapping.resize(N_robot);
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
    int mapping_z = 5;
    int N_robot;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string name_space;
    
    gbeam2_interfaces::msg::Graph graph;
    std::vector<std::unordered_map<int,int>> new_mapping;
    std::vector<gbeam2_interfaces::msg::GraphAdjacency> getAdjMatrixof;

    // Local Indexes
    int start=-1, curr, next, target=-1;

    // Global nodes
    gbeam2_interfaces::msg::Vertex start_node, curr_node, next_node, target_node;

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

    std::pair<float,std::vector<int>>  dijkstraWithAdj(gbeam2_interfaces::msg::Graph graph, int s, int t, int belong_who)
    {
    int N = graph.nodes.size();
    std::vector<float> adjMatrix;
    int N_matrix;
    int belong_to;
    // In this way we consider only the nodes of the adj matrix to which 
    // the source s belong (and also the target t)
    //int belong_to = graph.nodes[s].belong_to;

    if(s>N) RCLCPP_ERROR(this->get_logger(),"Dijkstra:: Source Node out of bound");
    if(t>N) RCLCPP_ERROR(this->get_logger(),"Dijkstra:: Target Node out of bound");
    // Since we're considering only reachable node we skip the filtering part.

    std::vector<float> dist(N, INF);
    std::vector<int> parent(N, -1); // To store the shortest path tree
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

    if(graph.nodes[s].belong_to!=graph.nodes[t].belong_to){
        RCLCPP_ERROR(this->get_logger(),"Dijkstra:: Source and target belong to different path! Cannot compute path!");
        return std::make_pair(INF,path);
    }else{
        belong_to = graph.nodes[s].belong_to;
        adjMatrix = getAdjMatrixof[belong_to].data;
        N_matrix = getAdjMatrixof[belong_to].size;
    }

    //printMatrix(this->get_logger(),GraphAdj2matrix(graph.length_matrix));

    dist[s] = 0.0;
    pq.push({0.0, s});

    //RCLCPP_INFO(this->get_logger(),"Dijkstra:: algorithm initialization path from %d to %d - (local enum.)",s,t);

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
    
    int moveToBridge(gbeam2_interfaces::msg::Bridge target_bridge){
        // Modify the path to get to the target_bridge starting from curr_vertex
        int local_target;
        // Check on which side the bridge has to be accessed
        int bridge_end_id = (start_node.belong_to == target_bridge.r1) ? target_bridge.v1 : target_bridge.v2;
        int bridge_end_belong_to = (start_node.belong_to == target_bridge.r1) ? target_bridge.r1 : target_bridge.r2;

        RCLCPP_INFO(this->get_logger(),"MoveToBridge:: bridge end id: %d", bridge_end_id);

        auto it = new_mapping[bridge_end_belong_to].find(bridge_end_id);
        if(it!=new_mapping[bridge_end_belong_to].end()){
            // I have a corresponding mapping
            local_target = it->second;
            RCLCPP_INFO(this->get_logger(), "MoveToBridge:: bridge end remapping n: %d", local_target);
        }else{
            RCLCPP_WARN(this->get_logger(), "MoveToBridge:: No mapping for this bridge end.");
        } 
        RCLCPP_INFO(this->get_logger(),"MoveToBridge:: bridge end id: %d local target: %d", bridge_end_id,local_target);

        // Compute the path from the last reached node (curr_node) and the the end to that bridge (local_target)
        path = dijkstraWithAdj(graph, start, local_target,bridge_end_belong_to).second;
        globalpath.clear();
        for(auto n :path){
            globalpath.push_back(graph.nodes[n].id);
        }

        logIntVector(this->get_logger(),path,"MoveToBridge:: Path to "+ std::to_string(local_target));
        logIntVector(this->get_logger(),globalpath,"MoveToBridge:: Global Path to "+ std::to_string(local_target));
        return local_target;
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

    int getMapping(gbeam2_interfaces::msg::Vertex node){
        auto it = new_mapping[node.belong_to].find(node.id);
        if(it!=new_mapping[node.belong_to].end()){
            return it->second;
        }else{
            return -1;
        }
    }

    void sendPosRef(gbeam2_interfaces::msg::Vertex node){
        geometry_msgs::msg::PoseStamped pos_ref;

        pos_ref.pose.position = vertex2point(node);
        pos_ref.pose.position.z = mapping_z;
        pos_ref.header.frame_id = name_space.substr(1, name_space.length()-1) + "/odom";
        pos_ref_publisher_->publish(pos_ref);

    }

    void sendTargetRef(gbeam2_interfaces::msg::Vertex node){
        geometry_msgs::msg::PoseStamped target_ref;

        target_ref.pose.position    = vertex2point(node);
        target_ref.pose.position.z  = mapping_z;
        target_ref.header.frame_id  = name_space.substr(1, name_space.length()-1) + "/odom";
        target_ref_publisher_->publish(target_ref);

    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Task>> goal_handle)
    {
        rclcpp::Rate loop_rate(4); //4Hz - every 250ms

        const auto goal = goal_handle->get_goal();
        int tot_bridges = goal->target_bridge.size();
        // Same inizialization of graph callback global variables
        graph = goal->assigned_graph;
        N = graph.nodes.size();
        getAdjMatrixof = goal->adj_matrix;

        task_completed = false;
        int local_target =-1; // Local enumeration of nodes using index (last_reached follow the same logic)
        int crossed_bridges = 0;
        bool has_target_bridge = goal->has_target_bridge;
        
        geometry_msgs::msg::PoseStamped target_ref;
        
        auto feedback = std::make_shared<Task::Feedback>();        
        auto result = std::make_shared<Task::Result>();
        
        RCLCPP_INFO(this->get_logger(),"Execute goal: #################NEW GOAL################");
        RCLCPP_INFO(this->get_logger(),"Execute goal: explore cluster %d of robot%d",goal->cluster_id_task, goal->belong_to);
        RCLCPP_INFO(this->get_logger(),"Execute goal: ########################################");

        // -- Remapping all the nodes index
        for(int z=0;z<N_robot;z++){new_mapping[z].clear();}
        
        for (int n = 0; n < graph.nodes.size(); n++) {
           auto& node = graph.nodes[n];
           
           new_mapping[node.belong_to][node.id] = n;
        }
        // --

        // ##########################
        // START NODE IDENTIFICATION
        // ##########################
        if(start<0){
            // INIT  Get the starting node for this first task. Take the closest
            // Here we assume that at first the enumeration of local and global is the same  
            curr_node = vert_graph_distance_noobstacle(graph,getCurrPos()).second; 
            curr = curr_node.id;
            start=curr; start_node=curr_node;
            //RCLCPP_INFO(this->get_logger(), "INIT: I am on the node with id: global:%d", id); 


        }else{
            // MAIN 
            start = getMapping(start_node); 
            curr = getMapping(curr_node);
            next = getMapping(next_node);
            target = getMapping(target_node); 

            RCLCPP_INFO(this->get_logger(),"LAST_START: n:%d id:%d R%d || CURR: n:%d id:%d R%d || LAST_TARGET: n:%d id:%d R%d ", 
                                start,start_node.id,start_node.belong_to,
                                curr,curr_node.id,curr_node.belong_to,
                                target,target_node.id,target_node.belong_to);

            if(curr==target && curr!=-1){
                // The curr node is the last target. Previous goal completed, last target_reached
                start=target; start_node=target_node;
            } else if(curr==start && curr!=-1){
                // The curr node is the last start. Previous goal has not even started

            } else {
                // The curr node is the neither start or target. Previous goal has been interrupted
                start=curr; start_node=curr_node;//reset

            }

            if(start==-1){
                RCLCPP_WARN(this->get_logger(), "execute:: Last reached vertex not found in updated graph.");
                start_node = vert_graph_distance_noobstacle(graph,getCurrPos()).second;
                start   = getMapping(start_node);
                curr=start; curr_node =start_node;
                             
                RCLCPP_WARN(this->get_logger(),"execute:: Taking the closest node  n: %d id: %d R:", curr, curr_node.id, curr_node.belong_to);
            }

            RCLCPP_INFO(this->get_logger(),"NEW START: n:%d id:%d R%d || CURR: n:%d id:%d R%d || LAST_TARGET: n:%d id:%d R%d ", 
                                start,start_node.id,start_node.belong_to,
                                curr,curr_node.id,curr_node.belong_to,
                                target,target_node.id,target_node.belong_to);
           
        }

        // ##########################
        // TARGET NODE IDENTIFICATION
        // ##########################

        if(has_target_bridge){
            // I need to go to an external cluster and get to a bridge end to trespass it
            auto target_bridge = goal->target_bridge[crossed_bridges];
            RCLCPP_INFO(this->get_logger(),"execute:: Move to target BRIDGE %d/%d from (n: %d cl: %d of R%d ) to (n: %d cl: %d of R%d) length %.2f ", 
                                                            crossed_bridges+1, tot_bridges,
                                                            target_bridge.v1, target_bridge.c1, target_bridge.r1,
                                                            target_bridge.v2, target_bridge.c2, target_bridge.r2, target_bridge.length);
            target = moveToBridge(target_bridge);
            target_node = graph.nodes[target];

        }else{
            // Client doesn't specify any particular target node, just need to explore a cluster
            // At start select the best node and compute the path to it
            //RCLCPP_INFO(this->get_logger(),"execute:: Search for the best Node");

            auto bestpair = getBestNode(goal->cluster_id_task, goal->belong_to);

            target          =   bestpair.first;
            target_node     =   graph.nodes[target];
            path            =   bestpair.second;
     
            
        }

        if(path.empty()){
            // Modify here the result information
            result->last_reached = curr_node;
            result->task_completed = task_completed;
            goal_handle->abort(result);
            active_goal_handle_ = nullptr; // Clear the active goal
            RCLCPP_INFO(this->get_logger(), "Goal cannot be completed - empty path to local node %d",local_target);
            return;
        
        }

        // Publish position reference to controller
        sendTargetRef(target_node);
        
        next        = (path.size()>1) ? path[1]: target;
        next_node   = graph.nodes[next];


        if (target < N) {
            sendPosRef(next_node);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid last_target_index after path computation: %d", target);
        }

        // ##########################################
        // ##### Assigned Graph Navigation ##########
        // ##########################################

        while(!task_completed && rclcpp::ok()){

            if (goal_handle->is_canceling()) {
                result->last_reached = curr_node;
                result->task_completed = false;               
                goal_handle->canceled(result);
                //RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
          
            auto [last_reached,is_local_target] = pathHandle(getCurrPos());

            if(is_local_target){

                //curr = target; curr_node=target_node;

                if(has_target_bridge){
                    // And end of a bridge has been reached as a local target and bridge has to be crossed
                    // with according graph switch

                    RCLCPP_INFO(this->get_logger(),"execute:: one Bridge end is reached! ");

                    auto& trasp_bridge = goal->target_bridge[crossed_bridges];

                    RCLCPP_INFO(this->get_logger(),"execute:: Last reached node belong to: %d",curr_node.belong_to );

                    // Enumeration based on the owner of the OTHER end
                    int bridge_end_id = (curr_node.belong_to == trasp_bridge.r1) ? trasp_bridge.v2 : trasp_bridge.v1;
                    int bridge_end_belong_to = (curr_node.belong_to == trasp_bridge.r1) ? trasp_bridge.r2 : trasp_bridge.r1; 

                    RCLCPP_INFO(this->get_logger(),"execute:: Trespass to other end... node id: %d belong to: %d",bridge_end_id,bridge_end_belong_to);

                
                   
                    // Recompute the new curr vertex of the new side of the bridge
                    auto it = new_mapping[bridge_end_belong_to].find(bridge_end_id);
                    if(it!=new_mapping[bridge_end_belong_to].end()){
                        // I have a corresponding mapping
                        start = it->second;
                        start_node =graph.nodes[start];
                        RCLCPP_INFO(this->get_logger(),"execute:: New Mapping of the bridge end is n: %d id: %d", new_mapping[bridge_end_belong_to][bridge_end_id], bridge_end_id);
                    }else{
                        start_node = vert_graph_distance_noobstacle(graph,getCurrPos()).second; 
                        start = start_node.id;
                       
                        RCLCPP_WARN(this->get_logger(),"execute:: No mapping for bridge end. Taking the closest node  n: %d id: %d R:", curr, curr_node.id, curr_node.belong_to);
                    } 


                    // graph.length_matrix = getAdjMatrixof[curr_node.belong_to];
                    // graph.adj_matrix    = getAdjMatrixof[curr_node.belong_to];

                    // Verify that all the bridges has been crossed
                    crossed_bridges++;
                    if(crossed_bridges>=graph.cluster_graph.bridges.size()){
                        // If all the bridges are crossed i need to compute the best node 
                        // among the one of the target cluster and switch adjacency matrix
                        

                        auto bestpair = getBestNode(goal->cluster_id_task,goal->belong_to);

                        target          =   bestpair.first;
                        target_node     =   graph.nodes[target];
                        path            =   bestpair.second;

                        RCLCPP_INFO(this->get_logger(),"I from start: n:%d id:%d R%d || I want to go to TARGET: n:%d id:%d R%d but CURR: n:%d id:%d R%d ",
                                                                start,start_node.id,start_node.belong_to,
                                                                curr,curr_node.id,curr_node.belong_to,
                                                                target,target_node.id,target_node.belong_to);
                        // Add a dummy vertex at the begin of the path, in order to have the start end bridge as second element 
                        path.insert(path.begin(), curr);

                        //curr=start; curr_node=start_node;

                        globalpath.clear();
                        for(auto n :path){
                            globalpath.push_back(graph.nodes[n].id);
                        }

                        logIntVector(this->get_logger(),path,"execute:: After bridge Path to "+ std::to_string(local_target));
                        logIntVector(this->get_logger(),globalpath,"execute:: After bridge Global Path to "+ std::to_string(local_target));

                        has_target_bridge = false;

                        if(path.empty()){
                            // Modify here the result information
                            result->last_reached = curr_node;
                            result->task_completed = task_completed;
                            goal_handle->abort(result);
                            active_goal_handle_ = nullptr; // Clear the active goal
                            RCLCPP_INFO(this->get_logger(), "Goal cannot be completed - empty path to local node %d",local_target);
                            return;
                        
                        }
                        // Publish position reference to controller
                        sendTargetRef(target_node);

                        next        = (path.size()>1) ? path[1]: target;
                        next_node   = graph.nodes[next];
                    }else{
                        // Here i need to switch from the previous adj matrix to the one required after
                        // the bridge is crossed
                        auto new_target_bridge = goal->target_bridge[crossed_bridges];
                        RCLCPP_INFO(this->get_logger(),"execute:: Move to target BRIDGE %d/%d from (n: %d cl: %d of R%d ) to (n: %d cl: %d of R%d) length %.2f ", 
                                                crossed_bridges+1, tot_bridges,
                                                new_target_bridge.v1, new_target_bridge.c1, new_target_bridge.r1,
                                                new_target_bridge.v2, new_target_bridge.c2, new_target_bridge.r2, new_target_bridge.length);

                    
                        target = moveToBridge(new_target_bridge);
                        target_node = graph.nodes[target];
                    }
                }else{  

                    task_completed=true; 
                    target = last_reached;               
                }
                

                

            }else{          
  
                // Publish position reference to controller
                sendPosRef(next_node);
            }
            
            feedback->curr_vert = curr_node;
            goal_handle->publish_feedback(feedback);
            
            loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            // Modify here the result information
            result->last_reached = curr_node;
            result->task_completed = task_completed;
            goal_handle->succeed(result);
            active_goal_handle_ = nullptr; // Clear the active goal
            //RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        
    }

    std::pair<int, std::vector<int>> getBestNode(int cluster_id, int belong_to){
        // Get the Best node among the one inside the specified cluster
        // Consider starting node the curr_node
        float max_reward = 0.0;
        int best_node = -1;
        std::vector<int> bestPath;


        for(size_t n = 0; n < N; n++)
        {
            if(n!=start)
            {
                if(graph.nodes[n].belong_to!=belong_to){ //&& graph.nodes[n].gain>0.0
                    //RCLCPP_INFO(this->get_logger(),"Node %d: id: %d does not belong to goal belong_to R%d", n, graph.nodes[n].id, belong_to);
                    // Skip unreachable nodes and the ones that doesn't belong to the specified cluster
                    continue;  
                }else if(graph.nodes[n].cluster_id!=cluster_id){
                    //RCLCPP_INFO(this->get_logger(),"Node %d: id: %d does not belong to cluster %d of R%d", n, graph.nodes[n].id,cluster_id, belong_to);
                    continue;

                }else if (graph.nodes[n].gain==0.0)
                {
                    //RCLCPP_INFO(this->get_logger(),"Node %d: id: %d has zero gain", n, graph.nodes[n].id);
                    // Skip unreachable nodes and the ones that doesn't belong to the specified cluster
                    continue; 
                }
                
                
                auto [distance, path] =  dijkstraWithAdj(graph,start,n,belong_to);
                // logIntVector(this->get_logger(),path,"getBestNode:: Path to "+std::to_string(n));

                // globalpath.clear();
                // for(auto n :path){
                //     globalpath.push_back(graph.nodes[n].id);
                // }

                
                // logIntVector(this->get_logger(),globalpath,"getBestNode::Global Path to "+ std::to_string(n));
                float reward = graph.nodes[n].gain / std::pow(distance, distance_exp);
                //RCLCPP_INFO(this->get_logger(),"getBestNode:: Node %d: id: %d || R%d - C%d || reward: %.2f distance:%.2f", n, graph.nodes[n].id, graph.nodes[n].belong_to, graph.nodes[n].cluster_id,reward, distance);
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
            return std::make_pair(start,std::vector<int>());;
        }

        RCLCPP_INFO(this->get_logger(), "getBestNode:: Best node -> local id: %d global id: %d", best_node, graph.nodes[best_node].id);
        logIntVector(this->get_logger(),bestPath,"getBestNode:: Path to "+std::to_string(best_node));

                globalpath.clear();
                for(auto n :bestPath){
                    globalpath.push_back(graph.nodes[n].id);
                }

                
                logIntVector(this->get_logger(),globalpath,"getBestNode::Global Path to "+ std::to_string(best_node));

        return std::make_pair(best_node,bestPath);
    }

    std::pair<int, bool> pathHandle(gbeam2_interfaces::msg::Vertex pos) {
        int last_reached = -1; // Ensure it's initialized
        bool is_target = false;

        // Check if path is not empty
        if (path.empty()) {
            return std::make_pair(-1, is_target); // Return invalid value if path is empty
        }

        // if(path.size()==1 && target==path[0]){
        //     RCLCPP_INFO(this->get_logger(),"I'm already in target!");
        //     last_reached = target;
        //     return std::make_pair(last_reached, true);
        // }

        // Check if the current position has reached the next vertex in the path
        if (dist(next_node, pos) <= reached_tol) {
            // We've reached the next node in the path
            last_reached = path[1];  // The node we just reached

            // Remove the first element of the path
            path.erase(path.begin());

            // If we reach the end of the path, set is_target to true
            if (path.empty()) {
                is_target = true;
                curr = target;
            } else {
                // Set the intermediate target as the next node in the path
                next = path[1]; 
                curr = path[0];
            }
        } else {
            // We're still on the current path segment
            last_reached = path[0];
            next = path[1]; 
            curr = path[0];
        }

        curr_node  = graph.nodes[curr];
        next_node  = graph.nodes[next];

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