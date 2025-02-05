//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread> // Include for threading
#include <future> // Include for promises and futures
#include <mutex>
#include <condition_variable>

#include "gbeam2_interfaces/msg/vertex.hpp"
#include "gbeam2_interfaces/msg/graph_edge.hpp"
#include "gbeam2_interfaces/msg/poly_area.hpp"
#include "gbeam2_interfaces/msg/graph.hpp"
#include "gbeam2_interfaces/msg/free_polygon.hpp"
#include "gbeam2_interfaces/msg/free_polygon_stamped.hpp"
#include "gbeam2_interfaces/msg/graph_update.hpp"
#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/global_map.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "gbeam2_interfaces/msg/status.hpp"

#include "gbeam2_interfaces/srv/graph_update.hpp"
#include "library_fcn.hpp"

using namespace std::chrono_literals;

class GraphMergerNode : public rclcpp::Node
{
public:
    GraphMergerNode() : Node("partial_graph_merger")
    {

    cb_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);        
    cb_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timers_cb_group = cb_group_1;

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_1;

    // SUBSCRIBED TOPICS
    graph_subscriber_ = this->create_subscription<gbeam2_interfaces::msg::Graph>(
                "gbeam/reachability_graph", 1, std::bind(&GraphMergerNode::switchCallback, this, std::placeholders::_1),sub_options);

    status_sub_ = this->create_subscription<gbeam2_interfaces::msg::Status>(
        "/status",1,std::bind(&GraphMergerNode::statusCallback, this, std::placeholders::_1));

    // PUBLISHING TOPICS
    merged_graph_pub_ = this->create_publisher<gbeam2_interfaces::msg::GlobalMap>(
                "gbeam/merged_graph", 1);
    external_updates_pub_ = this->create_publisher<gbeam2_interfaces::msg::GraphUpdate>(
                "external_nodes", 1);
    timer_pub_ =this->create_publisher<std_msgs::msg::Float32MultiArray>(
                "timers",1);

    // SERVICES
    graph_updates_service_ = this->create_service<gbeam2_interfaces::srv::GraphUpdate>(
        "getGraphUpdates",std::bind(&GraphMergerNode::serverCallback,this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,cb_group_2);
    start_merger_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_merger",std::bind(&GraphMergerNode::startMerger,this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,cb_group_2);


    timer_ptr_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&GraphMergerNode::periodicTimerCallback, this),
                                            timers_cb_group);
    

    // Get namespace
    name_space = this->get_namespace();
    name_space_id = name_space.back()- '0';

    // Initialize parameters
    this->declare_parameter<int>("N_robot",0);
    this->declare_parameter<double>("communication_range",0.0);
    this->declare_parameter<int>("periodic_call_time",0);
    this->declare_parameter<int>("max_no_connection_time", 0);

    // Get parameters
    N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
    wifi_range = this->get_parameter("communication_range").get_parameter_value().get<double>();
    periodic_call_time =this->get_parameter("periodic_call_time").get_parameter_value().get<int>();
    max_no_connection_time = this->get_parameter("max_no_connection_time").get_parameter_value().get<int>();

    RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF PARTIAL_GRAPH_MERGER: ############# ");
    RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
    RCLCPP_INFO(this->get_logger(),"1) Number of robots: %d",N_robot);
    RCLCPP_INFO(this->get_logger(),"2) Wifi Range: %f",wifi_range);
    // Initialize vectors with the correct size
    curr_updateBuffer.resize(N_robot);
    prev_updateBuffer.resize(N_robot);
    last_status.resize(N_robot);
    timers_CLIENTS.resize(N_robot);
    tracking_timers.data.resize(N_robot);
    last_update_node_with.resize(N_robot);
    global_map.map.resize(N_robot);
    global_map.last_updater=name_space_id;
    graphBuffer.resize(N_robot);

    graph_updates_CLIENTS.resize(N_robot);


    for (int i = 0; i < N_robot; ++i) {
        global_map.map[i] = gbeam2_interfaces::msg::Graph();
        global_map.map[i].robot_id = i;
        graphBuffer[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
        graphBuffer[i]->robot_id = name_space_id;
        curr_updateBuffer[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
        curr_updateBuffer[i]->robot_id = name_space_id;
        prev_updateBuffer[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
        prev_updateBuffer[i]->robot_id = name_space_id;



        last_update_node_with[i] = -1;
    }
   

    // Create a client and a timer for each robot
    std::string service_name;
    for (int i = 0; i < N_robot; i++)
    {   
        service_name = "/robot"+std::to_string(i)+"/getGraphUpdates";
        if (i!=name_space_id){
            RCLCPP_INFO(this->get_logger(),"Initialize client: %s",service_name.c_str());  
            graph_updates_CLIENTS[i]=this->create_client<gbeam2_interfaces::srv::GraphUpdate>(service_name);
            RCLCPP_INFO(this->get_logger(),"Initialize timer for robot%d",i);
            timers_CLIENTS[i] = this->create_wall_timer(std::chrono::seconds(max_no_connection_time), [this,i]() -> void { timeoutCallback(i);},
                                            timers_cb_group);
            timers_CLIENTS[i]->cancel();
        }   
        else{
            graph_updates_CLIENTS[i]= std::shared_ptr<rclcpp::Client<gbeam2_interfaces::srv::GraphUpdate>>();  
            timers_CLIENTS[i] = rclcpp::TimerBase::SharedPtr();        
        } 
        

    }
     

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    };

private:
    std::string name_space;
    int name_space_id;
    bool start_merging = false;
    bool is_gain_updated = false;

    // Parameters variables
    int N_robot;
    double wifi_range;
    int periodic_call_time;
    int max_no_connection_time;
    std::vector<gbeam2_interfaces::msg::Status> last_status;

    rclcpp::CallbackGroup::SharedPtr cb_group_1;        
    rclcpp::CallbackGroup::SharedPtr cb_group_2;
    rclcpp::CallbackGroup::SharedPtr timers_cb_group;

    rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr graph_subscriber_;
    rclcpp::Subscription<gbeam2_interfaces::msg::Status>::SharedPtr status_sub_;
    rclcpp::Publisher<gbeam2_interfaces::msg::GlobalMap>::SharedPtr merged_graph_pub_;
    rclcpp::Publisher<gbeam2_interfaces::msg::GraphUpdate>::SharedPtr external_updates_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr timer_pub_;
    rclcpp::Service<gbeam2_interfaces::srv::GraphUpdate>::SharedPtr  graph_updates_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_merger_service_;
    std::vector<rclcpp::Client<gbeam2_interfaces::srv::GraphUpdate>::SharedPtr> graph_updates_CLIENTS;

    rclcpp::TimerBase::SharedPtr timer_ptr_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_CLIENTS;
    std_msgs::msg::Float32MultiArray tracking_timers;
    std_msgs::msg::Float32MultiArray last_connection_with;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    std::mutex mutex_;
    std::condition_variable cv_;
    bool data_received_ = false;
    gbeam2_interfaces::srv::GraphUpdate::Response updateResponse;
    bool buffer_clean_ =false;

    // Local copies of all the map of other robots
    gbeam2_interfaces::msg::GlobalMap global_map;
    std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> graphBuffer;
    std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> curr_updateBuffer;
    std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> prev_updateBuffer;
    std::vector<int> last_update_node_with;
    

    geometry_msgs::msg::TransformStamped getTransform(std::string target_frame,std::string source_frame){

        try {
            
            //RCLCPP_INFO(this->get_logger(), "lookupTransform -------> %s to %s", target_frame.c_str(), source_frame.c_str());
            return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero); 
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(
                this->get_logger(), "GBEAM:graph_update:lookupTransform: Could not transform %s to %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
          //return;
        }
  }

    void startMerger(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        start_merging=request->data;
        if(start_merging){
            for (int i = 0; i < N_robot; i++){   
            if (i!=name_space_id){
                timers_CLIENTS[i]->reset();
                }    
            }
            response->success = true;
            response->message = "Start merging for " + name_space;
        }else{
            response->success = false;
            response->message = "Stop merging for" + name_space;

        }
        

        
        

    }

    void statusCallback(const std::shared_ptr<gbeam2_interfaces::msg::Status> status){
        // This function compute features for every robot in the network
        // RCLCPP_INFO(this->get_logger(), "Status received");

        // Check connection status 
        if(status->connection_status[name_space_id]!=1) return;
        last_status[status->robot_id]=*status;

        double node_dist_open = 0.3; //node_dist_open: 0.3
        // Update exploration gain based on the position and the current occupied cluster
        if(global_map.map[status->current_cluster.belong_to].nodes.empty()) return;

        std::lock_guard<std::mutex> lock(mutex_); //is needed for simultaneous access to common resource


        for (int id :status->current_cluster.nodes)
        {   
            if(id>global_map.map[status->current_cluster.belong_to].nodes.size()) continue;

            gbeam2_interfaces::msg::Vertex node = global_map.map[status->current_cluster.belong_to].nodes[id];
            if(node.gain==0.0) continue;
            auto& pos = status->current_position.pose.pose.position;
            double dist = sqrt(pow(node.x-pos.x, 2) + pow(node.y-pos.y, 2));
            if (dist < node_dist_open)
            {
                node.gain = 0;
                node.is_visited = true;
                bool is_present = false;
                // RCLCPP_INFO(this->get_logger(), "I want to update the gain of node id: %d cluster: %d of robot%d",
                //     node.id,node.cluster_id,node.belong_to);
                if(node.belong_to==name_space_id){
                    global_map.map[name_space_id].nodes[node.id]=node;
                }else{
                     for(auto buff_node : graphBuffer[node.belong_to]->nodes){
                            if(buff_node.belong_to==node.belong_to && buff_node.id==node.id) is_present=true;
                        }
                        if(!is_present) graphBuffer[node.belong_to]->nodes.push_back(node);
                }
                
                
            }
        }

        merged_graph_pub_->publish(global_map);

        is_gain_updated=true;
    }

    void updateGlobalMap(const gbeam2_interfaces::msg::Graph graph_received){     
        // The input graph could also contains update of other graphs
        // That differs from re_robot_id  
        
        int req_robot_id = graph_received.robot_id;
        int last_updater_id = graph_received.last_updater_id;
        // The requiring robot is the one that is sending updates or this robot itself that want to update its own map.
        // In the case of the robot itself (namespace) i can have req_robot_id!=namespace_id
        //RCLCPP_INFO(this->get_logger(),"Receiving robot%d graph last updated by %d updates",req_robot_id,last_updater_id);
        auto& prev_graph = global_map.map[graph_received.robot_id];
        std::vector<int> last_added_node(N_robot, -1);  
        int nodes_added = 0;
        int nodes_explored = 0;
        int nodes_explored_by_other = 0;

        //RCLCPP_INFO(this->get_logger(), "Update global map with map of robot%d - BEFORE Last updated node: %d ", req_robot_id,last_update_node_with[req_robot_id]);
        //RCLCPP_INFO(this->get_logger(), "Size of received graph %d - Size of prev graph %d ", graph_received.nodes.size(),prev_graph.nodes.size());

        // Fixed things that i want to just be the same
        // Maybe cluster total gain could change here?

        prev_graph.cluster_graph = graph_received.cluster_graph;
        prev_graph.adj_matrix = graph_received.adj_matrix;
        prev_graph.length_matrix = graph_received.length_matrix;

        if(req_robot_id==name_space_id){
            for (int i = 0; i < N_robot; i++){

                // CLUSTERS GRAPH SHOULD BE UPDATED AS WELL OTHERWISE we will not receive any updates on that layer
                
                graphBuffer[i]->cluster_graph = graph_received.cluster_graph;
                graphBuffer[i]->adj_matrix = graph_received.adj_matrix;
                graphBuffer[i]->length_matrix = graph_received.length_matrix;

            }
        }
    
        // Update node for each graph update received
        // They can be mine (exploration gain update) among old ones
        // or old nodes with new exploration gain of req_robot_id
        // or new nodes of req_robot_id

        for(gbeam2_interfaces::msg::Vertex node: graph_received.nodes){
            auto& mod_graph = global_map.map[node.belong_to];
            bool add_to_buffer=false;
            
            if(node.id<mod_graph.nodes.size()){
                // RCLCPP_INFO(this->get_logger(), "OLD Node id: %d of robot%d updated by robot%d Last updated node: %d",node.id,node.belong_to,req_robot_id,last_update_node_with[node.belong_to]);
                auto incoming = node;

                // PROBLEMS HERE
                if (node.id >= mod_graph.nodes.size()) {
                    RCLCPP_ERROR(this->get_logger(), "Invalid access: node.id=%d but mod_graph.nodes.size()=%d", node.id, mod_graph.nodes.size());
                }

                auto actual = mod_graph.nodes[node.id];
                //update old ones with all the features (cluster_id, gain...)
                //RCLCPP_INFO(this->get_logger(), "OLD Node id: %d of robot%d OK",node.id,node.belong_to);

                if(req_robot_id==name_space_id){
                    // In this case I'm receiving update by myself (from graph update) 
                    // For sure the incoming gain will be greater because is never modified in graph_update
                    if(incoming.gain>actual.gain){
                        nodes_explored++;
                        mod_graph.cluster_graph.clusters[incoming.cluster_id].total_gain -= incoming.gain;
                         
                        auto& unexpl_nodes = mod_graph.cluster_graph.clusters[incoming.cluster_id].unexplored_nodes;
                       
                        unexpl_nodes.erase(std::remove(unexpl_nodes.begin(), unexpl_nodes.end(), incoming.id), unexpl_nodes.end());
                    } 
                    add_to_buffer |= (incoming.gain > actual.gain); 
                    add_to_buffer |= (incoming.cluster_id!=actual.cluster_id && incoming.cluster_id!=-1);
                    add_to_buffer |= (incoming.neighbors.size()>actual.neighbors.size());

                }else{
                    // If instead we're receiving external updates we could have two situation
                    if (node.belong_to==name_space_id){
                        // I'm receiving updates regarding my nodes that has been explored by someone else
                        if(incoming.gain<actual.gain){
                            nodes_explored_by_other++;
                            mod_graph.cluster_graph.clusters[incoming.cluster_id].total_gain -= incoming.gain;
                         
                            auto& unexpl_nodes = mod_graph.cluster_graph.clusters[incoming.cluster_id].unexplored_nodes;
                        
                            unexpl_nodes.erase(std::remove(unexpl_nodes.begin(), unexpl_nodes.end(), incoming.id), unexpl_nodes.end());
                        }
                    }else{
                        // I'm receving general updates by the req_robot
                    }                   

                }
                node.gain = (incoming.gain<actual.gain) ? incoming.gain : actual.gain;

                mod_graph.nodes[node.id]=node;                          
            }
            else{
                // adding new nodes
                nodes_added ++;
                //RCLCPP_INFO(this->get_logger(), "NEW Node id: %d of robot%d added",node.id,req_robot_id);
                //RCLCPP_INFO(this->get_logger(), "Add new node %d", node.id);
                mod_graph.nodes.push_back(node);
                if(req_robot_id==name_space_id) add_to_buffer = true;
                last_added_node[req_robot_id] = node.id;
                
                
                
            }

            if(add_to_buffer){
                for (int i = 0; i < N_robot; i++){
                    bool is_present = false; 
                    if (i!=name_space_id){
                        
                        for(auto buff_node : graphBuffer[i]->nodes){
                            if(buff_node.belong_to==node.belong_to && buff_node.id==node.id) is_present=true;
                        }
                        if(!is_present) graphBuffer[i]->nodes.push_back(node);
                    } 
                    
                    }
            }

            


        }

        //RCLCPP_INFO(this->get_logger(), "Update:: Add %d NEW nodes to the graph of robot%d", nodes_added, graph_received.robot_id);
        //RCLCPP_INFO(this->get_logger(), "Global:: Explored %d nodes in my graph by robot%d", nodes_explored_by_other, graph_received.robot_id);
        //RCLCPP_INFO(this->get_logger(), "Map   :: Explored %d nodes in my graph", nodes_explored);

        for (int i = 0; i < N_robot; i++){
            // if (i!=name_space_id) RCLCPP_INFO(this->get_logger(),"UGM:: BUFFER[%d] nodes_size: %d", i,graphBuffer[i]->nodes.size());
            last_update_node_with[i] = global_map.map[i].nodes.size();

        //RCLCPP_INFO(this->get_logger(), "Update global map with map of robot%d - AFTER Last updated node: %d ", i,last_update_node_with[i]);
            }
        
        global_map.last_updater = req_robot_id;
        
    }

    void serverCallback(const std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Request> request,
                        std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Response> response)
    {   
        if(start_merging){ //start_merging=true;

            int req_robot_id = request->update_request.robot_id;
            if(name_space_id==req_robot_id) return;
            std::unique_lock<std::mutex> lock(mutex_);
            //RCLCPP_INFO(this->get_logger(), "SERVER [%d]: Service call received from %d", name_space_id, req_robot_id);
            //RCLCPP_INFO(this->get_logger(), "SERVER [%d]:: I'm receiving %ld nodes from: %d", name_space_id, request->update_request.nodes.size(), request->update_request.robot_id);

            std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom"; //because lookupTransform doesn't allow "/" as first character
            std::string source_frame = "robot"+ std::to_string(req_robot_id) + "/odom"; 

           
            //RCLCPP_INFO(this->get_logger(),"PREPARE FEEDBACK:: received request size %d",request->update_request.nodes.size());
            
            // Reset flags and prepare for new update
            data_received_ = false;
            updateResponse.success = false;
            
            // Prepare the external feedback for graph_update_node
            gbeam2_interfaces::msg::GraphUpdate updates;
            updates.bridges = request->update_request.cluster_graph.bridges;
            updates.robot_id = req_robot_id;

            // TODO: is not sending anything

            // for(gbeam2_interfaces::msg::Vertex node: request->update_request.nodes){
            //     RCLCPP_INFO(this->get_logger(),"PREPARE FEEDBACK:: node id: %d belong to %d",node.id,node.belong_to);
            //     bool add_to_feedback=false;
            //     auto& mod_graph = global_map.map[node.belong_to];
            //     // avoid sending back my own node received by someone else
            //     if(node.belong_to!=name_space_id){
            //         if(node.id<last_update_node_with[node.belong_to]){
                   
            //             // Send to graph update only old ones with changed cluster
            //             if(node.cluster_id!=global_map.map[node.belong_to].nodes[node.id].cluster_id){
            //                 add_to_feedback =true;
            //             }
                            
                   
            //         }
            //         else{
            //             // adding new node                    
            //             add_to_feedback = true;
                         
            //         }
            //     }    
            //     if(add_to_feedback) updates.new_nodes.push_back(vert_transform(node,getTransform(target_frame,source_frame)));
                        
            // }

            // Update the global map with the information just received
            //RCLCPP_INFO(this->get_logger(),"UPDATE GLOBAL MAP IN SERVER CALLBACK - request->update_request");
            updateGlobalMap(request->update_request);

            updates.new_nodes=global_map.map[request->update_request.robot_id].nodes;
                    

            //RCLCPP_INFO(this->get_logger(), "SERVER[%d]-->I'm sending %ld nodes from: %d", name_space_id, updates.new_nodes.size(), req_robot_id);
            // publish the external feedback for graph_update_node
            external_updates_pub_->publish(updates);

            // Wait for the update to be processed with a timeout
            auto timeout = std::chrono::seconds(5); // Adjust the timeout as needed
            auto status = cv_.wait_for(lock, timeout, [this] { return data_received_; });

            if (status) {
                // Data from Graph update received within the timeout period
                //RCLCPP_INFO(this->get_logger(),"SERVER[%d]: UpdateGlobalMap with my graph processed considering also last nodes of %d",name_space_id, req_robot_id);
                //
                // MANCAVA UPDATE RESPONSE
                response->update_response = *graphBuffer[req_robot_id];
                
                response->success = updateResponse.success;
                //RCLCPP_INFO(this->get_logger(), "SERVER [%d]: Data received from %d has been processed by graph_update", name_space_id, req_robot_id);
            
            } else {
                // Timeout occurred
                response->success = false;
                RCLCPP_WARN(this->get_logger(), "SERVER [%d]: Timeout waiting %d for graph update",name_space_id, req_robot_id);
                return;
            }
            //this->timers_CLIENTS[req_robot_id]->reset(); //if(this->timers_CLIENTS[req_robot_id]->time_until_trigger()!=std::chrono::nanoseconds::max())

        }
        else{
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "SERVER [%d]: Merging is not available yet",name_space_id);
            return;
        }
        
    }

    void switchCallback(const gbeam2_interfaces::msg::Graph::SharedPtr graph)
    {   
        std::lock_guard<std::mutex> lock(mutex_); 

        //RCLCPP_INFO(this->get_logger(),"UPDATE GLOBAL MAP IN SWITCH CALLBACK - my own graph");
        updateGlobalMap(*graph);
        
        if (graph->last_updater_id != name_space_id) {
            
            // My own map has been computed considering also external nodes just received 
            
            
            updateResponse.success = true;
            data_received_ = true;
            cv_.notify_one();
            //RCLCPP_INFO(this->get_logger(), "SWITCH:: Processing update considering nodes from %d...", graph->last_updater_id);
        } else {

            /// My own map has been computed considering also external nodes just received 
            //RCLCPP_INFO(this->get_logger(), "SWITCH:: Publishing on my own topic... %s", (is_gain_updated) ? "with the gain updated" : "no gain updated");
            is_gain_updated=false;
        }

        
        merged_graph_pub_->publish(global_map);
    }

    void timeoutCallback(int timer_index){
        RCLCPP_INFO(this->get_logger(),"Expired time for robot%d ...", timer_index);        
        timers_CLIENTS[timer_index]->reset();//cancel();
    }

    void periodicTimerCallback(){
        if(start_merging){
            for(int i=0; i< timers_CLIENTS.size();i++){
            if(i!=name_space_id){
                auto trigger_time = timers_CLIENTS[i]->time_until_trigger();
                if(trigger_time!=std::chrono::nanoseconds::max()){
                auto lb_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(max_no_connection_time) - std::chrono::seconds(periodic_call_time)-std::chrono::milliseconds(250));
                auto up_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(max_no_connection_time) - std::chrono::seconds(periodic_call_time)+std::chrono::milliseconds(250));
                if(trigger_time> lb_time && trigger_time<up_time){
                    //send a request
                    

                    //RCLCPP_INFO(this->get_logger(),"CLIENT[%d]: Send a request to %ld with %d nodes ...",name_space_id,i,graphBuffer[i]->nodes.size());

                    std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Request> request_ = std::make_shared<gbeam2_interfaces::srv::GraphUpdate::Request>();

                    // PREPARE THE REQUEST 
                    request_->update_request = *graphBuffer[i];  

                    // Reset here the Buffer to prevent excluding from update 
                    // All the nodes added while waiting for response
                    graphBuffer[i]->nodes.clear();      

             
                    using ServiceResponseFuture =
                    rclcpp::Client<gbeam2_interfaces::srv::GraphUpdate>::SharedFutureWithRequest;

                    // Here is defined the future callback that would be executed once the response is received

                    auto response_received_callback =
                    [node = this](ServiceResponseFuture future) -> void {
                        auto request_response = future.get();
                        //RCLCPP_INFO(node->get_logger(), "CLIENT[%d]:: Update global Map with received update from %d", node->name_space_id,request_response.second->update_response.robot_id);
                        // Update the Global Map with the updates received
                        
                        //RCLCPP_INFO(node->get_logger(),"UPDATE GLOBAL MAP IN CLIENT CALLBACK - request_response.second->update_response");
                        node->updateGlobalMap(request_response.second->update_response);

                        // Prepare the external feedback for graph_update_node
                        gbeam2_interfaces::msg::GraphUpdate updates;
                        updates.bridges     = request_response.second->update_response.cluster_graph.bridges;
                        updates.robot_id    = request_response.second->update_response.robot_id;



                        // for(gbeam2_interfaces::msg::Vertex vert: request_response.second->update_response.nodes){
                        //     RCLCPP_INFO(node->get_logger(),"PREPARE FEEDBACK:: node id: %d belong to %d",vert.id,vert.belong_to);
                        //     bool add_to_feedback=false;
                        //     auto& mod_graph = node->global_map.map[vert.belong_to];
                        //     // avoid sending back my own vert received by someone else
                        //     if(vert.belong_to!=node->name_space_id){
                        //         if(vert.id<node->last_update_node_with[vert.belong_to]){
                            
                        //             // Send to graph update only old ones with changed cluster
                        //             if(vert.cluster_id!=node->global_map.map[vert.belong_to].nodes[vert.id].cluster_id){
                        //                 add_to_feedback =true;
                        //             }
                                        
                            
                        //         }
                        //         else{
                        //             // adding new vert                    
                        //             add_to_feedback = true;
                                    
                        //         }
                        //     }    
                        //     if(add_to_feedback) updates.new_nodes.push_back(vert);
                        //     //if(add_to_feedback) updates.new_nodes.push_back(vert_transform(vert,node->getTransform(target_frame,source_frame)));
                                    
                        // }

                        //RCLCPP_INFO(node->get_logger(), "CLIENT[%d]-->I'm sending %ld nodes from: %d", node->name_space_id, updates.new_nodes.size(),updates.robot_id);
                        // publish the external feedback for graph_update_node

                        updates.new_nodes=node->global_map.map[updates.robot_id].nodes;
                        node->external_updates_pub_->publish(updates);

                        // publish the so updated Global map
                        node->merged_graph_pub_->publish(node->global_map);
                
                    };

                    // SEND THE REQUEST 
                    auto result_future = graph_updates_CLIENTS[i]->async_send_request(request_, std::move(response_received_callback));

                    
                    std::future_status status = result_future.wait_for(5s);  // timeout to guarantee a graceful finish
                    if (status == std::future_status::ready) {
                        //RCLCPP_INFO(this->get_logger(), "CLIENT[%d]: Received response from %d",name_space_id,i);        
                        timers_CLIENTS[i]->reset();
                        //graphBuffer[i]->nodes.clear();
                    }

                    
                }
            }else{
                //timer is canceled
            }
            tracking_timers.data[i] = std::round(100.0f * std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::seconds(max_no_connection_time) - trigger_time).count())/ 100.0f;
            } 
            
            
            }
            
            timer_pub_->publish(tracking_timers); 
        }
        
        
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GraphMergerNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

