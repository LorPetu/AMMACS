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
#include "gbeam2_interfaces/msg/ext_graph_update.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "gbeam2_interfaces/msg/status.hpp"
#include "gbeam2_interfaces/msg/frontier_stamped.hpp"

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

    // PUBLISHING TOPICS
    merged_graph_pub_ = this->create_publisher<gbeam2_interfaces::msg::Graph>(
                "gbeam/merged_graph", 1);
    fake_poly_pub_ = this->create_publisher<gbeam2_interfaces::msg::ExtGraphUpdate>(
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
    RCLCPP_INFO(this->get_logger(),"2) Wifi range: %f",wifi_range);
    // Initialize vectors with the correct size
    curr_updateBuffer.resize(N_robot);
    prev_updateBuffer.resize(N_robot);
    timers_CLIENTS.resize(N_robot);
    tracking_timers.data.resize(N_robot);

    graph_updates_CLIENTS.resize(N_robot);


    for (int i = 0; i < N_robot; ++i) {
        curr_updateBuffer[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
        curr_updateBuffer[i]->robot_id = name_space_id;
        curr_updateBuffer[i]->adj_matrix.resize(N_robot);
        prev_updateBuffer[i] = std::make_shared<gbeam2_interfaces::msg::Graph>();
        prev_updateBuffer[i]->robot_id = name_space_id;
        prev_updateBuffer[i]->adj_matrix.resize(N_robot);
        for(int z =0; z<N_robot;z++){
            curr_updateBuffer[i]->adj_matrix[z].data.reserve(1);
            prev_updateBuffer[i]->adj_matrix[z].data.reserve(1);            
        }
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

    // Parameters variables
    int N_robot;
    double wifi_range;
    int periodic_call_time;
    int max_no_connection_time;

    rclcpp::CallbackGroup::SharedPtr cb_group_1;        
    rclcpp::CallbackGroup::SharedPtr cb_group_2;
    rclcpp::CallbackGroup::SharedPtr timers_cb_group;

    rclcpp::Subscription<gbeam2_interfaces::msg::Graph>::SharedPtr graph_subscriber_;
    rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr merged_graph_pub_;
    rclcpp::Publisher<gbeam2_interfaces::msg::ExtGraphUpdate>::SharedPtr fake_poly_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr timer_pub_;
    rclcpp::Service<gbeam2_interfaces::srv::GraphUpdate>::SharedPtr  graph_updates_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_merger_service_;
    std::vector<rclcpp::Client<gbeam2_interfaces::srv::GraphUpdate>::SharedPtr> graph_updates_CLIENTS;

    rclcpp::TimerBase::SharedPtr timer_ptr_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_CLIENTS;
    std_msgs::msg::Float32MultiArray tracking_timers;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


    std::mutex mutex_;
    std::condition_variable cv_;
    bool data_received_ = false;
    gbeam2_interfaces::srv::GraphUpdate::Response updateResponse;
    gbeam2_interfaces::msg::ExtGraphUpdate external_updates;


    std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> curr_updateBuffer;
    std::vector<std::shared_ptr<gbeam2_interfaces::msg::Graph>> prev_updateBuffer;
    

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

    void serverCallback(const std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Request> request,
                        std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Response> response)
    {   
        if(start_merging){
            int req_robot_id = request->update_request.robot_id;
            if(name_space_id==req_robot_id) return;
            std::unique_lock<std::mutex> lock(mutex_);
            RCLCPP_INFO(this->get_logger(), "SERVER [%d]: Service call received from %d", name_space_id, req_robot_id);
            RCLCPP_INFO(this->get_logger(), "I'm receiving %ld nodes from: %d", request->update_request.nodes.size(), request->update_request.robot_id);

            std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom"; //becasue lookupTransform doesn't allow "/" as first character
            std::string source_frame = "robot"+ std::to_string(req_robot_id) + "/odom";

        

            // Prepare and publish the fake polygon
            auto [transformed_graph, fake_poly] = graph_transform_and_get_fakepoly(request->update_request,getTransform(target_frame,source_frame));
            fake_poly.robot_id = req_robot_id;
            fake_poly.header.frame_id = target_frame;

            RCLCPP_INFO(this->get_logger(), "-->I'm sending %ld nodes from: %d", fake_poly.polygon.vertices_reachable.size(), fake_poly.robot_id);
            
            // Reset flags and prepare for new update
            data_received_ = false;
            updateResponse.success = false;
            
            // Publish the fake polygon
            external_updates.polygon= fake_poly;
            external_updates.adj_matrix = request->update_request.adj_matrix;
            external_updates.robot_id = req_robot_id;
            RCLCPP_INFO(this->get_logger(),"Sending UPDATES to GraphUpdateNode from robot%d with matrix C_%d%d:: rows: %d columns:%d",req_robot_id,req_robot_id,name_space_id,external_updates.adj_matrix[name_space_id].row,external_updates.adj_matrix[name_space_id].col);
            RCLCPP_INFO(this->get_logger(),"Sending UPDATES to GraphUpdateNode from robot%d with matrix C_%d%d:: rows: %d columns:%d",req_robot_id,req_robot_id,req_robot_id,external_updates.adj_matrix[req_robot_id].row,external_updates.adj_matrix[req_robot_id].col);
            fake_poly_pub_->publish(external_updates);

            // Wait for the update to be processed with a timeout
            auto timeout = std::chrono::seconds(5); // Adjust the timeout as needed
            auto status = cv_.wait_for(lock, timeout, [this] { return data_received_; });

            if (status) {
                // Data received within the timeout period
                //RCLCPP_INFO(this->get_logger(),"Update response success: %d (false is:%d)",updateResponse.success, false);
                response->update_response = *curr_updateBuffer[req_robot_id];

                prev_updateBuffer[req_robot_id]=curr_updateBuffer[req_robot_id];
                response->success = updateResponse.success;
                RCLCPP_WARN(this->get_logger(), "SERVER [%d]: Data received from %d has been processed by graph_update", name_space_id, req_robot_id);
            
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
        
        for (int i = 0; i < N_robot; i++)
        {
            if (i!=name_space_id) {
                auto updated_graph = std::make_shared<gbeam2_interfaces::msg::Graph>(compareUpdates(graph, prev_updateBuffer[i]));
                curr_updateBuffer[graph->last_updater_id] = updated_graph;   
            }
        }
        
        
        if (graph->last_updater_id != name_space_id) {
            //RCLCPP_INFO(this->get_logger(), "MERGER:: last nodes updated: %d last_edge_updated: %d", prev_updateBuffer[last_updater_id]->nodes.size(), prev_updateBuffer[last_updater_id]->edges.size());
            updateResponse.success = true;
            data_received_ = true;
            cv_.notify_one();
            RCLCPP_INFO(this->get_logger(), "Processing update considering nodes from %d...", graph->last_updater_id);
        } else {
            // This is our own update, publish it
            //RCLCPP_INFO(this->get_logger(), "Publishing on my own topic...");
            merged_graph_pub_->publish(*graph);
        }
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

                    RCLCPP_INFO(this->get_logger(),"CLIENT[%d]: Send a request to %ld ...",name_space_id,i);

                    std::shared_ptr<gbeam2_interfaces::srv::GraphUpdate::Request> request_ = std::make_shared<gbeam2_interfaces::srv::GraphUpdate::Request>();

                    request_->update_request = *curr_updateBuffer[i];  
                    RCLCPP_INFO(this->get_logger(),"Sending REQUEST UPDATES to robot%d with matrix C_%d%d:: rows: %d columns:%d",i,name_space_id,i,curr_updateBuffer[i]->adj_matrix[i].row,curr_updateBuffer[i]->adj_matrix[i].col);                  
                    prev_updateBuffer[i]=curr_updateBuffer[i];

                    /*while (!graph_updates_CLIENTS[i]->wait_for_service(1s)) {
                    if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "CLIENT[%d]: Interrupted while waiting for the service. Exiting.", name_space_id);
                    return;
                    }
                    RCLCPP_INFO(this->get_logger(), "CLIENT[%d]: service not available, waiting again...", name_space_id);
                    }*/

                    using ServiceResponseFuture =
                    rclcpp::Client<gbeam2_interfaces::srv::GraphUpdate>::SharedFutureWithRequest;

                    auto response_received_callback =
                    [node = this](ServiceResponseFuture future) -> void {
                        auto request_response = future.get();
                        node->merged_graph_pub_->publish(request_response.second->update_response); 
                    };
                    
                    auto result_future = graph_updates_CLIENTS[i]->async_send_request(request_, std::move(response_received_callback));

                    
                    std::future_status status = result_future.wait_for(2s);  // timeout to guarantee a graceful finish
                    if (status == std::future_status::ready) {
                        RCLCPP_INFO(this->get_logger(), "CLIENT[%d]: Received response from %d",name_space_id,i);        
                        timers_CLIENTS[i]->reset();
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
        for(int z=0; z<current_graph->adj_matrix.size();z++){
            RCLCPP_INFO(this->get_logger(),"compareUpdates:: C_%d%d :: rows: %d columns: %d",current_graph->robot_id, z,current_graph->adj_matrix[z].row,current_graph->adj_matrix[z].col);
        }
        
        result.robot_id = current_graph->robot_id;


        return result;
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

