//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>



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
#include "gbeam2_interfaces/msg/ext_graph_update.hpp"

#include "gbeam2_interfaces/srv/set_mapping_status.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "library_fcn.hpp"

#define INF 100000
//-------------------------------------------------------------------------------------------------------------



class GraphUpdateNode : public rclcpp::Node
{
public:
    GraphUpdateNode() : Node("graph_update") 
    {
        name_space = this->get_namespace();
        name_space_id = name_space.back()- '0';
        graph.robot_id = name_space_id;
        graph.last_updater_id = name_space_id;
        
        RCLCPP_INFO(this->get_logger(), "namespace: %s ",name_space.c_str());
        RCLCPP_INFO(this->get_logger(), "namespace_id: %d",name_space_id);
        poly_sub_ = this->create_subscription<gbeam2_interfaces::msg::FreePolygonStamped>(
            "gbeam/free_polytope", 1, std::bind(&GraphUpdateNode::polyCallback, this, std::placeholders::_1));

        graph_pub_ =this->create_publisher<gbeam2_interfaces::msg::Graph>(
          "gbeam/reachability_graph",1);

        external_poly_sub_ = this->create_subscription<gbeam2_interfaces::msg::ExtGraphUpdate>(
            "external_nodes", 1, std::bind(&GraphUpdateNode::extNodesCallback, this, std::placeholders::_1));

        
        // SERVICE
        status_server_ = this->create_service<gbeam2_interfaces::srv::SetMappingStatus>(
           "gbeam/set_mapping_status", std::bind(&GraphUpdateNode::setStatus, this, std::placeholders::_1, std::placeholders::_2));

        //Initialize parameters
        this->declare_parameter<int>("N_robot",0);
        this->declare_parameter<double>("node_dist_min",0.0);
        this->declare_parameter<double>("node_dist_open", 0.0);
        this->declare_parameter<double>("node_bound_dist",0.0);
        this->declare_parameter<double>("obstacle_margin",0.0);
        this->declare_parameter<double>("safe_dist",0.0);
        
        //Exploration limits
        this->declare_parameter<double>("limit_xi",0.0);
        this->declare_parameter<double>("limit_xs",0.0);
        this->declare_parameter<double>("limit_yi",0.0);
        this->declare_parameter<double>("limit_ys",0.0);


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Get parameter from yaml file
        N_robot = this->get_parameter("N_robot").get_parameter_value().get<int>();
        node_dist_min = this->get_parameter("node_dist_min").get_parameter_value().get<double>();
        node_dist_open = this->get_parameter("node_dist_open").get_parameter_value().get<double>();
        node_bound_dist = this->get_parameter("node_bound_dist").get_parameter_value().get<double>();
        obstacle_margin = this->get_parameter("obstacle_margin").get_parameter_value().get<double>();
        safe_dist = this->get_parameter("safe_dist").get_parameter_value().get<double>();

        limit_xi = this->get_parameter("limit_xi").get_parameter_value().get<double>();
        limit_xs = this->get_parameter("limit_xs").get_parameter_value().get<double>();
        limit_yi = this->get_parameter("limit_yi").get_parameter_value().get<double>();
        limit_ys = this->get_parameter("limit_ys").get_parameter_value().get<double>();
      
        RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF GRAPH_UPDATE: ############# ");
        RCLCPP_INFO(this->get_logger(),"############# (for %s) ############# ",name_space.c_str());
        RCLCPP_INFO(this->get_logger(),"1) NODE_DIST_MIN: %f", node_dist_min);
        RCLCPP_INFO(this->get_logger(),"2) NODE_DIST_OPEN: %f", node_dist_open);
        RCLCPP_INFO(this->get_logger(),"3) NODE_BOUND_DIST: %f", node_bound_dist);
        RCLCPP_INFO(this->get_logger(),"4) OBSTACLE_MARGIN: %f", obstacle_margin);
        RCLCPP_INFO(this->get_logger(),"5) SAFE_DIST: %f", safe_dist);
        RCLCPP_INFO(this->get_logger(),"6) LIMIT_XI: %f", limit_xi);
        RCLCPP_INFO(this->get_logger(),"7) LIMIT_XS: %f", limit_xs);
        RCLCPP_INFO(this->get_logger(),"8) LIMIT_YI: %f", limit_yi);
        RCLCPP_INFO(this->get_logger(),"9) LIMIT_YS: %f", limit_ys);
        RCLCPP_INFO(this->get_logger(),"10) Number of robots: %d",N_robot);

        //Initialize 
        graph.adj_matrix.resize(N_robot);

    }    

    // Declaration of the setStatus function
    bool setStatus(
        const std::shared_ptr<gbeam2_interfaces::srv::SetMappingStatus::Request> request,
        std::shared_ptr<gbeam2_interfaces::srv::SetMappingStatus::Response> response)
    {
        RCLCPP_INFO(this->get_logger(),"setting status -------> done");
        // Assuming mapping_status is a member variable of your GraphUpdateNode class
        mapping_status = request->request;
        response->response = true;

        return true;
   }

private:
    bool mapping_status=false;
    double node_dist_min;
    double node_dist_open;
    double node_bound_dist;
    double obstacle_margin;
    double safe_dist;

    bool received_ext_nodes;

    
    double limit_xi, limit_xs, limit_yi, limit_ys;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string name_space;
    int name_space_id;
    int N_robot;

    gbeam2_interfaces::msg::FreePolygonStamped external_nodes;

    gbeam2_interfaces::msg::Graph graph;

    rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr graph_pub_;
    rclcpp::Subscription<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr poly_sub_;
    rclcpp::Subscription<gbeam2_interfaces::msg::ExtGraphUpdate>::SharedPtr external_poly_sub_;
    rclcpp::Service<gbeam2_interfaces::srv::SetMappingStatus>::SharedPtr status_server_;

    void extNodesCallback(const std::shared_ptr<gbeam2_interfaces::msg::ExtGraphUpdate> update){
        // Each time i receive external nodes I store them 
        external_nodes = update->polygon; 
        for(int z=0; z>graph.adj_matrix.size();z++){
            if(z!=name_space_id) GraphAdj2GraphAdjTranspose(update->adj_matrix[z]);
        }

        received_ext_nodes = true;
    }

    void polyCallback(const std::shared_ptr<gbeam2_interfaces::msg::FreePolygonStamped> poly_ptr)
    {
        if(!mapping_status)
            return;

        bool is_changed = false;

        geometry_msgs::msg::TransformStamped l2g_tf;
        //RCLCPP_INFO(this->get_logger(),"TransformStamped local to global tf -------> done");
        std::string target_frame = name_space.substr(1, name_space.length()-1) + "/odom"; //becasue lookupTransform doesn't allow "/" as first character
        std::string source_frame = poly_ptr->header.frame_id;
        try {
            
            //RCLCPP_INFO(this->get_logger(), "lookupTransform -------> %s to %s", target_frame.c_str(), source_frame.c_str());
            l2g_tf = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero); //poly_ptr->header.stamp we get tranformation in the future
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(
                this->get_logger(), "GBEAM:graph_update:lookupTransform: Could not transform %s to %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
            return;
        }
        
        //std::is_empty<gbeam2_interfaces::msg::FreePolygonStamped>
        if(received_ext_nodes){
            graph.last_updater_id = external_nodes.robot_id;
            is_changed = true;
            RCLCPP_INFO(this->get_logger(), "I received some nodes from %d!", external_nodes.robot_id);

        } else {
            graph.last_updater_id = name_space_id;
        }


        // ####################################################
        // ####### ---------- ADD GRAPH NODES --------- #######
        // ####################################################

        for (int i=0; i<poly_ptr->polygon.vertices_reachable.size(); i++)
        {
            //RCLCPP_INFO(this->get_logger(),"entrato nel primo for -------> ");
            gbeam2_interfaces::msg::Vertex vert = poly_ptr->polygon.vertices_reachable[i];  //get vertex from polytope
            vert.belong_to = name_space_id;
            vert = vert_transform(vert, l2g_tf); //change coordinates to global position

            float vert_dist = vert_graph_distance_noobstacle(graph, vert);
            // vert = applyBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys);

            /// ####### NEW PART FOR COMMUNICATION ############
            float vert_ext_dist; 
            if(received_ext_nodes && external_nodes.polygon.vertices_reachable.size()!=0){
                gbeam2_interfaces::msg::Graph fake_graph;
                fake_graph.nodes = external_nodes.polygon.vertices_reachable;
                vert_ext_dist = vert_graph_distance_noobstacle(fake_graph, vert);
            }
            else{
                vert_ext_dist = INF;
            }
            // ################################################

            if (vert_dist > node_dist_open && vert_ext_dist> node_dist_open) // modified also this condition
            {
            vert.id = graph.nodes.size();
            vert.is_reachable = true;
            vert.gain ++;
            if (!isInBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys))
            {
                vert.is_reachable = false;
                vert.gain = 0;
            }
            addNode(graph,vert,N_robot); //add vertex to the graph
            is_changed = true;
            }
            else if (vert_ext_dist< node_dist_open)
            {
                RCLCPP_INFO(this->get_logger(),"REACHABLE Vertex %d wasn't added due to conflict with external nodes",i);
            }
            
        }
        for (int i=0; i<poly_ptr->polygon.vertices_obstacles.size(); i++)
        {
            gbeam2_interfaces::msg::Vertex vert = poly_ptr->polygon.vertices_obstacles[i];  //get vertex from polytope
            vert.belong_to = name_space_id;
            vert = vert_transform(vert, l2g_tf); //change coordinates to global position

            vert = moveAway(vert, obstacle_margin);

            float vert_dist = vert_graph_distance_obstacle(graph, vert);

            /// ####### NEW PART FOR COMMUNICATION ############
            float vert_ext_dist; 
            if(received_ext_nodes && external_nodes.polygon.vertices_obstacles.size()!=0){
                gbeam2_interfaces::msg::Graph fake_graph;
                fake_graph.nodes = external_nodes.polygon.vertices_obstacles;
                vert_ext_dist = vert_graph_distance_noobstacle(fake_graph, vert);
            }
            else{
                vert_ext_dist = INF;
            }
            // #################################################
            if ((vert_dist > node_dist_min && vert_ext_dist> node_dist_min) && vert.is_obstacle)
            {
                
            vert.id = graph.nodes.size();
            vert.gain ++;
            if (!isInBoundary(vert, limit_xi, limit_xs, limit_yi, limit_ys))
            {
                vert.is_reachable = false;
                vert.gain = 0;
            }

            addNode(graph,vert, N_robot);         //add vertex to the graph
            is_changed = true;
            }
            else if (vert_ext_dist< node_dist_open)
            {
                RCLCPP_INFO(this->get_logger(),"OBSTACLE Vertex %d wasn't added due to conflict with external nodes",i);
            }
        }

        auto new_adj_matrix = GraphAdj2matrix(graph.adj_matrix[name_space_id]);

        // ####################################################
        // ####### ---------- ADD GRAPH EDGES --------- #######
        // ####################################################
        
        //compute polygon in global coordinates
        gbeam2_interfaces::msg::FreePolygon polyGlobal = poly_transform(poly_ptr->polygon, l2g_tf);

        //create vectors with indices of vertices inside polytopes
        std::vector<int> inReachableId;
        std::vector<int> inObstacleNotReachableId;
        for (int i=0; i<graph.nodes.size(); i++)
            if(isInsideObstacles(polyGlobal,graph.nodes[i])){
                if (isInsideReachable(polyGlobal, graph.nodes[i])){
                    inReachableId.push_back(i);
                }else{
                    if(graph.nodes[i].is_obstacle) inObstacleNotReachableId.push_back(i);
                }
            }
            

        
        for (int i=0; i<inReachableId.size(); i++)
        { 
            for (int j=i+1; j<inReachableId.size(); j++)
            {
            if (new_adj_matrix[inReachableId[i]][inReachableId[j]] == -1) 
            {
                //then add edge i-j to graph
                gbeam2_interfaces::msg::Vertex& node_i = graph.nodes[inReachableId[i]];
                gbeam2_interfaces::msg::Vertex& node_j = graph.nodes[inReachableId[j]]; 
                gbeam2_interfaces::msg::GraphEdge edge = computeEdge(node_i, node_j, node_bound_dist);
                edge.id = graph.edges.size();
                if(isInsideReachable(polyGlobal, node_i) && isInsideReachable(polyGlobal, node_j))
                edge.is_walkable = true;  // if both vertices are inside reachable poly, then the edge is walkable
                graph.edges.push_back(edge);

                //update adjacency matrix
                new_adj_matrix[inReachableId[i]][inReachableId[j]] = edge.id;
                new_adj_matrix[inReachableId[j]][inReachableId[i]] = edge.id;
                node_i.neighbors.push_back(node_j.id);
                node_j.neighbors.push_back(node_i.id);

                is_changed = true;
            }
            else  // if edge is present, check if it is walkable
            {
                //RCLCPP_INFO(this->get_logger()," -------> entra nell'else del secondo if (ADD GRAPH EDGES)");
                int e = new_adj_matrix[inReachableId[i]][inReachableId[j]];
                // DAVA SEGMENTATION FAULT QUI PERCHÈ LA MATRICE È TUTTA A 0 E NON -1
                if(isInsideReachable(polyGlobal, graph.nodes[graph.edges[e].v1]) && isInsideReachable(polyGlobal, graph.nodes[graph.edges[e].v2]))
                graph.edges[e].is_walkable = true;
            }
            }
        }

        for (int i=0; i<inObstacleNotReachableId.size(); i++)
        {
           for (int j=i+1; j<inObstacleNotReachableId.size(); j++)
            {
               if (new_adj_matrix[inObstacleNotReachableId[i]][inObstacleNotReachableId[j]] == -1) 
                { 
                    //then add edge i-j to graph
                    gbeam2_interfaces::msg::Vertex& obs_i = graph.nodes[inObstacleNotReachableId[i]];
                    gbeam2_interfaces::msg::Vertex& obs_j = graph.nodes[inObstacleNotReachableId[j]]; 
                    gbeam2_interfaces::msg::GraphEdge edge = computeEdge(obs_i, obs_j, node_bound_dist);
                    edge.id = graph.edges.size();
                    if(edge.is_boundary && obs_i.neighbors.size()<2 && obs_j.neighbors.size()<2){

                        graph.edges.push_back(edge);

                        //update adjacency matrix
                        new_adj_matrix[inObstacleNotReachableId[i]][inObstacleNotReachableId[j]] = edge.id;
                        new_adj_matrix[inObstacleNotReachableId[j]][inObstacleNotReachableId[i]] = edge.id;
                        obs_i.neighbors.push_back(obs_j.id);
                        obs_j.neighbors.push_back(obs_i.id);

                        is_changed = true;
                    } 

                }
            }
        }
        

        graph.adj_matrix[name_space_id]=matrix2GraphAdj(new_adj_matrix);

        // ####################################################
        // ####### --- UPDATE CONNECTIONS AND GAINS --- #######
        // #################################################### 

        if(is_changed)
        {
            for(int n=0; n<graph.nodes.size(); n++)
            {
            if(graph.nodes[n].is_obstacle && !graph.nodes[n].is_completely_connected)
            {
                bool connected_left = false, connected_right = false;
                for(int e : new_adj_matrix[n]){
                    if(e!= -1 && graph.edges[e].is_boundary)
                {
                    // compute angular coefficient of the line containing normal: y=mx
                    float m = graph.nodes[n].obstacle_normal.y/graph.nodes[n].obstacle_normal.x;
                    // compute value of the inequality mx-y>0, evaluated for edge direction
                    float value = m * graph.edges[e].direction.x - graph.edges[e].direction.y;
                    if(graph.edges[e].v2 == n)
                    value = -value;
                    if(value>0)
                    connected_right = true;
                    else
                    connected_left = true;
                }

                }
                /*
                for(int e=0; e<graph.edges.size(); e++)
                {
                if(graph.edges[e].is_boundary && ((graph.edges[e].v1 == n) || (graph.edges[e].v2 == n)))
                {
                    // compute angular coefficient of the line containing normal: y=mx
                    float m = graph.nodes[n].obstacle_normal.y/graph.nodes[n].obstacle_normal.x;
                    // compute value of the inequality mx-y>0, evaluated for edge direction
                    float value = m * graph.edges[e].direction.x - graph.edges[e].direction.y;
                    if(graph.edges[e].v2 == n)
                    value = -value;
                    if(value>0)
                    connected_right = true;
                    else
                    connected_left = true;
                }
                }*/


                if (connected_left && connected_right)
                graph.nodes[n].is_completely_connected = true;
                is_changed = true;
                graph.nodes[n].gain = 0;
            }
            }
        }

        // update exploration gain
        gbeam2_interfaces::msg::Vertex position;
        position = vert_transform(position, l2g_tf);  // create temporary vertex at robot position
        for (int i=0; i<graph.nodes.size(); i++)
        {
            if (dist(position, graph.nodes[i]) < node_dist_open)
            {
            graph.nodes[i].gain = 0;
            graph.nodes[i].is_visited = true;
            is_changed = true;
            }
        }

        // publish graph if some change has occurred
        if(is_changed)
            graph_pub_->publish(graph);
            
        if(is_changed && received_ext_nodes)   received_ext_nodes = false;
    
        //end of polyCallback
    }
        
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphUpdateNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

