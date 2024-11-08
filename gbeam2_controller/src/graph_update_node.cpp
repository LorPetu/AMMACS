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
#include <limits>



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

#include "gbeam2_interfaces/msg/graph_cluster_node.hpp"
#include "gbeam2_interfaces/msg/graph_cluster.hpp"
#include "gbeam2_interfaces/srv/set_mapping_status.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp> // For easier point cloud population

#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "library_fcn.hpp"

#define INF 100000
//-------------------------------------------------------------------------------------------------------------

struct V_info
{
    int node_id;
    double coeff;
    int cluster_id;
};



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

        clusters_pub_ =this->create_publisher<gbeam2_interfaces::msg::GraphCluster>(
          "gbeam/clusters",1);

        external_poly_sub_ = this->create_subscription<gbeam2_interfaces::msg::FreePolygonStamped>(
            "external_nodes", 1, std::bind(&GraphUpdateNode::extNodesCallback, this, std::placeholders::_1));

        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "merged_obstacles",1);

        
        // SERVICE
        status_server_ = this->create_service<gbeam2_interfaces::srv::SetMappingStatus>(
           "gbeam/set_mapping_status", std::bind(&GraphUpdateNode::setStatus, this, std::placeholders::_1, std::placeholders::_2));

        //Initialize parameters
        this->declare_parameter<double>("node_dist_min",0.0);
        this->declare_parameter<double>("node_dist_open", 0.0);
        this->declare_parameter<double>("node_bound_dist",0.0);
        this->declare_parameter<double>("obstacle_margin",0.0);
        this->declare_parameter<double>("safe_dist",0.0);
        this->declare_parameter<double>("max_lenght_edge",0.0);
        
        //Exploration limits
        this->declare_parameter<double>("limit_xi",0.0);
        this->declare_parameter<double>("limit_xs",0.0);
        this->declare_parameter<double>("limit_yi",0.0);
        this->declare_parameter<double>("limit_ys",0.0);


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Get parameter from yaml file
        node_dist_min = this->get_parameter("node_dist_min").get_parameter_value().get<double>();
        node_dist_open = this->get_parameter("node_dist_open").get_parameter_value().get<double>();
        node_bound_dist = this->get_parameter("node_bound_dist").get_parameter_value().get<double>();
        obstacle_margin = this->get_parameter("obstacle_margin").get_parameter_value().get<double>();
        safe_dist = this->get_parameter("safe_dist").get_parameter_value().get<double>();
        max_lenght_edge = this->get_parameter("max_lenght_edge").get_parameter_value().get<double>();

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

    }    

    // Declaration of the setStatus function
    bool setStatus(
        const std::shared_ptr<gbeam2_interfaces::srv::SetMappingStatus::Request> request,
        std::shared_ptr<gbeam2_interfaces::srv::SetMappingStatus::Response> response)
    {
        //RCLCPP_INFO(this->get_logger(),"setting status -------> done");
        // Assuming mapping_status is a member variable of your GraphUpdateNode class
        mapping_status = request->request;
        response->response = true;

        return true;
   }

private:
    bool mapping_status;
    double node_dist_min;
    double node_dist_open;
    double node_bound_dist;
    double obstacle_margin;
    double safe_dist;
    double max_lenght_edge;

    bool received_ext_nodes;

    
    double limit_xi, limit_xs, limit_yi, limit_ys;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string name_space;
    int name_space_id;

    gbeam2_interfaces::msg::FreePolygonStamped external_nodes;

    gbeam2_interfaces::msg::Graph graph;
    std::vector<gbeam2_interfaces::msg::Vertex> new_reach_node;
    int E_new=0; //new added edges
    int V_new=0; //new just added edges
    int cluster_state=0;
    std::vector<gbeam2_interfaces::msg::Vertex> first_batch;
    std::vector<gbeam2_interfaces::msg::Vertex> second_batch;
    double gamma_1 = std::numeric_limits<double>::quiet_NaN();
    double gamma_2 = std::numeric_limits<double>::quiet_NaN();
    int V_1;
    int V_2;
    int E_1;
    int E_2;
    std::vector<int> first_batch_ids;
    std::vector<int> second_batch_ids;
    bool start_batch=false;

    gbeam2_interfaces::msg::GraphCluster clusters;
    std::vector<V_info> unclustered_nodes;

    rclcpp::Publisher<gbeam2_interfaces::msg::Graph>::SharedPtr graph_pub_;
    rclcpp::Publisher<gbeam2_interfaces::msg::GraphCluster>::SharedPtr clusters_pub_;
    rclcpp::Subscription<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr poly_sub_;
    rclcpp::Subscription<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr external_poly_sub_;
    rclcpp::Service<gbeam2_interfaces::srv::SetMappingStatus>::SharedPtr status_server_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    void addToCluster(gbeam2_interfaces::msg::Vertex){

    }

    gbeam2_interfaces::msg::GraphClusterNode createCluster(std::vector<gbeam2_interfaces::msg::Vertex> nodes){
        gbeam2_interfaces::msg::GraphClusterNode new_cl;
        new_cl.cluster_id = clusters.clusters.size();
        for(auto& node: nodes){
            new_cl.nodes.push_back(node.id);
        }

        return new_cl;

    }

    double computeClusterCoeff(gbeam2_interfaces::msg::Vertex& node, std::vector<int>& batch_nodes_ids, gbeam2_interfaces::msg::GraphClusterNode& cluster, std::vector<std::vector<float>>& adj_matrix){
        double coeff=-1.0;
        int E=0;
        std::vector<int> neighbours; // Vector to store the common elements

        std::sort(node.neighbors.begin(), node.neighbors.end());
        std::sort(batch_nodes_ids.begin(), batch_nodes_ids.end());
        std::sort(cluster.nodes.begin(), cluster.nodes.end());


        
        // Get the intersection of both arrays
        std::set_intersection(node.neighbors.begin(), node.neighbors.end(),
                                batch_nodes_ids.begin(), batch_nodes_ids.end(),
                                std::back_inserter(neighbours));

        std::set_intersection(cluster.nodes.begin(), cluster.nodes.end(),
                                neighbours.begin(), neighbours.end(),
                                std::back_inserter(neighbours));
        int V = neighbours.size();

        for (int i:neighbours)
            {
                for (int j:neighbours)
                {
                    if(adj_matrix[i][j]!=-1) E++;
                }
                
            }
        if(V>1) coeff = 2.0*E/(V*(V-1));

        return coeff;
    }
    

    void extNodesCallback(const std::shared_ptr<gbeam2_interfaces::msg::FreePolygonStamped> received_nodes){
        // Each time i receive external nodes I store them 
        external_nodes = *received_nodes;
        received_ext_nodes = true;
    }

    void polyCallback(const std::shared_ptr<gbeam2_interfaces::msg::FreePolygonStamped> poly_ptr)
    {
        if(!mapping_status)
            return;

        bool is_changed = false;
        new_reach_node.clear();
        // Compute density before adding new nodes
        int E =graph.edges.size();
        int V = graph.nodes.size();
        double tot_density_prev = 2.0*E/(V*(V-1));

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
            addNode(graph,vert); //add vertex to the graph
            if(vert.is_reachable) new_reach_node.push_back(vert);
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

            addNode(graph,vert);         //add vertex to the graph
            if(vert.is_reachable) new_reach_node.push_back(vert);
            is_changed = true;
            }
            else if (vert_ext_dist< node_dist_open)
            {
                RCLCPP_INFO(this->get_logger(),"OBSTACLE Vertex %d wasn't added due to conflict with external nodes",i);
            }
        }

        auto new_adj_matrix = GraphAdj2matrix(graph.adj_matrix);

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
                if(!node_i.is_obstacle && !node_j.is_obstacle && edge.length < max_lenght_edge){
                    if(isInsideReachable(polyGlobal, node_i) && isInsideReachable(polyGlobal, node_j))
                    edge.is_walkable = true;  // if both vertices are inside reachable poly, then the edge is walkable
                    graph.edges.push_back(edge);

                    //update adjacency matrix
                    new_adj_matrix[inReachableId[i]][inReachableId[j]] = edge.id;
                    new_adj_matrix[inReachableId[j]][inReachableId[i]] = edge.id;
                    graph.nodes[inReachableId[i]].neighbors.push_back(node_j.id);
                    graph.nodes[inReachableId[j]].neighbors.push_back(node_i.id);

                    is_changed = true;

                }
                
            }
            else  // if edge is present, check if it is walkable
            {
                //RCLCPP_INFO(this->get_logger()," -------> entra nell'else del secondo if (ADD GRAPH EDGES)");
                int e = new_adj_matrix[inReachableId[i]][inReachableId[j]];
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
                    if(edge.is_boundary){ //&& obs_i.is_completely_connected && obs_j.is_completely_connected

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
        

        graph.adj_matrix=matrix2GraphAdj(new_adj_matrix);

        // ####################################################
        // ############### --- CLUSTERING --- #################
        // ####################################################

        // For new added nodes
        // - Populate first batch according to gamma index that increase until a treshold
        // - Start populate second batch according to gamma index and then stop when same trashold is reached
        // This trigger the clustering evaluation:
        // We have three entities: existing cluster, first batch, second batch.
        // We assume that the second batch compose a new cluster and we focus mainly on assign the nodes "in the middle"
        // or to an existing cluster or otherwhise to the second batch cluster

        V = graph.nodes.size();
        E = graph.edges.size();

        double tot_density_curr;
        V_info node_info;
        double gamma_min = 0.3;

        switch (cluster_state){

            case 0:
                for(auto& new_node:new_reach_node){
                    first_batch.push_back(new_node);
                    first_batch_ids.push_back(new_node.id);
                    node_info.node_id=new_node.id;
                    node_info.coeff = -1.0;
                    node_info.cluster_id =-1;
                    unclustered_nodes.push_back(node_info);
                }
                E_1=0;
                // Count how many edges are between nodes of this batch
                for (int i = 0; i < first_batch.size(); i++)
                {
                    for (int j = first_batch[i].id +1; j < V; j++)
                    {
                        if(new_adj_matrix[first_batch[i].id][j]!=-1) E_1++;
                    }
                    
                }
                V_1 = first_batch.size();
                if (V_1>1)
                {
                    tot_density_curr = 2.0*E_1/(V_1*(V_1-1)); // Current Edge density of the FIRST batch

                    RCLCPP_INFO(this->get_logger(), "Case: %d || current_density: %f V: %d E: %d  || delta: %f",cluster_state,tot_density_curr,V_1,E_1,abs(tot_density_curr - gamma_1)/gamma_1);
                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Case: %d || I can't compute current_density V: %d E: %d ",cluster_state,V_1,E_1);
                    break;
                }

                if(tot_density_curr!=0.0 && std::isnan(gamma_1)){
                    gamma_1 = tot_density_curr; 
                    // If i have already enough edge density, i have already a revelant batch
                    if(gamma_1>gamma_min) cluster_state=1;
                } else{
                    if(gamma_1!=0 && abs(tot_density_curr - gamma_1)/gamma_1>gamma_min) cluster_state=1;
                }    


                break;

            case 1:
                // Add new nodes to the batch
                for(auto& new_node:new_reach_node){
                    second_batch.push_back(new_node);
                    second_batch_ids.push_back(new_node.id);
                    node_info.node_id=new_node.id;
                    node_info.coeff = -1.0;
                    node_info.cluster_id =-1;
                    unclustered_nodes.push_back(node_info);
                }
                E_2=0;
                // Count how many edges are between nodes of this batch
                for (int i = 0; i < second_batch.size(); i++)
                {
                    for (int j = second_batch[i].id +1; j < V; j++)
                    {
                        if(new_adj_matrix[second_batch[i].id][j]!=-1) E_2++;
                    }
                    
                }
                V_2 = second_batch.size();

                if (V_2>1)
                {
                    tot_density_curr = 2.0*E_2/(V_2*(V_2-1)); // Current Edge density of the SECOND batch

                    RCLCPP_INFO(this->get_logger(), "Case: %d ||current_density %f V: %d E: %d || delta: %f",cluster_state,tot_density_curr,V_2,E_2,abs(tot_density_curr - gamma_2)/gamma_2);

                }
                else{
                    RCLCPP_INFO(this->get_logger(), "Case: %d ||I can't compute current_density V: %d E: %d ",cluster_state,V_2,E_2);
                    break;
                }

                if(tot_density_curr!=0.0 && std::isnan(gamma_2)){
                    gamma_2 = tot_density_curr; 
                    // If i have already enough edge density, i have already a revelant batch
                    if(gamma_2>gamma_min) cluster_state=1;
                } else{
                    if(gamma_2!=0 && abs(tot_density_curr - gamma_2)/gamma_2>gamma_min) cluster_state=2;
                } 
                
                break;

            default:
                break;
        }
            

        RCLCPP_INFO(this->get_logger(), "Case: %d ||gamma_1: %f gamma_2 %f",cluster_state,gamma_1,gamma_2);

        // Evaluate delta between density and switch logic

      

        if(cluster_state==1 && clusters.clusters.empty()){
            // If I haven't create any cluster yet first batch is the first cluster

            clusters.clusters.push_back(createCluster(first_batch));
            RCLCPP_INFO(this->get_logger(), " ###### Created the FIRST new cluster! ######");

            //RESET
            tot_density_curr = 0.0;
            first_batch.clear();
            first_batch_ids.clear();
            second_batch.clear();
            second_batch_ids.clear();
            cluster_state=0;
            gamma_1 = std::numeric_limits<double>::quiet_NaN();
            gamma_2 = std::numeric_limits<double>::quiet_NaN();
        }

        if(cluster_state==2){ // Evaluate clustering coefficient and create new cluster
            
            auto cluster_temp = createCluster(second_batch);
            clusters.clusters.push_back(cluster_temp);

            for(int i=unclustered_nodes.size();i>=0;i--)
            {   
                for (auto& cluster:clusters.clusters)
                {
                    if(i>first_batch.size()){ // Processing only node of the second batch that could forms a new cluster
                        double temp=computeClusterCoeff(graph.nodes[unclustered_nodes[i].node_id],second_batch_ids,cluster,new_adj_matrix);
                        if(temp > unclustered_nodes[i].coeff){
                            unclustered_nodes[i].coeff=temp;
                            unclustered_nodes[i].cluster_id = cluster.cluster_id;
                            
                        }  
                    }
                    else{ 
                        
                        double temp=computeClusterCoeff(graph.nodes[unclustered_nodes[i].node_id],first_batch_ids,cluster,new_adj_matrix);
                        if(temp > unclustered_nodes[i].coeff){
                            unclustered_nodes[i].coeff=temp;
                            unclustered_nodes[i].cluster_id = cluster.cluster_id;
                        }   

                    }

                                  
                }
                // After evaluate all the cluster coeff, i need to eventually reassign the ones in the second batch cluster

                if (i > first_batch.size() && unclustered_nodes[i].cluster_id != cluster_temp.cluster_id) {
                    // If the node of the second batch is re-assigned, remove it from the previous cluster if present
                    auto& nodes = cluster_temp.nodes;

                    // Find the node in the list of nodes
                    auto it = std::find(nodes.begin(), nodes.end(), unclustered_nodes[i].node_id);

                    // If the node ID is found, erase it
                    if (it != nodes.end()) {
                        nodes.erase(it);
                    }
                }

                           
            }
            
            

            RCLCPP_INFO(this->get_logger(), "Case: %d || Compute clusters and reset",cluster_state);
            tot_density_curr = 0.0;
            first_batch.clear();
            first_batch_ids.clear();
            second_batch.clear();
            second_batch_ids.clear();
            cluster_state=0;
            gamma_1 = std::numeric_limits<double>::quiet_NaN();
            gamma_2 = std::numeric_limits<double>::quiet_NaN();

        }

        // DEBUG CLOUDPOINT 
        // Prepare the PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();  // Set timestamp
        cloud_msg.header.frame_id = "world";     // Set frame ID (adjust if necessary)

        // Reserve space for the points and the additional "side" field
        cloud_msg.height = 1;                  // Unordered point cloud (1D array)
        cloud_msg.is_dense = false;            // Allow for possible invalid points
        int total_points = first_batch.size() + second_batch.size();
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
        for (const auto& node : first_batch) {

            //auto node = stored_Graph[name_space_id]->nodes[id];
            *iter_x = node.x;
            *iter_y = node.y;
            *iter_z = node.z;
            *iter_comp = 0; 
            
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_comp;

            i++;
        }
        for (const auto& node : second_batch) {

            //auto node = stored_Graph[name_space_id]->nodes[id];
            *iter_x = node.x;
            *iter_y = node.y;
            *iter_z = node.z;
            *iter_comp = 1;//unclustered_nodes[i].cluster_id; 
            
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_comp;

            i++;
        }
        // Process obstacles and reachables

        // Publish the point cloud
        point_cloud_publisher_->publish(cloud_msg);

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

