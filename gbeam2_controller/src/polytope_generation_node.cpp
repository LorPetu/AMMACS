//----------------------------------- INCLUDE ----------------------------------------------------------------
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

// da inserire nella cartella include 
#include "gbeam2_interfaces/msg/free_polygon.hpp"
#include "gbeam2_interfaces/msg/free_polygon_stamped.hpp"
#include "gbeam2_interfaces/msg/vertex.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "library_fcn.hpp"
//-------------------------------------------------------------------------------------------------------------



//------------------------------------------ CLASS -------------------------------------------------------------
class PolyGenNode : public rclcpp::Node
{
public:
  PolyGenNode() : Node("poly_gen")
  {
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",
     1, std::bind(&PolyGenNode::scanCallback, this, std::placeholders::_1)); //potremmo cambiarlo da std::placeholders::_1 --> _1

    free_poly_publisher_ = this->create_publisher<gbeam2_interfaces::msg::FreePolygonStamped>(
        "gbeam/free_polytope", 1);

    test_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(
    "gbeam/test", 1);

    name_space = this->get_namespace();
    name_space_id = name_space.back()- '0';

    // Initialize parameters
    // this->declare_parameter("rate", update_freq_);
    // this->declare_parameter("/gbeam_controller/polytope_generation_param/num_vertices", num_vertices_);
    // this->declare_parameter("/gbeam_controller/polytope_generation_param/distance_step", dist_step);
    // this->declare_parameter("/gbeam_controller/polytope_generation_param/start_distance", start_dist_);
    // this->declare_parameter("/gbeam_controller/polytope_generation_param/vertex_obstacle_dist", obstacle_d_thr_);
    // this->declare_parameter("/gbeam_controller/robot_param/safe_dist", safe_dist_);


    // Initialize parameters
    this->declare_parameter<int>("num_vertices",0);
    this->declare_parameter<double>("dist_step",0.0);
    this->declare_parameter<double>("start_dist",0.0);
    this->declare_parameter<double>("obstacle_d_thr",0.0);
    this->declare_parameter<double>("safe_dist",0.0);
    this->declare_parameter<double>("update_freq",2.0);
    this->declare_parameter<float>("obst_dist_min", 0.0);
    

    // Get parameter from yaml file
    num_vertices = this->get_parameter("num_vertices").get_parameter_value().get<int>();    //Cambiato qua 
    dist_step = this->get_parameter("dist_step").get_parameter_value().get<double>();
    start_dist = this->get_parameter("start_dist").get_parameter_value().get<double>();
    obstacle_d_thr = this->get_parameter("obstacle_d_thr").get_parameter_value().get<double>();
    safe_dist = this->get_parameter("safe_dist").get_parameter_value().get<double>();
    update_freq = this->get_parameter("update_freq").get_parameter_value().get<double>();
    obst_dist_min = this->get_parameter("obst_dist_min").get_parameter_value().get<double>();

    RCLCPP_INFO(this->get_logger(),"############# PARAMETERS OF POLYTOPE_GEN: ############# ");
    RCLCPP_INFO(this->get_logger(),"1) NUM_VERTICES: %d", num_vertices);
    RCLCPP_INFO(this->get_logger(),"2) DIST_STEP: %f", dist_step);
    RCLCPP_INFO(this->get_logger(),"3) START_DIST: %f", start_dist);
    RCLCPP_INFO(this->get_logger(),"4) OBSTACLE_D_THR: %f", obstacle_d_thr);
    RCLCPP_INFO(this->get_logger(),"5) SAFE_DIST: %f", safe_dist);
    RCLCPP_INFO(this->get_logger(),"6) UPDATE_FREQ: %f", update_freq);
    RCLCPP_INFO(this->get_logger(),"7) OBST_DIST_MIN: %f", obst_dist_min);


    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0f / update_freq)),
        std::bind(&PolyGenNode::generatePoly, this));
  }

private:
  int num_vertices;
  double dist_step;
  double safe_dist;
  double obstacle_d_thr;
  double start_dist ;
  double update_freq ;
  float obst_dist_min; 
  double offset_angle=0;

  std::string name_space;
  int name_space_id;

  sensor_msgs::msg::LaserScan::SharedPtr scan;

  rclcpp::Publisher<gbeam2_interfaces::msg::FreePolygonStamped>::SharedPtr free_poly_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr test_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr)
  {
    scan = scan_ptr;        //forse qua è scan senza _
  }

  void updateParam(){
    num_vertices = this->get_parameter("num_vertices").get_parameter_value().get<int>();    //Cambiato qua 
    dist_step = this->get_parameter("dist_step").get_parameter_value().get<double>();
    start_dist = this->get_parameter("start_dist").get_parameter_value().get<double>();
    obstacle_d_thr = this->get_parameter("obstacle_d_thr").get_parameter_value().get<double>();
    safe_dist = this->get_parameter("safe_dist").get_parameter_value().get<double>();
    update_freq = this->get_parameter("update_freq").get_parameter_value().get<double>();
    obst_dist_min = this->get_parameter("obst_dist_min").get_parameter_value().get<double>();
  }

  void generatePoly()
  {
  
    if (scan == nullptr)
      return;

    updateParam();
    int num_measurements = scan->ranges.size();

    if (num_measurements > 0)
    {
      
      
      // num_vertices_ = this->get_parameter("/gbeam_controller/polytope_generation_param/num_vertices").as_int();
      // dist_step = this->get_parameter("/gbeam_controller/polytope_generation_param/distance_step").as_double();
      // start_dist_ = this->get_parameter("/gbeam_controller/polytope_generation_param/start_distance").as_double();
      // obstacle_d_thr_ = this->get_parameter("/gbeam_controller/polytope_generation_param/vertex_obstacle_dist").as_double();
      // safe_dist_ = this->get_parameter("/gbeam_controller/robot_param/safe_dist").as_double();

      sensor_msgs::msg::PointCloud obstacles;
      obstacles.header = scan->header;

      //---------------------- Initialization of a vector of obstacle points ----------------------------
      
      for(int i = 0; i < num_measurements; i++) {
          float a = scan->angle_min + i * scan->angle_increment;
          float range = scan->ranges[i];

            if ((range >= scan->range_max) || (range < start_dist))
                range = scan->range_max;
            else {
                geometry_msgs::msg::Point32 p;
                p.x = range * cos(a);
                p.y = range * sin(a);
                p.z = 0;
                obstacles.points.push_back(p);

            }

      }

      test_publisher_->publish(obstacles);
      //-------------------------------------------------------------------------------------------------

      //---------------------- Initialization of polygon vertices directions ----------------------------
      
      geometry_msgs::msg::Vector3 vert_directions[num_vertices];
      float angle_diff_vert = 2*M_PI / num_vertices;

        for(int v=0; v<num_vertices; v++)
          {
            float vert_angle = v*angle_diff_vert + angle_diff_vert/2;// + offset_angle;
            vert_directions[v].x = cos(vert_angle);
            vert_directions[v].y = sin(vert_angle);
            vert_directions[v].z = 0;
          }
        
        //offset_angle+=10;
        //if(offset_angle==360) offset_angle=0;
      //-------------------------------------------------------------------------------------------------


      //------------------------------- Initialization of polygon ---------------------------------------
      geometry_msgs::msg::PolygonStamped poly;

      poly.header = scan->header;
      bool move_vertex[num_vertices];
        for (int v = 0; v < num_vertices; v++) {
            move_vertex[v] = true;
            geometry_msgs::msg::Point32 p;

            p = movePoint(p, vert_directions[v], start_dist);
            poly.polygon.points.push_back(p);
        }
      //-------------------------------------------------------------------------------------------------


      //----------------- Increase polygon until reaching obstacle or max_distance  
       int num_stopped = 0;
        int iter = 0, max_iter = (scan->range_max - start_dist) / dist_step;
        while (num_stopped < num_vertices && iter++ < max_iter) {
            for (int v = 0; v < num_vertices; v++) {
                if (move_vertex[v]) {
                    poly.polygon.points[v] = movePoint(poly.polygon.points[v], vert_directions[v], dist_step);
                    
                    bool containsObstacles = countInsideConv(poly.polygon, obstacles) > 0;
                  

                    if (containsObstacles || !isConv(poly.polygon)) { //inverted condition
                        poly.polygon.points[v] = movePoint(poly.polygon.points[v], vert_directions[v], -dist_step);
                        move_vertex[v] = false;
                        num_stopped++;
                    }
                    
                }


            }

          
        }
        
      //------------------------------------------------------------------------------------------------


      //---------------------------- Initialize free poly ----------------------------------------------
          
        gbeam2_interfaces::msg::FreePolygonStamped free_poly;
        free_poly.header = poly.header;
        free_poly.robot_id = name_space_id;

        sensor_msgs::msg::ChannelFloat32 expGainChan, obstacleChan;
        expGainChan.name = "exp_gain";
        obstacleChan.name = "is_obstacle";
      //-------------------------------------------------------------------------------------------------



      //--------------------- Compute exploration gain, copy values to free_poly, vertices --------------
      
      for(int v=0; v<num_vertices; v++)
      {
        gbeam2_interfaces::msg::Vertex vert_obs;   //vertex against obstacle
        vert_obs.x = poly.polygon.points[v].x;
        vert_obs.y = poly.polygon.points[v].y;
        vert_obs.z = poly.polygon.points[v].z;
        float exp_gain = 0;
        int vert_obstacles = num_measurements/num_vertices;
        for (int o=(v*vert_obstacles); o<((v+1)*vert_obstacles); o++)
        {
          if (scan->ranges[o] >= scan->range_max)
          exp_gain++;
          float step = abs(scan->ranges[o]-scan->ranges[o+1]);
          float tmp = step>scan->range_max ? 0 : step / obst_dist_min;
          if( tmp>1){
            //ROS_INFO("vertex: %d   ## tmp value: %f", v, tmp);
            //ROS_INFO("Vertex position: x:%f  y:%f",vert_obs.x,vert_obs.y);
            //ROS_INFO("range_max: %f", scan->range_max);
          }
          

          exp_gain += tmp>1 ? tmp : 0;
        }
        vert_obs.gain = exp_gain;
        vert_obs.is_visited = false;
        vert_obs.is_reachable = false;

        if (countClose(poly.polygon.points[v], obstacles, obstacle_d_thr)>2)
        {
          vert_obs.is_obstacle = true;
          vert_obs.obstacle_normal = computeNormal(poly.polygon.points[v], obstacles, obstacle_d_thr);
        }
        else
        {
          vert_obs.is_obstacle = false;
        }

        free_poly.polygon.vertices_obstacles.push_back(vert_obs);
        }
      //---------------------------------------------------------------------------------------------------


      //---------------------------------- Create Reachable Polygon ---------------------------------------

        free_poly.polygon = createReachablePolygon(free_poly.polygon, safe_dist);
      //---------------------------------------------------------------------------------------------------


      // ------------------------ Publishing the generated polygon ----------------------------------------
      free_poly_publisher_->publish(free_poly);
      //---------------------------------------------------------------------------------------------------
      
      
    }
  }

 
};
//------------------------------------------- END CLASS ---------------------------------------------------------



//-------------------------------------------- MAIN -----------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PolyGenNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
//----------------------------------------- END MAIN ---------------------------------------------------------