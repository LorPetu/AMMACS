
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

#include <math.h>
#include <chrono>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"



#define PI 3.14159265359
// #define T_PI 2.0*PI          not used

class PositionController : public rclcpp::Node
{
public:
    PositionController(): Node("pos_contr") 
    {
        name_space = this->get_namespace();
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            name_space+"/scan", 1, std::bind(&PositionController::scanCallback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            name_space+ "/odom", 1, std::bind(&PositionController::odomCallback, this, std::placeholders::_1));

        ref_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            name_space+"/gbeam/gbeam_pos_ref", 1, std::bind(&PositionController::refPosCallback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

        start_moving_service_ = this->create_service<std_srvs::srv::SetBool>(
            "gbeam/start_moving",std::bind(&PositionController::startMoving,this, std::placeholders::_1, std::placeholders::_2));


        //******************** GET PARAMETERS **************************
        this->declare_parameter<float>("k_rho", 0.0);
        this->declare_parameter<float>("k_alpha", 0.0);
        this->declare_parameter<float>("alpha_thr", 0.0);
        this->declare_parameter<float>("rho_thr", 0.0);
        this->declare_parameter<float>("yaw_max", 0.0);
        this->declare_parameter<float>("vel_max", 0.0);

        
        k_rho = this->get_parameter("k_rho").get_parameter_value().get<double>();
        k_alpha = this->get_parameter("k_alpha").get_parameter_value().get<double>();
        alpha_thr = this->get_parameter("alpha_thr").get_parameter_value().get<double>();
        yaw_max = this->get_parameter("yaw_max").get_parameter_value().get<double>();
        vel_max = this->get_parameter("vel_max").get_parameter_value().get<double>();
        rho_thr = this->get_parameter("rho_thr").get_parameter_value().get<double>();
        


        RCLCPP_INFO(this->get_logger(),"################# PARAMETERS OF DDRIVE CONTROLLER ############");
        RCLCPP_INFO(this->get_logger(),"1) k_rho: %f", k_rho);
        RCLCPP_INFO(this->get_logger(),"2) k_alpha: %f", k_alpha);
        RCLCPP_INFO(this->get_logger(),"3) alpha_thr: %f", alpha_thr);
        RCLCPP_INFO(this->get_logger(),"4) yaw_max: %f", yaw_max);
        RCLCPP_INFO(this->get_logger(),"5) vel_max: %f", vel_max);
        RCLCPP_INFO(this->get_logger(),"6) rho_thr: %f", rho_thr);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PositionController::controlLoop, this));
    }

private:
   float k_rho, k_alpha, alpha_thr, yaw_max, vel_max, rho_thr;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pos_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    std::string name_space;
    bool start_moving = true;

    sensor_msgs::msg::LaserScan scan_;
    nav_msgs::msg::Odometry robot_odom_;
    geometry_msgs::msg::PoseStamped target_pos_;
    bool pos_ref_received = false;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_moving_service_;

    void startMoving(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response){
        start_moving=request->data;
        if(start_moving){
            response->success = true;
            response->message = "Start exploring for " + name_space;
        }else{
            response->success = false;
            response->message = "Stop exploring for" + name_space;

        }      

    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_ptr)
    {
        scan_ = *scan_ptr;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
    {
        robot_odom_ = *odom_ptr;
        //RCLCPP_INFO(this->get_logger(),"odom received: x:%f y:%f",robot_odom_.pose.pose.position.x,robot_odom_.pose.pose.position.y);
    }

    void refPosCallback(const geometry_msgs::msg::PoseStamped::SharedPtr tar_pos_ptr)
    {
        //RCLCPP_INFO(this->get_logger(), "refCallback executed. Point received x: %f y: %f",tar_pos_ptr->pose.position.x, tar_pos_ptr->pose.position.x);
        target_pos_ = *tar_pos_ptr;
        pos_ref_received=true;

    }

    float limit(float v, float max, float min)
    {
        return (v < min) ? min : ((v > max) ? max : v);
    }

    void controlLoop()
    {
        if(!start_moving) return;
        geometry_msgs::msg::Twist cmd_vel;

        if(pos_ref_received){
            float delta_x = target_pos_.pose.position.x - robot_odom_.pose.pose.position.x;
        float delta_y = target_pos_.pose.position.y - robot_odom_.pose.pose.position.y;
        //RCLCPP_INFO(this->get_logger(),"Target node position: delta_x:%f delta_y:%f",delta_x,delta_y);
        float yaw_part_y = 2 * (robot_odom_.pose.pose.orientation.w * robot_odom_.pose.pose.orientation.z + robot_odom_.pose.pose.orientation.x * robot_odom_.pose.pose.orientation.y);
        float yaw_part_x = 1 - 2 * (robot_odom_.pose.pose.orientation.y * robot_odom_.pose.pose.orientation.y + robot_odom_.pose.pose.orientation.z * robot_odom_.pose.pose.orientation.z);
        float yaw = atan2(yaw_part_y, yaw_part_x); // yaw of the robot

        //RCLCPP_INFO(this->get_logger(),"Target node position: x:%f y:%f",target_pos_.pose.position.x,target_pos_.pose.position.y);

        float rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2)); // distance from the target
        //RCLCPP_INFO(this->get_logger(),"Distance from the target node: x:%f",rho);
        float dest_ang = atan2(delta_y, delta_x); // direction of target wrt pos
        float alpha = atan2(sin(dest_ang - yaw), cos(dest_ang - yaw)); // angular difference from yaw to target direction
        //RCLCPP_INFO_EXPRESSION(this->get_logger(),rho<rho_thr,"Distance from the target node: x:%f << than %f",rho, rho_thr);
        if (rho < rho_thr)
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
        }
        else
        {
            if (alpha < alpha_thr && alpha > -alpha_thr)
            {
                //RCLCPP_INFO(this->get_logger(),"Linear and angular velocity command");
                cmd_vel.linear.x = limit(k_rho * rho * cos(alpha), vel_max, 0);
                cmd_vel.angular.z = limit(k_alpha * alpha, yaw_max, -yaw_max);
            }
            else
            {
                //RCLCPP_INFO(this->get_logger(),"ONLY angular velocity command");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = limit(k_alpha * alpha, yaw_max, -yaw_max);
            }
        }

        cmd_vel_publisher_->publish(cmd_vel);
    }
        }

        




};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionController>());
    rclcpp::shutdown();
    return 0;
}
