#include <cmath>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/exceptions.h>
#include <boost/array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <common.h>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


#define sign(A)  ( (A) >= 0 ? 1.0: -1.0 )
#define _USE_MATH_DEFINES

namespace automatic_parking {
    class autodock_controller :public rclcpp::Node{
        public:
            autodock_controller():Node("autodock_controller"){
                
                this->declare_parameter<double>("cmd_vel_angular_rate", 0.5);
                this->get_parameter("cmd_vel_angular_rate", cmd_vel_angular_rate );
                this->declare_parameter<double>("cmd_vel_linear_rate", 0.5);
                this->get_parameter("cmd_vel_linear_rate" , cmd_vel_linear_rate );
                this->declare_parameter<double>("approach_angle" , 0.1);
                this->get_parameter("approach_angle" , approach_angle );
                this->declare_parameter<int>("lost_tag_max" ,5);
                this->get_parameter("lost_tag_max" , lost_tag_max);
                this->declare_parameter<double>("default_turn", 1.0);
                this->get_parameter("default_turn" , default_turn );
                this->declare_parameter<int>("max_center_count", 10);
                this->get_parameter("max_center_count" , max_center_count );
                this->declare_parameter<double>("final_approach_distance" ,1.0);
                this->get_parameter("final_approach_distance" , final_approach_distance);
                this->declare_parameter<double>("jog_distance" ,0.2);
                this->get_parameter("jog_distance" , jog_distance);
                this->declare_parameter<double>("finish_distance" ,0.3);
                this->get_parameter("finish_distance" , finish_distance);
                this->declare_parameter<double>("mini_turn_period" , 0.18);
                this->get_parameter("mini_turn_period" , mini_turn_period);
                this->declare_parameter<double>("tune_angle" ,0.42);
                this->get_parameter("tune_angle" , tune_angle);

                vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 20);
                buffer_dock2bot = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                buffer_odom = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                buffer_bot2dock = std::make_unique<tf2_ros::Buffer>(this->get_clock());

                //tf_listener_dock = std::make_unique<tf2_ros::TransformListener>(*buffer_dock);
                //tf_listener_odom = std::make_unique<tf2_ros::TransformListener>(*buffer_odom);
            }
            void run();
            void set_docking_state(std::string new_docking_state);

        private:
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
            std::unique_ptr<tf2_ros::Buffer> buffer_dock2bot;
            std::unique_ptr<tf2_ros::Buffer> buffer_odom;
            std::unique_ptr<tf2_ros::Buffer> buffer_bot2dock;

            geometry_msgs::msg::TransformStamped tf_dock2bot , tf_odom ,tf_bot2dock;

            void tags_callback();
            void set_action_state(std::string new_action_state);
            void fid2pos();
            void docking_state_manage();
            void blind_state_fun();
            void searching_state_fun();
            void centering_state_fun();
            void approach_state_fun();
            void final_approach_state_fun();
            void openrover_forward(double distance);
            void openrover_stop();
            void openrover_turn(double radians);
            void receive_tf();
            void action_state_manage();
            std::string action_state , docking_state , last_docking_state , last_action_state;
            bool in_view;
            int tag_callback_counter , centering_counter , max_center_count , lost_tag_max , final_counter;
            double cmd_vel_angular_rate, cmd_vel_linear_rate, approach_angle, default_turn, final_approach_distance, jog_distance, finish_distance,mini_turn_period;
            struct tag_pose{
                double theta;
                double distance ;
                double theta_bounds;
            };
            tag_pose pose_set;
            geometry_msgs::msg::Twist cmd_vel_msg;
            boost::array<double,3> robot_point_temp , robot_point;
            double temp_theta , temp_distance , tag_x, tag_y , tag_yaw, desire_angle,tune_angle;
            double last_sign = 1;

        
    };
}