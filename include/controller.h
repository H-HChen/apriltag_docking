#include <cmath>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <boost/array.hpp>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <common.h>
#define sign(A)  ( (A) >= 0 ? 1.0: -1.0 )

namespace automatic_parking {
    class autodock_controller {
        public:
            autodock_controller(ros::NodeHandle& nh){
                this->n = nh;
                vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 20);
                tags_sub = n.subscribe("/tag_detections", 1 , &autodock_controller::tags_callback,this);
                nh.param<double>("cmd_vel_angular_rate", cmd_vel_angular_rate , 0.5);
                nh.param<double>("cmd_vel_linear_rate" , cmd_vel_linear_rate , 0.5);
                nh.param<double>("approach_angle" , approach_angle , 0.1);
                nh.param<int>("lost_tag_max" , lost_tag_max ,5);
                nh.param<double>("default_turn" , default_turn , 1.0);
                nh.param<int>("max_center_count" , max_center_count ,30);
                nh.param<double>("final_approach_distance" , final_approach_distance ,1.0);
                nh.param<double>("jog_distance" , jog_distance ,0.15);
                nh.param<double>("finish_distance" , finish_distance ,0.3);
                nh.param<double>("mini_turn_period" , mini_turn_period , 0.18);
                nh.param<double>("tune_angle" , tune_angle ,0.42);
                //nh.param<double>();
            }
            void run();
            void set_docking_state(std::string new_docking_state);

        private:
            ros::NodeHandle n ;
            ros::Publisher vel_pub;
            ros::Subscriber tags_sub;
            tf::TransformListener listener;
            tf::StampedTransform tf_odom ,tf_tag;

            void tags_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr&);
            void set_action_state(std::string new_action_state);
            void fid2pos( geometry_msgs::PoseWithCovarianceStamped fid_tf );
            void docking_state_manage();
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
            int tag_callback_counter , centering_counter , max_center_count , lost_tag_max;
            geometry_msgs::PoseWithCovarianceStamped dock_tag_tf , last_dock_tag_tf;
            double cmd_vel_angular_rate, cmd_vel_linear_rate, approach_angle, default_turn, final_approach_distance, jog_distance, finish_distance,mini_turn_period;
            ros::Timer linear_timer , turn_timer;
            struct tag_pose{
                double theta;
                double distance ;
                double theta_bounds;
            };
            tag_pose pose_set;
            geometry_msgs::Twist cmd_vel_msg;
            boost::array<double,3> robot_point_temp , robot_point;
            double temp_theta , temp_distance , tag_x , desire_angle , tune_angle;
            double last_sign = 1;

        
    };
}