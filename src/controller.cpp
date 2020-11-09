#include "controller.h"

using namespace automatic_parking ;

void autodock_controller::tags_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& tags_array){
    if (action_state == "count_aruco_callbacks"){
        tag_callback_counter += 1;
    }
    else{
        tag_callback_counter = 0;
    }
    if (tags_array->detections.empty()){
        in_view = false;
    }
    else{
        //take first detection 
        geometry_msgs::PoseWithCovarianceStamped fid_tf = tags_array->detections[0].pose;
        last_dock_tag_tf = dock_tag_tf;
        dock_tag_tf = fid_tf;
        in_view = true;
        ROS_INFO("marker detected");
        if (docking_state=="searching"){
            openrover_stop();
            tag_callback_counter = 0;
            set_docking_state("centering");
        }
        fid2pos(dock_tag_tf);
    }
        
}

void autodock_controller::docking_state_manage(){
    ROS_INFO("%s | %s", docking_state.c_str(), last_docking_state.c_str());
    
    if (docking_state == "searching"){
        searching_state_fun();
    }
        
    if (docking_state == "centering"){
        centering_state_fun();
    }
        
    if (docking_state == "approach"){
        approach_state_fun();
    }

    if (docking_state == "final_approach"){
        final_approach_state_fun();
    }

    if (docking_state == "docked"){
        openrover_stop();
    }

    vel_pub.publish(cmd_vel_msg);
}

void autodock_controller::fid2pos( geometry_msgs::PoseWithCovarianceStamped fid_tf ){

    double x_trans = fid_tf.pose.pose.position.x;
    double y_trans = fid_tf.pose.pose.position.y;
    //y_trans = y_trans - 0.1;
    double theta = atan2(-y_trans, x_trans);
    double r = sqrt(pow(x_trans ,2) + pow(y_trans , 2));
    double theta_bounds;
    if (r > 3.0){
        theta_bounds = approach_angle;
    }

    else{
        theta_bounds = r/30.0;
    }
    ROS_INFO("Theta: %3.3f, r: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds);
    pose_set = {theta: theta+desire_angle, distance: r, theta_bounds: theta_bounds};
}

void autodock_controller::set_docking_state(std::string new_docking_state){
    if (docking_state != new_docking_state){
        last_docking_state = docking_state;
        docking_state = new_docking_state;
        ROS_INFO("new state: %s, last state: %s", docking_state.c_str(), last_docking_state.c_str());
    }
}

void autodock_controller::set_action_state(std::string new_action_state){
    if (action_state != new_action_state){
        last_action_state = action_state;
        action_state = new_action_state;
        ROS_INFO("new state: %s, last state: %s", action_state.c_str(), last_action_state.c_str());
    }
}

void autodock_controller::searching_state_fun(){
    ROS_INFO("searching tag count: %d", tag_callback_counter);
    centering_counter = 0;
    if (action_state=="turning"){
        return;
    }
    if (tag_callback_counter<lost_tag_max){
        set_action_state("count_aruco_callbacks");
    }
    else{
        tag_callback_counter = 0;
        set_action_state("");
        openrover_turn(default_turn);
    }
}

void autodock_controller::centering_state_fun(){

    if (action_state=="turning"){
        return;
    }
    if (tag_callback_counter < 1){
        centering_counter += 1;
        ROS_INFO("centering_counter:%d" , centering_counter);
        set_action_state("count_aruco_callbacks");
        return;
    }
    tag_callback_counter = 0;
    set_action_state("");
    if (centering_counter == 5){
        ROS_INFO("tuning:%f",default_turn*last_sign);
        openrover_turn(default_turn*sign(-tag_x));
    }
    if (centering_counter >= max_center_count){
        ROS_WARN("centering failed. reverting to last state: searching");
        tag_callback_counter = 0;
        set_action_state("");
        set_docking_state("searching");
        return;
    }
    if (in_view){
        if (fabs(pose_set.theta)>pose_set.theta_bounds){
            openrover_turn(pose_set.theta);
        }
        else{
            ROS_INFO("centered switching to approach state");
            set_docking_state("approach");
            openrover_stop();
        }
    }
}

void autodock_controller::approach_state_fun(){
    centering_counter = 0;
    if (in_view){
        if (fabs(pose_set.theta)>pose_set.theta_bounds){
            ROS_INFO("approach angle exceeded: %f", fabs(pose_set.theta));
            openrover_stop();
            set_docking_state("centering");
        }
    
        else{
            if (action_state == "jogging"){
                return;
            }
            if (fabs(pose_set.distance-finish_distance) < jog_distance){
                openrover_stop();
                openrover_forward(pose_set.distance - finish_distance);
                set_docking_state("final_approach");
            }
            else{
                openrover_forward(jog_distance);
            }
        }
    }
    else{
        openrover_stop();
        set_docking_state(last_docking_state);
    }
    
}

void autodock_controller::final_approach_state_fun(){
    if (in_view and (fabs(pose_set.distance)>finish_distance)){
        set_docking_state("approach");
        return;
    }
    if (action_state == "jogging"){
        return;
    }
    if (action_state == ""){
        openrover_stop();
        set_docking_state("docked");
        ROS_INFO("Finish Docking");
    }
}

void autodock_controller::openrover_forward(double distance){
    double cmd_vel_linear ;
    if (action_state == ""){
        set_action_state("jogging");
        
        if (distance > 0){
            ROS_INFO("Moving forward");
            cmd_vel_linear = cmd_vel_linear_rate;
        }
        else{
            ROS_INFO("Moving Backward");
            cmd_vel_linear = -cmd_vel_linear_rate;
        }

        if (fabs(pose_set.distance) < final_approach_distance){
            cmd_vel_linear = cmd_vel_linear/2;
        }
    }
    cmd_vel_msg.linear.x = cmd_vel_linear;
    robot_point_temp = robot_point;
    temp_distance = distance;
}

void autodock_controller::openrover_stop(){
    set_action_state("");
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.angular.z = 0;
    
}

void autodock_controller::openrover_turn(double radians){
    double cmd_vel_angular;
    if (action_state == ""){
        set_action_state("turning");
        if (radians>0){
            cmd_vel_angular = -cmd_vel_angular_rate ;
        }
        else{
            cmd_vel_angular = cmd_vel_angular_rate;
        }

        if (fabs(radians) < 0.1){
            cmd_vel_angular = cmd_vel_angular/2;
        }
    }
    cmd_vel_msg.angular.z = cmd_vel_angular;
    robot_point_temp = robot_point;
    temp_theta = radians;
}

void autodock_controller::action_state_manage(){
    if (action_state == "jogging"){
        if (distance(robot_point_temp,robot_point) >= temp_distance){
            openrover_stop();
        }
    }
    if (action_state == "turning"){
        if (fabs(robot_point_temp[2]-robot_point[2])>=temp_theta){
            openrover_stop();
        }
    }
}

void autodock_controller::receive_tf(){
    try{
        listener.waitForTransform("odom","base_link",ros::Time(0),ros::Duration(1.0));
        listener.lookupTransform("odom","base_link",ros::Time(0),tf_odom);
        listener.lookupTransform("tag_0","base_link",ros::Time(0),tf_tag);
        double tag_x = tf_tag.getOrigin().x();
        double odom_x = tf_odom.getOrigin().x();
        double odom_y = tf_odom.getOrigin().y();
        double odom_yaw = tf::getYaw(tf_odom.getRotation());
        robot_point = {odom_x , odom_y , odom_yaw};
        if (fabs(tag_x)<0.1){
            desire_angle = 0;
        } 
        else{
            desire_angle = tune_angle;
        }
        ROS_INFO("%f" , tag_x);
    }
    catch(tf2::TransformException &ex){
        ROS_ERROR("%s",ex.what());
    }
}

void autodock_controller::run(){
    receive_tf();
    action_state_manage();
    docking_state_manage();

}

int main(int argc, char** argv){
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle n;
    auto controller_node = std::make_shared<automatic_parking::autodock_controller>(n) ;
    ros::Rate rate(30.0);
    controller_node->set_docking_state("searching");
    while (ros::ok()){
        controller_node->run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}