#include "controller.h"

using namespace automatic_parking ;


void autodock_controller::docking_state_manage(){
    RCLCPP_INFO(get_logger(),"%s | %s", docking_state.c_str(), last_docking_state.c_str());
    
    if (docking_state == "searching"){
        searching_state_fun();
    }

    if (docking_state == "blind"){
        blind_state_fun();
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

}

void autodock_controller::set_docking_state(std::string new_docking_state){
    if (docking_state != new_docking_state){
        last_docking_state = docking_state;
        docking_state = new_docking_state;
        RCLCPP_INFO(get_logger(),"new state: %s, last state: %s", docking_state.c_str(), last_docking_state.c_str());
    }
}

void autodock_controller::set_action_state(std::string new_action_state){
    if (action_state != new_action_state){
        last_action_state = action_state;
        action_state = new_action_state;
        RCLCPP_INFO(get_logger(),"new state: %s, last state: %s", action_state.c_str(), last_action_state.c_str());
    }
}

void autodock_controller::searching_state_fun(){
    RCLCPP_INFO(get_logger(),"searching tag count: %d", tag_callback_counter);
    centering_counter = 0;
    if (action_state== "turning"){
        return;
    }
    if (action_state == "jogging"){
        return;
    }
    if (tag_callback_counter<lost_tag_max){
        set_action_state("count_aruco_callbacks");
    }
    else{
        tag_callback_counter = 0;
        set_action_state("");
        openrover_turn(default_turn*sign(tag_y));
    }
}

void autodock_controller::blind_state_fun(){
    if (action_state=="turning"){
        return;
    }
    if (action_state == "jogging"){
        return;
    }
    if (in_view){
        openrover_stop();
        openrover_turn(0.7*sign(-tag_y));
    }
    else{
        openrover_stop();
        openrover_forward(fabs(tag_y/2));
        set_docking_state("searching");
        
    }

}

void autodock_controller::centering_state_fun(){

    if (action_state == "turning"){
        return;
    }
    if (action_state == "jogging"){
        return;
    }
    if (tag_callback_counter < 1){
        centering_counter += 1;
        RCLCPP_INFO(get_logger(),"centering_counter:%d" , centering_counter);
        set_action_state("count_aruco_callbacks");
        return;
    }
    tag_callback_counter = 0;
    set_action_state("");
    
    if (centering_counter >= max_center_count){
        RCLCPP_WARN(get_logger(),"centering failed. reverting to last state: searching");
        tag_callback_counter = 0;
        set_docking_state("searching");
        return;
    }
    if (in_view){
        if (fabs(pose_set.theta)>pose_set.theta_bounds){
            openrover_stop();
            openrover_turn(pose_set.theta);
        }
        else{
            //RCLCPP_INFO(get_logger(),"centered switching to approach state");
            openrover_stop();
            set_docking_state("approach");
            
        }
    }
}

void autodock_controller::approach_state_fun(){
    centering_counter = 0;
    if (in_view){

        if (action_state == "jogging"){
            return;
        }
        
        if (desire_angle == 0){
            if (fabs(pose_set.theta)>pose_set.theta_bounds){
                RCLCPP_INFO(get_logger(),"approach angle exceeded: %f", fabs(pose_set.theta));
                set_docking_state("centering");
            }
            else{
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
            openrover_forward(jog_distance);
        }

    }
    else{
        if (desire_angle == tune_angle){
            openrover_stop();
            //openrover_turn(default_turn*sign(tag_y)/3);
            set_docking_state("searching");
        }
    }
    
}

void autodock_controller::final_approach_state_fun(){
    if (action_state == "turning"){
        return;
    }
    if (action_state == "jogging"){
        return;
    }  

    if (in_view and fabs(pose_set.distance-finish_distance)>0.1){
        set_docking_state("approach");
        return;
    }

    if (M_PI-fabs(tag_yaw)>pose_set.theta_bounds){
        openrover_turn((M_PI-fabs(tag_yaw))*sign(-tag_yaw));
        return;
    }
    else{
        openrover_stop();
        set_docking_state("docked");
        RCLCPP_INFO(get_logger(),"Finish Docking");
    }
}

void autodock_controller::openrover_forward(double distance){
    double cmd_vel_linear ;
    if (action_state == ""){
        set_action_state("jogging");
        
        if (distance > 0){
            RCLCPP_INFO(get_logger(),"Moving forward");
            cmd_vel_linear = cmd_vel_linear_rate;
        }
        else{
            RCLCPP_INFO(get_logger(),"Moving Backward");
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
        if (distance(robot_point_temp,robot_point) >= fabs(temp_distance)){
            openrover_stop();
        }

    }
    if (action_state == "turning"){
        if (fabs(robot_point_temp[2]-robot_point[2])>= fabs(temp_theta)){
            openrover_stop();
        }
    }
    if (tag_y){
        if ((fabs(tag_y)<0.03) and (desire_angle == tune_angle)){
            openrover_stop();
            final_counter += 1;
            if (final_counter > 3){
                desire_angle = 0;
            }
        }
    }
    else{
       desire_angle = tune_angle;
    }
    
    vel_pub->publish(cmd_vel_msg);
}

void autodock_controller::tags_callback(){
    if (action_state == "count_aruco_callbacks"){
        tag_callback_counter += 1;
    }
    else{
        tag_callback_counter = 0;
    }

    in_view = true;
    RCLCPP_INFO(get_logger(),"marker detected");
    if (docking_state=="searching"){
        if(fabs(tag_y/tag_x) >= fabs(tag_x/2)){
            openrover_stop();
            tag_callback_counter = 0;
            set_docking_state("blind");
        }
        else{
        openrover_stop();
        tag_callback_counter = 0;
        set_docking_state("centering");
        }
    }   
}

void autodock_controller::fid2pos(){
    double x_trans = tf_bot2dock.transform.translation.x;
    double y_trans = tf_bot2dock.transform.translation.y;
    double theta_tf = tf2::getYaw(tf_bot2dock.transform.rotation);
    double theta = atan2(-y_trans, x_trans);
    RCLCPP_INFO(get_logger(),"theta_cal:%f , theta_tf:%f",theta ,theta_tf );
    double r = sqrt(pow(x_trans ,2) + pow(y_trans , 2));
    double theta_bounds;
    if (r > 3.0){
        theta_bounds = approach_angle;
    }

    else{
        theta_bounds = r/30.0;
    }
    RCLCPP_INFO(get_logger(),"Theta: %3.3f, r: %3.3f, theta_bounds: %3.3f", theta, r, theta_bounds);
    pose_set = {theta: theta-desire_angle*sign(tag_y), distance: r, theta_bounds: theta_bounds};
}

void autodock_controller::receive_tf(){
    try{
        tf_odom = buffer_odom->lookupTransform("odom","base_link",tf2::TimePoint(std::chrono::milliseconds(0)),tf2::Duration(std::chrono::seconds(0)));
        tf_dock2bot = buffer_dock2bot->lookupTransform("tag36h11:0","base_link",tf2::TimePoint(std::chrono::milliseconds(0)),tf2::Duration(std::chrono::seconds(0)));
        tf_bot2dock = buffer_bot2dock->lookupTransform("base_link","tag36h11:0",tf2::TimePoint(std::chrono::milliseconds(0)),tf2::Duration(std::chrono::seconds(0)));

        tag_x = tf_dock2bot.transform.translation.z;
        tag_y = tf_dock2bot.transform.translation.x;
        tag_yaw = tf2::getYaw(tf_dock2bot.transform.rotation);
        double odom_x = tf_odom.transform.translation.x;
        double odom_y = tf_odom.transform.translation.y; 
        double odom_yaw = tf2::getYaw(tf_odom.transform.rotation);
        robot_point = {odom_x , odom_y , odom_yaw};
        //RCLCPP_INFO(get_logger(),"%f",tag_y);
        tags_callback();
        fid2pos();
    }
    catch(tf2::TransformException &ex){
        RCLCPP_ERROR(get_logger(),"%s",ex.what());
        in_view = false;

    }
}

void autodock_controller::run(){
    receive_tf();
    docking_state_manage();
    action_state_manage();
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<automatic_parking::autodock_controller>() ;
    rclcpp::Rate rate(30.0);
    controller_node->set_docking_state("searching");
    while (rclcpp::ok()){
        controller_node->run();
        rclcpp::spin_some(controller_node);
        rate.sleep();
    }
    return 0;
}