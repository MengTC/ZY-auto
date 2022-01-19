#include <ros/ros.h>
#include "control.hpp"
#include <sstream>

namespace ns_control
{
  // Constructor
  Control::Control(ros::NodeHandle &nh) : nh_(nh),
                                          pid_controller(1.0, 0.0, 0.0),
                                          pp_controller(3.975)
                                          {};

  // Getters
  common_msgs::ChassisControl Control::getChassisControlCommand(){
    return chassis_control_command;
  }
  geometry_msgs::PointStamped Control::getLookaheadPoint() { 
    lookahead_point.header.frame_id = "world";
    lookahead_point.header.stamp = ros::Time::now();
    return lookahead_point; 
  }

  geometry_msgs::PointStamped Control::getNearestPoint(){
    nearest_point.header.frame_id = "world";
    nearest_point.header.stamp = ros::Time::now();
    return nearest_point;
  }

  common_msgs::ControlState Control::getControlState(){
    control_state.header.frame_id = "world";
    control_state.header.stamp = ros::Time::now();
  }

  common_msgs::Trigger Control::getReplayTrigger(){
    replay_trigger.trigger = true;
  }

  // Setters
  void Control::setFinalWaypoints(const autoware_msgs::Lane &msg){
    final_waypoints = msg;
    current_waypoints = final_waypoints.waypoints;
  }
  void Control::setVehicleDynamicState(const common_msgs::ChassisState &msg){
    vehicle_dynamic_state = msg;
    // ROS_INFO_STREAM("[Control]current velocity: " << vehicle_state.twist.linear.x);
  }
  void Control::setUtmPose(const nav_msgs::Odometry &msg){
    utm_pose = msg; 
    current_pose = utm_pose.pose.pose;
  }

  void Control::setPidParameters(const Pid_para &msg){
    pid_para = msg;
    pid_controller.kp = pid_para.kp;
    pid_controller.ki = pid_para.ki;
    pid_controller.kd = pid_para.kd;
  }
  void Control::setPurePursuitParameters(const Pure_pursuit_para &msg){
    pp_para = msg;
    if (pp_para.mode == "fixed"){
      lookahead_distance = pp_para.lookahead_distance;
    }
  }

  void Control::setLQRParameters(const LQR_para &msg){
    lqr_para = msg;
    lqr_controller.lqr_para_filename = lqr_para.para_filename;
    lqr_controller.readLQRParameters();
  }

  void Control::setControlParameters(const Para &msg){
    control_para = msg;
    
  }
  int Control::findNearestWaypoint(){
    int waypoints_size = current_waypoints.size();
    if (waypoints_size == 0){
      ROS_WARN("No waypoints in final_waypoints.");
      return -1;
    }

    // find nearest point
    int nearest_idx = 0;
    double nearest_distance = getPlaneDistance(current_waypoints.at(0).pose.pose.position,current_pose.position);
    for (int i = 0; i < waypoints_size; i++){
      // if search waypoint is the last
      if(i == (waypoints_size - 1)){
        ROS_INFO("search waypoint is the last");
        // break;
      }
      double dis = getPlaneDistance(current_waypoints.at(i).pose.pose.position,current_pose.position);
      if (dis < nearest_distance){
        nearest_idx = i;
        nearest_distance = dis;
      }
    }
    nearest_waypoint = final_waypoints.waypoints[nearest_idx];
    nearest_ps = nearest_waypoint.pose;
    nearest_point.point = nearest_ps.pose.position;
    return nearest_idx;
  }

  int Control::findLookAheadWaypoint(float lookAheadDistance){
    int waypoints_size = current_waypoints.size();
    int nearest_waypoint_idx = findNearestWaypoint();
    if (nearest_waypoint_idx < 0 | nearest_waypoint_idx == waypoints_size - 1){
      return -1;
    }
    
    // look for the next waypoint
    for (int j = nearest_waypoint_idx; j < waypoints_size; j++){
      // if search waypoint is the last
      if (j == (waypoints_size - 1)){
        ROS_INFO("search waypoints is the last");
      }
      // if there exists an effective waypoint
      if (getPlaneDistance(current_waypoints.at(j).pose.pose.position, current_pose.position) > lookAheadDistance){
          lookahead_waypoint = final_waypoints.waypoints[j];
          lookahead_ps = lookahead_waypoint.pose;
          lookahead_point.point = lookahead_ps.pose.position;
        return j;
      }
    }
  }

  double Control::latControlUpdate(){
    // State update
    double v_x = utm_pose.twist.twist.linear.x;
    double v_y = utm_pose.twist.twist.linear.y;

    double yaw_rate = utm_pose.twist.twist.angular.z; 
    yaw_rate = lateral_speed/ 3.89 / 0.55/ 180.0*M_PI;
    double curvature = 0;

    // calculate yaw 
    tf::Quaternion quat,near_quat;
    tf::quaternionMsgToTF(current_pose.orientation, quat);
    tf::quaternionMsgToTF(nearest_ps.pose.orientation, near_quat);
    double roll, pitch, near_yaw, cur_yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, near_yaw);
    tf::Matrix3x3(quat).getRPY(roll, pitch, cur_yaw);

    control_state.lateral_error = calcRelativeCoordinate(nearest_ps.pose.position, current_pose).y;
    control_state.heading_error = near_yaw - cur_yaw;

    if( heading_error > M_PI) heading_error = heading_error-2*M_PI;
    else if( heading_error < -M_PI) heading_error = heading_error+ 2*M_PI;

    if (pp_para.mode == "variable"){
        lookahead_distance = pp_para.lookahead_distance;
    }
    int lookahead_waypoint_idx = findLookAheadWaypoint(lookahead_distance);

    ROS_INFO_STREAM("[Control] lookahead waypoint idx: " << lookahead_waypoint_idx
                    << ", x: " << lookahead_ps.pose.position.x << ", y: " << lookahead_ps.pose.position.y);
    
    // calculate front wheel angle
    double front_wheel_angle;
    ROS_INFO_STREAM("lat control id: " << control_para.lat_controller_id);
    switch (control_para.lat_controller_id){
      case 1: // pure pursuit controller
        {
          front_wheel_angle = pp_controller.outputFrontWheelAngle(lookahead_ps.pose.position,current_pose);
          ROS_INFO_STREAM("Using pure pursuit controller, output: " << front_wheel_angle);  
        }
        
        break;

      case 2: {// lqr controller
        ROS_INFO("Using lqr controller.");
        geometry_msgs::PoseStamped ref_ps;
        ref_ps = lookahead_ps;
        double lateral_error = calcRelativeCoordinate(ref_ps.pose.position, current_pose).y;
        
        tf::Quaternion ref_quat;
        tf::quaternionMsgToTF(ref_ps.pose.orientation, ref_quat);
        double ref_yaw;
        tf::Matrix3x3(ref_quat).getRPY(roll, pitch, ref_yaw);
        double heading_error = ref_yaw - cur_yaw;
        double dot_lateral_error = v_y + v_x * heading_error;
        double dot_heading_error = -v_x * curvature + yaw_rate;
        ROS_INFO_STREAM("lateral error: " << lateral_error << ", dot_lateral_error: " << dot_lateral_error
                      <<"heading error: " << heading_error << ", dot_heading_error: " << dot_heading_error);
        double tmp[4] = {lateral_error,dot_lateral_error,heading_error,dot_heading_error};
        std::vector<double> current_state(tmp,tmp+4);
        front_wheel_angle = lqr_controller.outputFrontWheelAngle(v_x,current_state);
        }
        break;

      default:{
        front_wheel_angle = 0;
        ROS_WARN("Illegal controller id!");
      }
        
        break;
    }
    return front_wheel_angle;
  }

  double Control::lonControlUpdate(){
    
  }

  void Control::runAlgorithm(){

    ROS_DEBUG("[Control]In run() ... ");
    if (vehicleDynamicStateFlag && finalWaypointsFlag && utmPoseFlag){
      
      // Lateral control
      if (control_para.lateral_control_switch){
        // limit front wheel angle
        double front_wheel_angle = latControlUpdate();
        BOUND(front_wheel_angle,LIMIT_STEERING_ANGLE,-LIMIT_STEERING_ANGLE);
        // convert front wheel angle to steering wheel angle
        chassis_control_command.steer_angle = 
                - (24.1066 * front_wheel_angle + 4.8505);
        ROS_INFO_STREAM("[Contorl] chassis_control_command steer angle: " << chassis_control_command.steer_angle);
      }
      else{
        chassis_control_command.steer_angle = 0;
        ROS_INFO_STREAM("[Contorl] Lateral control disabled");
      }
     
      // Longitudinal Control
      if (control_para.longitudinal_control_switch){
        if (control_para.longitudinal_mode == 1){// constant speed control
          // chassis_control_command.linear_velocity = control_para.desired_speed;
          }
      }
      else{
        ROS_INFO_STREAM("[Contorl] Longitudinal control disabled");
      }
    }else{
      ROS_WARN("Waiting for final waypoints or vehicle state...");
    }

  }

}
