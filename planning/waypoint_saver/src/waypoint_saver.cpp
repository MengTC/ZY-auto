#include <ros/ros.h>
#include "waypoint_saver.hpp"


namespace ns_waypoint_saver {
// Constructor
Wp_saver::Wp_saver(ros::NodeHandle &nh) : nh_(nh) {
  cur_pose.pose.pose.position.x = 0.0;
  recorded_pose.pose.pose.position.x = 0.0;
  // record_file.close();
};

Wp_saver::~Wp_saver(){
  record_file.close();
  ROS_INFO("Record file closed.");
}

// Getters

// Setters
void Wp_saver::setLocalization (nav_msgs::Odometry msg) {
  cur_pose = msg;
}
void Wp_saver::setParameters(Para msg){
  para = msg;
}

void Wp_saver::openRecordFile(){
  record_file.open(para.waypoint_filename, std::ios::out);
  std::string header;
  if(para.record_mode == 0){
    // record for path tracking
    header = "frame,time,x,y,heading,v_x,v_y,yaw_rate";
  }
  else{
    header = "frame,time,x,y,heading,v_x,v_y,yaw_rate,a_x,distance,pedal_acc,pedal_brake";
  }
  record_file << header << std::endl;
  frame = 0;
  ROS_INFO("[Waypoint Saver] Record file created.");
  open_file_flag = true;
}

void Wp_saver::runAlgorithm() {
  if (!open_file_flag){
      openRecordFile();
  }
  else{  
    if (para.record_mode == 0){
      if (distance_compute(cur_pose, recorded_pose) > para.min_record_distance){
        std::string output_x = std::to_string(recorded_pose.pose.pose.position.x);
        std::string output_dis = std::to_string(distance_compute(recorded_pose, cur_pose));
        ROS_INFO_STREAM("recorded position x is " + output_x);
        ROS_INFO_STREAM("distance is " + output_dis);
        recorded_pose = cur_pose;
        write2File(recorded_pose);
      }    
    }
    else{
      write2File(recorded_pose);
    }
  }
}

inline double Wp_saver::distance_compute(nav_msgs::Odometry msg1, nav_msgs::Odometry msg2) {

  double x_dist = msg2.pose.pose.position.x - msg1.pose.pose.position.x;
  double y_dist = msg2.pose.pose.position.y - msg1.pose.pose.position.y;

  double dist = std::sqrt(x_dist * x_dist + y_dist * y_dist);

  return dist;
}

void Wp_saver::write2File(nav_msgs::Odometry msg) {
  using namespace std;  

  string frame_s = to_string(frame);
  ros::Time timestamp = msg.header.stamp;
  string time = to_string(timestamp.toSec());
  string x = to_string(msg.pose.pose.position.x);
  string y = to_string(msg.pose.pose.position.y);
  
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  string heading = to_string(yaw);
  string v_x = to_string(msg.twist.twist.linear.x);
  string v_y = to_string(msg.twist.twist.linear.y);
  string yaw_rate = to_string(msg.twist.twist.angular.z);
  
  if (para.record_mode == 0){// record for path tracking 
    record_file << frame + "," + time + "," + x + "," + y + "," 
          + heading + "," + v_x  + "," + v_y  + "," + yaw_rate << endl;
  }
  else{ // record for virtual platoon test
    record_file << frame + "," + time + "," + x + "," + y + "," 
      + heading + "," + v_x  + "," + v_y  + "," + yaw_rate << endl;
  }

  }

}
