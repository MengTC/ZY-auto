#ifndef CONTROL_HANDLE_HPP
#define CONTROL_HANDLE_HPP

#include "control.hpp"

namespace ns_control {

class ControlHandle {

 public:
  // Constructor
  ControlHandle(ros::NodeHandle &nodeHandle);

  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();
  // void sendVisualization();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Subscriber finalWaypointsSubscriber_;
  ros::Subscriber vehicleDynamicStateSubscriber_;
  ros::Subscriber utmPoseSubscriber_;

  ros::Publisher controlCommandPublisher_;
  ros::Publisher lookaheadpointPublisher_;
  ros::Publisher nearestPointPublisher_;

  void finalWaypointsCallback(const autoware_msgs::Lane &msg);
  void vehicleDynamicStateCallback(const common_msgs::VehicleDynamicState &msg); 
  void utmPoseCallback(const nav_msgs::Odometry &msg);

  std::string final_waypoints_topic_name_;
  std::string vehicle_dynamic_state_topic_name_;
  std::string localization_utm_topic_name_;
  std::string lookahead_point_topic_name_;
  std::string nearest_point_topic_name_;

  std::string control_command_topic_name_;

  int node_rate_;
  int control_mode_;

  Para control_para_;
  Pid_para pid_para_;
  Pure_pursuit_para pp_para_;
  LQR_para lqr_para_;

  Control control_;

};
}

#endif //CONTROL_HANDLE_HPP
