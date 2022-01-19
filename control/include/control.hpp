#ifndef CONTROL_HPP
#define CONTROL_HPP

#define LIMIT_STEERING_ANGLE 20
#define BOUND(a,MAX,min) {if ((a)>(MAX)){(a)=(MAX);} if((a)<(min)){(a)=(min);}}

#include "autoware_msgs/Lane.h"
#include "common_msgs/ChassisState.h"
#include "nav_msgs/Odometry.h"
#include "common_msgs/ChassisControl.h"
#include "common_msgs/ControlState.h"
#include "common_msgs/Trigger.h"
#include "common_msgs/VirtualVehicleState.h"

#include "std_msgs/String.h"
#include "pid.hpp"
#include "pure_pursuit.hpp"
#include "lqr_path_tracking.hpp"

namespace ns_control {

struct Para{
  bool longitudinal_control_switch;
  bool lateral_control_switch;
  int  longitudinal_mode; // 1:constant speed, 2: planned speed, 3: desired distance
  double desired_speed;
  double desired_distance;
  int lon_controller_id;
  int lat_controller_id;
};

class Control {

 public:
  // Constructor
  Control(ros::NodeHandle &nh);

  // Getters
  common_msgs::ChassisControl getChassisControlCommand();
  geometry_msgs::PointStamped getLookaheadPoint();
  geometry_msgs::PointStamped getNearestPoint();
  common_msgs::ControlState getControlState();
  common_msgs::Trigger getReplayTrigger();

  // Setters
  void setFinalWaypoints(const autoware_msgs::Lane &msg);
  void setVehicleDynamicState(const common_msgs::ChassisState &mgs);
  void setUtmPose(const nav_msgs::Odometry &msg);
  void setPidParameters(const Pid_para &msg);
  void setPurePursuitParameters(const Pure_pursuit_para &msg);
  void setLQRParameters(const LQR_para &msg);
  void setControlParameters(const Para &msg);
  void setVirtualVehicleState(const common_msgs::VirtualVehicleState &msg);

  // Methods
  void runAlgorithm();
  void initializeController();
  double stateUpdate();
  double latControlUpdate();
  double lonControlUpdate();

  bool finalWaypointsFlag = false;
  bool vehicleDynamicStateFlag = false;
  bool utmPoseFlag = false;
  bool virtualFlag = false;

 private:

  ros::NodeHandle &nh_;

  autoware_msgs::Lane final_waypoints;
  std::vector<autoware_msgs::Waypoint> current_waypoints;
  nav_msgs::Odometry utm_pose;
  geometry_msgs::Pose current_pose;
  common_msgs::ChassisState vehicle_dynamic_state;
  common_msgs::ChassisControl chassis_control_command;
  common_msgs::ControlState control_state;
  common_msgs::Trigger replay_trigger;
  common_msgs::VirtualVehicleState virtual_vehicle_state;

  PID pid_controller;
  Pure_pursuit pp_controller;
  LQRPathTracking lqr_controller;

  Pid_para pid_para;
  Pure_pursuit_para pp_para;
  LQR_para lqr_para;
  
  Para control_para;
  
  // methods

  double lookahead_distance;
  autoware_msgs::Waypoint nearest_waypoint;
  autoware_msgs::Waypoint lookahead_waypoint;
  geometry_msgs::PoseStamped nearest_ps;
  geometry_msgs::PoseStamped lookahead_ps;
  geometry_msgs::PointStamped nearest_point;
  geometry_msgs::PointStamped lookahead_point;

  // int nearest_waypoint_idx;
  // int lookahead_waypoint_idx;
  
  int findNearestWaypoint();
  int findLookAheadWaypoint(float lookAheadDistance);
};
}

#endif //CONTROL_HPP
