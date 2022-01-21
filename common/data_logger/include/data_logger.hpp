#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include "nav_msgs/Odometry.h"
#include "common_msgs/ChassisState.h"
#include "common_msgs/ChassisControl.h"
#include "common_msgs/Trigger.h"
#include "std_msgs/String.h"

#include "tf/transform_datatypes.h"
#include <sstream>
#include <string>
#include <fstream>
#include <math.h>
#include <stdio.h>

namespace ns_data_logger {
struct Para{
  std::string log_filename;
  bool use_trigger;
};
class DataLogger {
  

 public:
  // Constructor
  DataLogger(ros::NodeHandle &nh);

  // Setters
  void setLocalization(nav_msgs::Odometry msg);
  void setChassisState(common_msgs::ChassisState msg);
  void setControlCommand(common_msgs::ChassisControl msg);
  void setLogTrigger(common_msgs::Trigger msg);


  void setParameters(Para msg);

  void runAlgorithm(); 

 private:

  ros::NodeHandle &nh_;

  nav_msgs::Odometry cur_pose;
  common_msgs::ChassisState chassis_state;
  common_msgs::ChassisControl control_cmd;
  geometry_msgs::Point last_point;
  common_msgs::Trigger log_trigger;
  bool logStartFlag;

  Para para;

  std::fstream record_file;
  int frame;
  double begin_time;
  double last_time;
  bool open_file_flag;
  void openRecordFile();
  void write2File();
  double distance;
  
};
}

#endif //DATA_LOGGER_HPP
