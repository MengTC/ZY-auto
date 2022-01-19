#include <ros/ros.h>
#include "data_logger_handle.hpp"
#include "register.h"
#include <chrono>

namespace ns_data_logger {

// Constructor
DataLoggerHandle::DataLoggerHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    data_logger_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  data_logger_.setParameters(para_);
  subscribeToTopics();
  publishToTopics();
}

// Getters
int DataLoggerHandle::getNodeRate() const { return node_rate_; }

// Methods
void DataLoggerHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("localization_topic_name",
                                      localization_topic_name_,
                                      "/localization/utmpose")) {
    ROS_WARN_STREAM(
        "Did not load localization_topic_name. Standard value is: " << localization_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("chassis_state_topic_name",
                                      chassis_state_topic_name_,
                                      "/chassis_state")) {
    ROS_WARN_STREAM(
        "Did not load chassis_state_topic_name. Standard value is: " << chassis_state_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("control_command_topic_name",
                                      control_command_topic_name_,
                                      "/chassis_control")) {
    ROS_WARN_STREAM(
        "Did not load control_command_topic_name. Standard value is: " << control_command_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
  nodeHandle_.param<std::string>("log_filename",para_.log_filename," ");
  //nodeHandle_.param("config_name",variable_name,value);
}

void DataLoggerHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  localizationSubscriber_ =
      nodeHandle_.subscribe(localization_topic_name_, 1, &DataLoggerHandle::localizationCallback, this);
  controlCommandSubscriber_ =
      nodeHandle_.subscribe(control_command_topic_name_, 1, &DataLoggerHandle::controlCommandCallback, this);
  chassisStateSubscriber_ =
      nodeHandle_.subscribe(chassis_state_topic_name_, 1, &DataLoggerHandle::chassisStateCallback, this);
}

void DataLoggerHandle::publishToTopics() {
  ROS_INFO("publish to topics");
}

void DataLoggerHandle::run() {
  // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  // if(localization_flag == 1 && control_command_flag == 1 && chassis_state_flag == 1){
  
  if(localization_flag == 1 | control_command_flag == 1){
    data_logger_.runAlgorithm();
    localization_flag = 0;
    control_command_flag = 0;
    chassis_state_flag = 0;
  }
  else{
    ROS_INFO("localization flag: %d, control command flag: %d, chassis_state_flag: %d",
            localization_flag,control_command_flag,chassis_state_flag);
  }
  
  // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  // double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  // std::cout << "time cost = " << time_round << ", frequency = " << 1 / time_round << std::endl;
  sendMsg();
}

void DataLoggerHandle::sendMsg() {
  // data_loggerStatePublisher_.publish(data_logger_.getConeDetections());
}

void DataLoggerHandle::localizationCallback(const nav_msgs::Odometry &msg) {
  data_logger_.setLocalization(msg);
  localization_flag = 1;
}

void DataLoggerHandle::chassisStateCallback(const common_msgs::ChassisState &msg){
  data_logger_.setChassisState(msg);
  chassis_state_flag = 1;
}

void DataLoggerHandle::controlCommandCallback(const common_msgs::ChassisControl &msg){
  data_logger_.setControlCommand(msg);
  control_command_flag = 1;
}
}