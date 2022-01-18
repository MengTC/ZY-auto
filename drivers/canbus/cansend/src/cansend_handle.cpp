/*
    Formula Student Driverless Project (FSD-Project).
    Copyright (c) 2019:
     - chentairan <killasipilin@gmail.com>

    FSD-Project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    FSD-Project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "cansend_handle.hpp"
#include "register.h"
#include <chrono>

namespace ns_cansend {

// Constructor
CansendHandle::CansendHandle(ros::NodeHandle &nodeHandle) :
    nodeHandle_(nodeHandle),
    cansend_(nodeHandle) {
  ROS_INFO("Constructing Handle");
  loadParameters();
  cansend_.setParameters(para_);
  subscribeToTopics();
  publishToTopics();
}

// Getters
int CansendHandle::getNodeRate() const { return node_rate_; }

// Methods
void CansendHandle::loadParameters() {
  ROS_INFO("loading handle parameters");
  if (!nodeHandle_.param<std::string>("chassis_control_topic_name",
                                      chassis_control_topic_name_,
                                      "/chassis/control_cmd")) {
    ROS_WARN_STREAM(
        "Did not load chassis_control_topic_name. Standard value is: " << chassis_control_topic_name_);
  }
  if (!nodeHandle_.param<std::string>("cansend_topic_name",
                                      cansend_topic_name_,
                                      "/sent_messages")) {
    ROS_WARN_STREAM("Did not load cansend_topic_name. Standard value is: " << cansend_topic_name_);
  }
  if (!nodeHandle_.param("node_rate", node_rate_, 1)) {
    ROS_WARN_STREAM("Did not load node_rate. Standard value is: " << node_rate_);
  }
  nodeHandle_.param<int>("cansend_para/send_mode",para_.send_mode,0);
  nodeHandle_.param<double>("cansend_para/test_steer_angle",para_.test_steer_angle,0);
  nodeHandle_.param<double>("cansend_para/test_acc_pedal",para_.test_acc_pedal,0);
  nodeHandle_.param<double>("cansend_para/test_brk_pedal",para_.test_brk_pedal,0);
  nodeHandle_.param<double>("cansend_para/setup_steer_speed",para_.setup_steer_speed,100);
}

void CansendHandle::subscribeToTopics() {
  ROS_INFO("subscribe to topics");
  chassisControlSubscriber_ =
      nodeHandle_.subscribe(chassis_control_topic_name_, 1, &CansendHandle::chassisControlCallback, this);
}

void CansendHandle::publishToTopics() {
  ROS_INFO("publish to topics");
  cansendStatePublisher_ = nodeHandle_.advertise<can_msgs::Frame>(cansend_topic_name_, 100);
}

void CansendHandle::run() {
  // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  cansend_.runAlgorithm();
  // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  // double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  // std::cout << "time cost = " << time_round << ", frequency = " << 1 / time_round << std::endl;
  sendMsg();
}

void CansendHandle::sendMsg() {
  cansendStatePublisher_.publish(cansend_.getFrame(id_0x04EF8480));
  cansendStatePublisher_.publish(cansend_.getFrame(id_0x0C040B2A));
}

void CansendHandle::chassisControlCallback(const common_msgs::ChassisControl &msg) {
  cansend_.setChassisControl(msg);
}
}
