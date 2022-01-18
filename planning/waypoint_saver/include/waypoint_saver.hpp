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

#ifndef WAYPOINT_SAVER_HPP
#define WAYPOINT_SAVER_HPP

#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <sstream>
#include <string>
#include <fstream>
#include <math.h>
#include <stdio.h>

namespace ns_waypoint_saver {
 struct Para{
   int record_mode;
   std::string waypoint_filename;
   double min_record_distance;
 };

class Wp_saver {
 public:
  // Constructor
  Wp_saver(ros::NodeHandle &nh);
  ~Wp_saver();

  // Getters

  // Setters
  void setLocalization(nav_msgs::Odometry msg);
  void setParameters(Para msg);

  void runAlgorithm();
  inline double distance_compute(nav_msgs::Odometry msg1, nav_msgs::Odometry msg2);
  void write2File(nav_msgs::Odometry msg);

 private:

  ros::NodeHandle &nh_;

  nav_msgs::Odometry cur_pose;
  nav_msgs::Odometry recorded_pose;

  Para para;
  
  std::fstream record_file;
  int frame;

  bool open_file_flag;
  void openRecordFile();

};

}

#endif //WAYPOINT_SAVER_HPP
