#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#define KAPPA_MAX 9e10
#define KAPPA_MIN (1/KAPPA_MAX)


#include <cmath>
#include "autoware_msgs/Waypoint.h"
#include "ros/ros.h"
#include <libwaypoint_follower/libwaypoint_follower.h>

struct Pure_pursuit_para{
   std::string mode;
   double lookahead_distance;
   double k_pre;
};


class Pure_pursuit{
 private:
    double wheel_base; //wheel base
 public:
    Pure_pursuit(const double wheelbase);
    double outputFrontWheelAngle(geometry_msgs::Point target, geometry_msgs::Pose current_pose);
   //  double outputFrontWheelAngle(autoware_msgs::Waypoint reference_waypoint);
};

#endif // PURE_PURSUIT

