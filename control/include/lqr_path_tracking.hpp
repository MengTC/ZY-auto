#ifndef LQR_PATH_TRACKING_H
#define LQR_PATH_TRACKING_H
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <fstream>
#include <cmath>

struct LQR_para{
  std::string para_filename;   
};

class LQRPathTracking{
    private:
        // double control_output;
        std::vector<std::vector<double>> k_vector;
        

    public:
        std::string lqr_para_filename;
        LQRPathTracking();
        void readLQRParameters();
        double outputFrontWheelAngle(const double current_speed, const std::vector<double> &current_state);
};

#endif //LQR_PATH_TRACKING