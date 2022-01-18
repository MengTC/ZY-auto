#include "lqr_path_tracking.hpp"

LQRPathTracking::LQRPathTracking(){
}

void LQRPathTracking::readLQRParameters(){
    using namespace std;
    ifstream f(lqr_para_filename);
    string temp;
    vector<double> temp_line;
    while (getline(f,temp))
    {
        stringstream input(temp);
        string out;
        while (input >> out){
            temp_line.push_back(stod(out));
        }
        k_vector.push_back(temp_line);
        temp_line.clear();
    }
    ROS_INFO_STREAM("Loaded! Size of LQR parameters: " << k_vector.size());
    f.close();
}

double LQRPathTracking::outputFrontWheelAngle(const double current_speed, 
                                          const std::vector<double> &current_state){
    int speed_level = floor(current_speed/1.0);
    double u = 0;
    double lateral_error =  current_state[0];
    double dot_lateral_error = current_state[1];
    double heading_error = current_state[2];
    double dot_heading_error = current_state[3];
    int p;
    if (speed_level > 0 && speed_level < 12){
        p = speed_level - 1;
    }else{
         p = 0;
    }
    u = k_vector[p][0] * lateral_error + k_vector[p][1] * dot_lateral_error
       +k_vector[p][2] * heading_error + k_vector[p][3] * dot_heading_error;
    u = u * 180/M_PI;    
    return u;
}
