#include <ros/ros.h>
#include "cansend.hpp"
#include <sstream>
ID_0x04EF8480 *id_0x04EF8480;
ID_0x0C040B2A *id_0x0C040B2A;

namespace ns_cansend {
// Constructor
Cansend::Cansend(ros::NodeHandle &nh) : nh_(nh) {
  id_0x04EF8480=new ID_0x04EF8480();
  id_0x04EF8480->SetconDegCmd(0.0);
  id_0x04EF8480->SetcomControlCmd(0.0);
  id_0x04EF8480->SetconRtCmd(0.0);

  id_0x0C040B2A=new ID_0x0C040B2A();
  id_0x0C040B2A->SetconAccReq(0.0);
  id_0x0C040B2A->SetconSta(0.0);
  id_0x0C040B2A->SetcontrolScheme(0.0);

  loop_number = 0;
};

Cansend::~Cansend(){
  delete id_0x04EF8480;
  delete id_0x0C040B2A;
}

// Getters
can_msgs::Frame Cansend::getFrame(protocol *frame) {
  can_msgs::Frame sendframe;
  sendframe.id = frame->id();
  sendframe.dlc = frame->dlc();
  sendframe.is_extended = frame->is_extended();
  sendframe.is_rtr = frame->is_rtr();
  frame->Update(sendframe.data.c_array());  
  return sendframe;
}

// Setters
void Cansend::setChassisControl(common_msgs::ChassisControl msg) {
  chassis_control_cmd = msg;
}
void Cansend::setParameters(const Para &msg){
  para = msg;
}

void Cansend::runAlgorithm() {
  if(para.send_mode == 0){
    //test mode
    id_0x04EF8480->SetconDegCmd(para.test_steer_angle);
    id_0x04EF8480->SetcomControlCmd(1);
    id_0x04EF8480->SetconRtCmd(para.setup_steer_speed);

    id_0x0C040B2A->SetconAccReq(0);
    id_0x0C040B2A->SetconSta(loop_number);
    int control_mode = 0;
    int target_acc_pedal = 0;
    int target_brk_pedal = 0;
    if (para.test_brk_pedal > 0){
      control_mode = 1;
      target_brk_pedal = para.test_brk_pedal;
    }else{
      if (para.test_acc_pedal > 0){
        control_mode = 2;
        target_acc_pedal = para.test_acc_pedal;
      }
    }
    id_0x0C040B2A->SetcontrolScheme(control_mode);
    id_0x0C040B2A->SetAccPedOpenReq(target_acc_pedal);
    id_0x0C040B2A->SetBrkPedOpenReq(target_brk_pedal);
  }else{
    //autonomous driving mode
    id_0x04EF8480->SetconDegCmd(chassis_control_cmd.steer_angle);
    id_0x04EF8480->SetcomControlCmd(1);
    id_0x04EF8480->SetconRtCmd(para.setup_steer_speed);

    id_0x0C040B2A->SetconAccReq(0);
    id_0x0C040B2A->SetconSta(loop_number);
    int control_mode = 0;
    int target_acc_pedal = 0;
    int target_brk_pedal = 0;
    if (chassis_control_cmd.acc_pedal_open_request > 0){
      control_mode = 1;
      target_brk_pedal = chassis_control_cmd.acc_pedal_open_request;
    }else{
      if (chassis_control_cmd.brk_pedal_open_request > 0){
        control_mode = 2;
        target_acc_pedal = chassis_control_cmd.brk_pedal_open_request;
      }
    }
    id_0x0C040B2A->SetcontrolScheme(control_mode);
    id_0x0C040B2A->SetAccPedOpenReq(target_acc_pedal);
    id_0x0C040B2A->SetBrkPedOpenReq(target_brk_pedal);
  }

  if (loop_number >= 16){
    loop_number = 0;
  }  
  loop_number += 1;
}


}
