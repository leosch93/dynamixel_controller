//
// Created by marius on 16.04.16.
//

#include "dynamixel_controller/controller_processor.h"

namespace controller {

ControllerProcessor::ControllerProcessor(const ros::NodeHandle& nodehandle,
                                         const ParameterBag& params_bag):
    nh_(nodehandle),
    parameter_(params_bag) {
  ROS_DEBUG("Controller Processor started!");

  // Create ROS subscriber
  sub_enc_1_ = nh_.subscribe(parameter_.sub_rostopic_enc_1,
                           parameter_.queue_size_sub_enc_1,
                           &ControllerProcessor::CallbackImg, this);

  sub_enc_3_ = nh_.subscribe(parameter_.sub_rostopic_enc_3,
                          parameter_.queue_size_sub_enc_3,
                          &ControllerProcessor::CallbackImg, this);

  // Create ROS publisher
  pub_cmd_1_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_1,
      parameter_.queue_size_pub_command_1);

  pub_cmd_3_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_3,
      parameter_.queue_size_pub_command_3);
}

void ControllerProcessor::CallbackImg(const geometry_msgs::PointStamped &pt_s) {
  ROS_DEBUG("Received message form encoder");


}

} // namespace controller
