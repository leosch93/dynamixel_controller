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
                           &ControllerProcessor::CallbackEnc1, this);

  sub_enc_3_ = nh_.subscribe(parameter_.sub_rostopic_enc_3,
                          parameter_.queue_size_sub_enc_3,
                          &ControllerProcessor::CallbackEnc3, this);

  // Create ROS publisher
  pub_cmd_1_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_1,
      parameter_.queue_size_pub_command_1);

  pub_cmd_2_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_2,
      parameter_.queue_size_pub_command_2);

  pub_cmd_3_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_3,
      parameter_.queue_size_pub_command_3);

  dynamic_reconfigure::Server<dynamixel_controller::controllerConfig>::CallbackType f;
  f = boost::bind(&ControllerProcessor::ConfigCallback, this, _1, _2);
  server_.setCallback(f);
}
// 2 arguments correct?

void ControllerProcessor::ConfigCallback(
  dynamixel_controller::controllerConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f",
            config.double_param_1,
            config.double_param_2,
            config.double_param_3);


            std_msgs::Float64 testangle_1;
            testangle_1.data = config.double_param_1/360*2*M_PI;
            pub_cmd_1_.publish(testangle_1);

            std_msgs::Float64 testangle_2;
            testangle_2.data = config.double_param_2/360*2*M_PI;
            pub_cmd_2_.publish(testangle_2);

            std_msgs::Float64 testangle_3;
            testangle_3.data = config.double_param_3/360*2*M_PI;
            pub_cmd_3_.publish(testangle_3);


            // 1) Parameter empfangen und local parameter zuweisen
            // 2) Publish and pub_cmd_1
}

void ControllerProcessor::CallbackEnc1(const geometry_msgs::PointStamped &pt_s_1) {
  ROS_DEBUG("Received message form encoder 1");


  float pulsewidth_e1 = pt_s_1.point.x;
  float period_e1 = pt_s_1.point.y;
  float dutycycle_e1 = pt_s_1.point.z;
  float uc = 1000000;

  float angle_deg_a1 = ((pulsewidth_e1*uc*4098/(period_e1*uc))-1)*360/4096;

  std_msgs::Float64 angle_deg_msg;
  angle_deg_msg.data = angle_deg_a1;

  // Publish
  //pub_cmd_1_.publish(angle_deg_msg);

  ROS_INFO("Received message form encoder 1: [%f]",angle_deg_a1);



}

void ControllerProcessor::CallbackEnc3(const geometry_msgs::PointStamped &pt_s_3) {
  ROS_DEBUG("Received message form encoder 3");

  float pulsewidth_e3 = pt_s_3.point.x;
  float period_e3 = pt_s_3.point.y;
  float dutycycle_e3 = pt_s_3.point.z;
  float uc = 1000000;

  float angle_deg_a3 = ((pulsewidth_e3*uc*4098/(period_e3*uc))-1)*360/4096;

  std_msgs::Float64 angle_deg_msg;
  angle_deg_msg.data = angle_deg_a3;

  //pub_cmd_3_.publish(angle_deg_msg);

  ROS_INFO("Received message form encoder 3: [%f]",angle_deg_a3);

}


} // namespace controller
