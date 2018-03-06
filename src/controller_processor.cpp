//
// Created by Leo on 16.02.17.
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


  sub_dyn_1_ = nh_.subscribe(parameter_.sub_rostopic_dynamixel_1,
                            parameter_.queue_size_sub_dynamixel_1,
                            &ControllerProcessor::CallbackDyn1,this);

  sub_dyn_3_ = nh_.subscribe(parameter_.sub_rostopic_dynamixel_3,
                            parameter_.queue_size_sub_dynamixel_3,
                            &ControllerProcessor::CallbackDyn3,this);




  // Create ROS publisher

  // Commands
  pub_cmd_1_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_1,
      parameter_.queue_size_pub_command_1);

  pub_cmd_2_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_2,
      parameter_.queue_size_pub_command_2);

  pub_cmd_3_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_3,
      parameter_.queue_size_pub_command_3);


  // Amgles
  pub_angle_1_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_1,
      parameter_.queue_size_pub_rostopic_1);

  pub_angle_3_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_2,
      parameter_.queue_size_pub_rostopic_2);

  pub_angle_1_filtered_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_3,
      parameter_.queue_size_pub_rostopic_3);

  pub_angle_3_filtered_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_4,
      parameter_.queue_size_pub_rostopic_4);


  dynamic_reconfigure::Server<dynamixel_controller::controllerConfig>::CallbackType f;
  f = boost::bind(&ControllerProcessor::ConfigCallback, this, _1, _2);
  server_.setCallback(f);
}


// Callback for dynamic reconfigure
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



}

// Median filter with 3 elements
float median_n_3(float a,float b,float c)
{
  float median;

  if ((a<=b) && (a<=c))  {
    median = (b<=c) ? b : c;
  } else if ((b<=a) && (b<=c)) {
    median = (a<=c) ? a : c;
  } else {
    median = (a<=b) ? a : b;
  }
  return median;
}


void ControllerProcessor::CallbackEnc1(const geometry_msgs::PointStamped &pt_s_1) {
  ROS_DEBUG("Received message form encoder 1");

  //if (!only_once_) {
  //  start_ = ros::Time::now();
  //  only_once_ = true;
  //}

  // Create valiables for subscribed values
  float pulsewidth_e1 = pt_s_1.point.x;
  float period_e1 = pt_s_1.point.y;
  float dutycycle_e1 = pt_s_1.point.z;
  float uc = 1000000;

  // Calculate andle from pulsewidth
  float angle_deg_a1 = ((pulsewidth_e1*uc*4098/(period_e1*uc))-1)*360/4096;



  // Store the last 3 angles
  angle_val_e1_3_ = angle_val_e1_2_;
  angle_val_e1_2_ = angle_val_e1_1_;
  angle_val_e1_1_ = angle_deg_a1;

  // Calculate median from the last three angle values
  float median_val_e1 = median_n_3(angle_val_e1_1_,angle_val_e1_2_,angle_val_e1_3_);

  // Create message from value
  std_msgs::Float64 angle_1_deg_msg;
  std_msgs::Float64 angle_1_deg_filtered_msg;

  angle_1_deg_msg.data = angle_deg_a1;
  angle_1_deg_filtered_msg.data = median_val_e1;

  // Publish
  pub_angle_1_.publish(angle_1_deg_msg);
  pub_angle_1_filtered_.publish(angle_1_deg_filtered_msg);

  //std::cout << pt_s_1.header.stamp - start_ << ": "
  //<< angle_val_e1_3_ << " | "
  //<< angle_val_e1_2_ << " | "
  //<< angle_val_e1_1_ << ": "
  //<< median_val_e1 << std::endl;

  //ROS_INFO("Received message from encoder 1: [%f]",angle_deg_a1);
  //ROS_INFO("Median message from encoder 1: [%f]",median_val_e1);
}



void ControllerProcessor::CallbackEnc3(const geometry_msgs::PointStamped &pt_s_3) {
  ROS_DEBUG("Received message from encoder 3");

  // Create valiables for subscribed values
  float pulsewidth_e3 = pt_s_3.point.x;
  float period_e3 = pt_s_3.point.y;
  float dutycycle_e3 = pt_s_3.point.z;
  float uc = 1000000;

  // Calculate andle from pulsewidth
  float angle_deg_a3 = ((pulsewidth_e3*uc*4098/(period_e3*uc))-1)*360/4096;

  // Store the last 3 angles
  angle_val_e3_3_ = angle_val_e3_2_;
  angle_val_e3_2_ = angle_val_e3_1_;
  angle_val_e3_1_ = angle_deg_a3;

  // Calculate median from the last three angle values
  float median_val_e3 = median_n_3(angle_val_e3_1_,angle_val_e3_2_,angle_val_e3_3_);

  // Create message from value
  std_msgs::Float64 angle_3_deg_msg;
  std_msgs::Float64 angle_3_deg_filtered_msg;

  angle_3_deg_msg.data = angle_deg_a3;
  angle_3_deg_filtered_msg.data = median_val_e3;


  // Publish
  pub_angle_3_.publish(angle_3_deg_msg);
  pub_angle_3_filtered_.publish(angle_3_deg_filtered_msg);


  //ROS_INFO("Received message from encoder 3: [%f]",angle_deg_a3);
  //ROS_INFO("Median message from encoder 3: [%f]",median_val_e3);

}


void ControllerProcessor::CallbackDyn1(const dynamixel_msgs::JointState& dyn_state_1){

float state_1 = dyn_state_1.current_pos;
ROS_INFO("Received message from dynamixel 1: [%f]",state_1);
}

void ControllerProcessor::CallbackDyn3(const dynamixel_msgs::JointState& dyn_state_3){
float state_3 = dyn_state_3.current_pos;
ROS_INFO("Received message from dynamixel 3: [%f]",state_3);
}



} // namespace controller
