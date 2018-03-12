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

  pub_angle_1_filtered_o_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_3,
      parameter_.queue_size_pub_rostopic_3);

  pub_angle_3_filtered_o_ = nh_.advertise<std_msgs::Float64>(
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

// Callback for dynamic reconfigure
// void ControllerProcessor::ConfigCallback(
//   dynamixel_controller::controllerConfig &config, uint32_t level) {
//   ROS_INFO("Reconfigure Request: %f %f %f %d %d %f %f %f",
//             config.double_param_1,
//             config.double_param_2,
//             config.double_param_3,
//             config.bool_start_positive,
//             config.bool_start_negative,
//             config.double_param_inc,
//             config.max_angle_pos,
//             config.max_angle_neg);
//
//             bool start_pos=config.bool_start_positive;
//             bool start_neg=config.bool_start_negative;
//             float incr=config.double_param_inc;
//             float max_angle_pos=config.max_angle_pos;
//             float max_angle_neg=config.max_angle_neg;




            // while(!start_pos && !start_neg){

              // std_msgs::Float64 testangle_1;
              // testangle_1.data = config.double_param_1/360*2*M_PI;
              // pub_cmd_1_.publish(testangle_1);
              //
              // std_msgs::Float64 testangle_2;
              // testangle_2.data = config.double_param_2/360*2*M_PI;
              // pub_cmd_2_.publish(testangle_2);
              //
              // std_msgs::Float64 testangle_3;
              // testangle_3.data = config.double_param_3/360*2*M_PI;
              // pub_cmd_3_.publish(testangle_3);
              // ROS_INFO("The published testangle is: %f",testangle_3.data*360/2/M_PI);
              // if(start_pos)
              // {
              //   break;
              // }
            // }

            // std_msgs::Float64 testangle_3;
            // testangle_3.data = 0;

            // if(start_pos){
            //   pub_cmd_3_.publish(testangle_3);
            //   float temp_3_pos = testangle_3.data*360/2/M_PI;
            //   ROS_INFO("The published testangle is: %f",testangle_3.data*360/2/M_PI);
            //   ROS_INFO("Test_angle_temp_deg %f",temp_3_pos);
            //   ros::Duration(5).sleep();
            //   for(temp_3_pos <= max_angle_pos){
            //     ROS_INFO("in if");
            //     testangle_3.data = (temp_3_pos+incr)/360*2*M_PI;
            //   }
            //   if(!start_pos)
            //   {
            //     break;
            //   }
            // }

            // if(start_neg){
            //   pub_cmd_3_.publish(testangle_3);
            //   float temp_3_neg = testangle_3.data*360/2/M_PI;
            //   ROS_INFO("The published testangle is: %f",testangle_3.data*360/2/M_PI);
            //   ROS_INFO("Test_angle_temp_deg %f",temp_3_neg);
            //   ros::Duration(5).sleep();
            //   for(temp_3_neg=testangle_3.data*360/2/M_PI;temp_3_neg >= max_angle_neg;temp_3_neg=(temp_3_neg-incr)/360*2*M_PI){
            //     ROS_INFO("in if");
            //     testangle_3.data = temp_3_neg;
            //     ROS_INFO("testangle_3.data %f from if",temp_3_neg);
            //   }
              // if(!start_neg)
              // {
              //   break
              // }
            // }


            // ros::spin ();
// }//namespace callback dynamic reconfigure

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

void ControllerProcessor::CallbackDyn1(const dynamixel_msgs::JointState& dyn_state_1){
//float state_1 = dyn_state_1.current_pos;

if (only_once_dyn_1_== true) {
  dynam_state_1_initial_ = dyn_state_1.current_pos/2/M_PI*360;
  only_once_dyn_1_ = false;
}
dynam_angle_1_ = dyn_state_1.current_pos/2/M_PI*360;
//ROS_INFO("Received message from dynamixel 1: [%f]",state_1);
//ROS_INFO("Received initial value froonly_once_enc_1_m dynamixel 1: [%f]",dynam_state_1_initial_);
}

void ControllerProcessor::CallbackDyn3(const dynamixel_msgs::JointState& dyn_state_3){
//float state_3 = dyn_state_3.current_pos;

if (only_once_dyn_3_ == true) {
  dynam_state_3_initial_ = dyn_state_3.current_pos/2/M_PI*360;
  only_once_dyn_3_ = false;
}
dynam_angle_3_ = dyn_state_3.current_pos/2/M_PI*360;
//ROS_INFO("Received message from dynamixel 3: [%f]",state_3/2/M_PI*360);
//ROS_INFO("Received initial value from dynamixel 3: [%f]",dynam_state_3_initial_);
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



  if (offset_calculated_1 == false){

      if (only_once_enc_1_1_== true){
        only_once_enc_1_1_ = false;
      }
      else if (only_once_enc_1_2_ == true){
        only_once_enc_1_2_ = false;
      }
      else{
        encoder_angle_1_initial_ =  median_val_e1;
            if (dynam_state_1_initial_ != 0){
              offset_angle_a_1_ = encoder_angle_1_initial_ - dynam_state_1_initial_;
              offset_calculated_1 = true;
            }
      }

    //ROS_INFO("Received angle from encoder 3: [%f]",angle_deg_a3);
    //ROS_INFO("Median  from encoder 3: [%f]",median_val_e3);
    //ROS_INFO("State initial dynamixel: [%f]",dynam_state_3_initial_);
    //ROS_INFO("Received initial angle from encoder 3: [%f]",encoder_angle_3_initial_);
    //ROS_INFO("Received initial angle from dynamixel 3: [%f]",dynam_state_3_initial_);
    //ROS_INFO("Offset angle from encoder 3: [%f]",offset_angle_a_3_);
    //ROS_INFO("Encoder 3 filtered and offset corrected: [%f]",enc_3_angle_filt_offset_);
    //ROS_INFO("----");
    }


    enc_1_angle_filt_offset_ = median_val_e1-offset_angle_a_1_;
    //ROS_INFO("Encoder 1 filtered and offset corrected: [%f]",enc_1_angle_filt_offset_);

    // Calculate angle difference
    angle_diff_a_1_ = enc_1_angle_filt_offset_-dynam_angle_1_;

    // ROS_INFO("enc_1_angle_filt_offset_ = [%f]",enc_1_angle_filt_offset_);
    // ROS_INFO("dynam_angle_1_ = [%f]",dynam_angle_1_);
    // ROS_INFO("angle_diff_a_1_ = [%f]",angle_diff_a_1_);

    // Calculate torque estimate
    t_est_1_ = angle_diff_a_1_*k_1_;
    // ROS_INFO("Torque estimate in Nm = [%f]",t_est_1_);

    // Create message from value
    std_msgs::Float64 t_est_1_msg;
    std_msgs::Float64 angle_1_deg_filtered_o_msg;

    t_est_1_msg.data = t_est_1_;
    angle_1_deg_filtered_o_msg.data = enc_1_angle_filt_offset_;

    // Publish
    pub_angle_1_.publish(t_est_1_msg);
    pub_angle_1_filtered_o_.publish(angle_1_deg_filtered_o_msg);


} // namespace Callback 1



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






if (offset_calculated_3 == false){

    if (only_once_enc_3_1_== true){
      only_once_enc_3_1_ = false;
    }
    else if (only_once_enc_3_2_ == true){
      only_once_enc_3_2_ = false;
    }
    else{
      encoder_angle_3_initial_ =  median_val_e3;
          if (dynam_state_3_initial_ != 0){
            offset_angle_a_3_ = encoder_angle_3_initial_ - dynam_state_3_initial_;
            offset_calculated_3 = true;
          }
    }

  //ROS_INFO("Received angle from encoder 3: [%f]",angle_deg_a3);
  //ROS_INFO("Median  from encoder 3: [%f]",median_val_e3);
  //ROS_INFO("State initial dynamixel: [%f]",dynam_state_3_initial_);
  //ROS_INFO("Received initial angle from encoder 3: [%f]",encoder_angle_3_initial_);
  //ROS_INFO("Received initial angle from dynamixel 3: [%f]",dynam_state_3_initial_);
  //ROS_INFO("Offset angle from encoder 3: [%f]",offset_angle_a_3_);
  //ROS_INFO("Encoder 3 filtered and offset corrected: [%f]",enc_3_angle_filt_offset_);
  }

  // ROS_INFO("OFFSET corrected [%i]",offset_calculated_3);

  enc_3_angle_filt_offset_ = median_val_e3-offset_angle_a_3_;
  //ROS_INFO("Encoder 3 filtered and offset corrected: [%f]",enc_3_angle_filt_offset_);


  // Calculate angle difference
  angle_diff_a_3_ = enc_3_angle_filt_offset_-dynam_angle_3_;
  // ROS_INFO("enc_3_angle_filt_offset_ = [%f]",enc_3_angle_filt_offset_);
  // ROS_INFO("dynam_angle_3_ = [%f]",dynam_angle_3_);
  // ROS_INFO("angle_diff_a_3_ = [%f]",angle_diff_a_3_);

  // Calculate torque estimate
  t_est_3_ = angle_diff_a_3_*k_3_;

  // Create message from value
  std_msgs::Float64 t_est_3_msg;
  std_msgs::Float64 angle_3_deg_filtered_o_msg;

  t_est_3_msg.data = angle_deg_a3;
  angle_3_deg_filtered_o_msg.data = enc_3_angle_filt_offset_;

  // Publish
  pub_angle_3_.publish(t_est_3_msg);
  pub_angle_3_filtered_o_.publish(angle_3_deg_filtered_o_msg);



} // namespace callback 3




} // namespace controller
