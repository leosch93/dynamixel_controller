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
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %d %d %d %d %f %f %f %f %f",

            config.double_param_1,
            config.double_param_2,
            config.double_param_3,

            config.double_point_x,
            config.double_point_y,
            config.double_point_z,

            config.bool_test,
            config.bool_start_positive,
            config.bool_start_negative,
            config.bool_start_both,
            config.double_param_inc,
            config.max_angle_pos,
            config.max_angle_neg,
            config.max_angle_both_pos,
            config.max_angle_both_neg);


            bool testbench_settings=config.bool_test;
            bool start_pos=config.bool_start_positive;
            bool start_neg=config.bool_start_negative;
            bool bool_start_both=config.bool_start_both;
            float incr=config.double_param_inc;
            float max_angle_pos=config.max_angle_pos;
            float max_angle_neg=config.max_angle_neg;
            float max_angle_both_pos=config.max_angle_both_pos;
            float max_angle_both_neg=config.max_angle_both_neg;

            double fk_testing_angle_1 = config.double_param_1*2.0*M_PI/360.0;
            double fk_testing_angle_2 = config.double_param_2*2.0*M_PI/360.0;
            double fk_testing_angle_3 = config.double_param_3*2.0*M_PI/360.0;

            double ik_testing_position_x = config.double_point_x;
            double ik_testing_position_y = config.double_point_y;
            double ik_testing_position_z = config.double_point_z;

            // ROS_INFO("Entered manual control mode");
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


            // ROS_INFO("Testing forward Kinematics given angles %f %f %f",fk_testing_angle_1,fk_testing_angle_2,fk_testing_angle_3);
            // Eigen::Vector3d pos_3d = ControllerProcessor::Joint_to_position(fk_testing_angle_1/360.0*2.0*M_PI, fk_testing_angle_2/360.0*2.0*M_PI, fk_testing_angle_3/360.0*2.0*M_PI);
            // ROS_INFO("Position of forward kinemaics %f %f %f",pos_3d(0), pos_3d(1),pos_3d(2));

            // Eigen::Matrix4f transform1 = ControllerProcessor::T_world_dynamixel_to_hebi(fk_testing_angle_3/360*2*M_PI);
            // ROS_INFO("transform1 calculated World to Dynamixel_to_hebi");
            // ROS_INFO("%f ||%f|| %f|| %f",transform1(0,0),transform1(0,1),transform1(0,2),transform1(0,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform1(1,0),transform1(1,1),transform1(1,2),transform1(1,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform1(2,0),transform1(2,1),transform1(2,2),transform1(2,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform1(3,0),transform1(3,1),transform1(3,2),transform1(3,3));
            //
            // Eigen::Matrix4f transform2 = ControllerProcessor::T_dynamixel_to_hebi_clamp2(fk_testing_angle_2/360*2*M_PI);
            // ROS_INFO("transform2 calculated Dynamixel_to_hebi to Clamp2");
            // ROS_INFO("%f ||%f|| %f|| %f",transform2(0,0),transform2(0,1),transform2(0,2),transform2(0,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform2(1,0),transform2(1,1),transform2(1,2),transform2(1,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform2(2,0),transform2(2,1),transform2(2,2),transform2(2,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform2(3,0),transform2(3,1),transform2(3,2),transform2(3,3));
            //
            //
            // Eigen::Matrix4f transform3 = ControllerProcessor::T_clamp2_clamp1(fk_testing_angle_1/360*2*M_PI);
            // ROS_INFO("transform1 calculated Clamp2 to Clamp1");
            // ROS_INFO("%f ||%f|| %f|| %f",transform3(0,0),transform3(0,1),transform3(0,2),transform3(0,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform3(1,0),transform3(1,1),transform3(1,2),transform3(1,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform3(2,0),transform3(2,1),transform3(2,2),transform3(2,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform3(3,0),transform3(3,1),transform3(3,2),transform3(3,3));
            //
            // Eigen::Matrix4f transform4 = ControllerProcessor::T_clamp1_tip();
            // ROS_INFO("transform4 Clamp1 to tip");
            // ROS_INFO("%f ||%f|| %f|| %f",transform4(0,0),transform4(0,1),transform4(0,2),transform4(0,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform4(1,0),transform4(1,1),transform4(1,2),transform4(1,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform4(2,0),transform4(2,1),transform4(2,2),transform4(2,3));
            // ROS_INFO("%f ||%f|| %f|| %f",transform4(3,0),transform4(3,1),transform4(3,2),transform4(3,3));

            // Eigen::Matrix3f testrot = ControllerProcessor::Joint_to_rotation_mat(0,0,0);
            // ROS_INFO("RotationMatrix");
            // ROS_INFO("%f ||%f|| %f",testrot(0,0),testrot(0,1),testrot(0,2));
            // ROS_INFO("%f ||%f|| %f",testrot(1,0),testrot(1,1),testrot(1,2));
            // ROS_INFO("%f ||%f|| %f",testrot(2,0),testrot(2,1),testrot(2,2));
            // ROS_INFO("----");
            //
            // Eigen::Quaternionf testquat = ControllerProcessor::Joint_to_quaternion(0,0,0);
            // ROS_INFO("Quaternion %f ||%f|| %f|| %f",testquat.w(),testquat.x(),testquat.y(),testquat.z());
            //
            // Eigen::Matrix3f testmat = ControllerProcessor::Quaternion_to_rotation_mat(testquat);
            // ROS_INFO("BacktoRotationMatrix");
            // ROS_INFO("%f ||%f|| %f",testmat(0,0),testmat(0,1),testmat(0,2));
            // ROS_INFO("%f ||%f|| %f",testmat(1,0),testmat(1,1),testmat(1,2));
            // ROS_INFO("%f ||%f|| %f",testmat(2,0),testmat(2,1),testmat(2,2));
            // ROS_INFO("----");
            // Eigen::Matrix3f testmat = Joint_to_position_jacobian(0,0,0);
            // ROS_INFO("testing");
            // ROS_INFO("testing jacibians");
            // ROS_INFO("%f ||%f|| %f",testmat(0,0),testmat(0,1),testmat(0,2));
            // ROS_INFO("%f ||%f|| %f",testmat(1,0),testmat(1,1),testmat(1,2));
            // ROS_INFO("%f ||%f|| %f",testmat(2,0),testmat(2,1),testmat(2,2));
            //
            // Eigen::Matrix3f testmat_1 = Joint_to_rotation_jacobian(0,0,0);
            // ROS_INFO("testing");
            // ROS_INFO("testing jacibians");
            // ROS_INFO("%f ||%f|| %f",testmat_1(0,0),testmat_1(0,1),testmat_1(0,2));
            // ROS_INFO("%f ||%f|| %f",testmat_1(1,0),testmat_1(1,1),testmat_1(1,2));
            // ROS_INFO("%f ||%f|| %f",testmat_1(2,0),testmat_1(2,1),testmat_1(2,2));


            Eigen::Vector3f zeroposition = Joint_to_position(fk_testing_angle_1,fk_testing_angle_2,fk_testing_angle_3);
            ROS_INFO("Input 3 Joints a1 = %f||a2 = %f|| a3 = %f",fk_testing_angle_1*360/2.0/M_PI,fk_testing_angle_2*360/2.0/M_PI,fk_testing_angle_3*360/2.0/M_PI);
            ROS_INFO("x = %f ||%f|| %f",zeroposition(0),zeroposition(1),zeroposition(2));


            // Eigen::Vector3f desired_position(ik_testing_position_x,ik_testing_position_y,ik_testing_position_z);
            Eigen::Vector3f desired_position(0.567438,0.701206,0.421076);
            Eigen::Quaternionf orientation_quat(1,0,0,0);
            Eigen::Vector3f r_init(9/360.0*2.0*M_PI,9/360.0*2.0*M_PI,9/360.0*2.0*M_PI);
            double epsilon = 0.01;
            Eigen::Vector3f inversetest = inverse_kinematics(desired_position,orientation_quat,r_init,epsilon);
            ROS_INFO("Input Position x = %f||y = %f|| z = %f",0.567438,0.701206,0.421076);
            ROS_INFO("Resulting angles a1 = %f || a2 = %f|| a3 = %f",inversetest(0)*360.0/2.0/M_PI,inversetest(1)*360.0/2.0/M_PI,inversetest(2)*360.0/2.0/M_PI);

            if(testbench_settings){
              if(!start_pos && !start_neg && !bool_start_both){
                ROS_INFO("Entered manual control mode");
                std_msgs::Float64 testangle_1;
                testangle_1.data = M_PI+config.double_param_1/360*2*M_PI;
                pub_cmd_1_.publish(testangle_1);

                std_msgs::Float64 testangle_2;
                testangle_2.data = M_PI+config.double_param_2/360*2*M_PI;
                pub_cmd_2_.publish(testangle_2);

                std_msgs::Float64 testangle_3;
                testangle_3.data = M_PI+config.double_param_3/360*2*M_PI;
                pub_cmd_3_.publish(testangle_3);


              }




              if(start_pos)
              {
                ROS_INFO("Entered loop mode to positive angles");
                std_msgs::Float64 testangle_3;
                float temp_3_pos = testangle_3.data*360/2/M_PI;
                // ROS_INFO("Initialized test angle %f",temp_3_pos);
                ROS_INFO("Sleeping 3s, then starting increasing angles");
                ros::Duration(3).sleep();
                for(temp_3_pos;temp_3_pos <= (max_angle_pos+0.01);temp_3_pos = temp_3_pos+incr)
                    {
                      testangle_3.data = M_PI+temp_3_pos/360*2*M_PI;
                      ROS_INFO("Test_angle to publish from for loop %f",temp_3_pos);
                      pub_cmd_3_.publish(testangle_3);
                      ROS_INFO("The published testing angle is: %f",testangle_3.data*360/2/M_PI);
                      ROS_INFO("Sleeping 1s");
                      ros::Duration(1).sleep();
                    }

                ROS_INFO("Sleeping 3s, then starting decreasing angles");
                ros::Duration(3).sleep();
                temp_3_pos = testangle_3.data*360/2/M_PI;
                for(temp_3_pos;temp_3_pos >= 179.99 ;temp_3_pos = temp_3_pos-incr)
                    {
                      testangle_3.data = temp_3_pos/360*2*M_PI;
                      ROS_INFO("Test_angle to publish from for loop %f",temp_3_pos);
                      pub_cmd_3_.publish(testangle_3);
                      ROS_INFO("The published testing angle is: %f",testangle_3.data*360/2/M_PI);
                      ROS_INFO("Sleeping 1s");
                      ros::Duration(1).sleep();
                    }

                }

                if(start_neg)
                {
                  ROS_INFO("Entered loop mode to negative angles");
                  std_msgs::Float64 testangle_3;
                  float temp_3_neg = testangle_3.data*360/2/M_PI;
                  // ROS_INFO("Initialized test angle %f",temp_3_neg);
                  ROS_INFO("Sleeping 3s, then starting decrease angles");
                  ros::Duration(3).sleep();
                  for(temp_3_neg;temp_3_neg >= (max_angle_neg-0.01);temp_3_neg = temp_3_neg-incr)
                      {
                        testangle_3.data = M_PI+temp_3_neg/360*2*M_PI;
                        // ROS_INFO("Test_angle to publish from for loop %f",temp_3_pos);
                        pub_cmd_3_.publish(testangle_3);
                        ROS_INFO("The published testing angle is: %f",testangle_3.data*360/2/M_PI);
                        ROS_INFO("Sleeping 1s");
                        ros::Duration(1).sleep();
                      }

                  ROS_INFO("Sleeping 3s, then starting increasing angles");
                  ros::Duration(3).sleep();
                  temp_3_neg = testangle_3.data*360/2/M_PI;
                  for(temp_3_neg;temp_3_neg <= 180.01 ;temp_3_neg = temp_3_neg+incr)
                      {
                        testangle_3.data = temp_3_neg/360*2*M_PI;
                        // ROS_INFO("Test_angle to publish from for loop %f",temp_3_neg);
                        pub_cmd_3_.publish(testangle_3);
                        ROS_INFO("The published testing angle is: %f",testangle_3.data*360/2/M_PI);
                        ROS_INFO("Sleeping 1s");
                        ros::Duration(1).sleep();
                      }

                  }



                  if(bool_start_both)
                  {
                    ROS_INFO("Entered loop mode to both angles");
                    std_msgs::Float64 testangle_3;
                    float temp_3_both = testangle_3.data*360/2/M_PI;
                    // ROS_INFO("Initialized test angle %f",temp_3_pos);
                    ROS_INFO("Sleeping 3s, then starting loop angles");
                    ros::Duration(3).sleep();
                    for(temp_3_both;temp_3_both <= (max_angle_both_pos+0.01);temp_3_both = temp_3_both+incr)
                        {
                          testangle_3.data = M_PI+temp_3_both/360*2*M_PI;
                          ROS_INFO("Test_angle to publish from for loop %f",temp_3_both);
                          pub_cmd_3_.publish(testangle_3);
                          ROS_INFO("The published testing angle is: %f",testangle_3.data*360/2/M_PI);
                          ROS_INFO("Sleeping 1s");
                          ros::Duration(1).sleep();
                        }

                    ROS_INFO("Sleeping 3s, then starting decreasing angles");
                    ros::Duration(3).sleep();
                    temp_3_both = testangle_3.data*360/2/M_PI;
                    for(temp_3_both;temp_3_both >= (180+max_angle_both_neg-0.01);temp_3_both = temp_3_both-incr)
                        {
                          testangle_3.data = temp_3_both/360*2*M_PI;
                          ROS_INFO("Test_angle to publish from for loop %f",temp_3_both);
                          pub_cmd_3_.publish(testangle_3);
                          ROS_INFO("The published testing angle is: %f",testangle_3.data*360/2/M_PI);
                          ROS_INFO("Sleeping 1s");
                          ros::Duration(1).sleep();
                        }
                      ROS_INFO("Sleeping 3s, then starting increasing angles");
                      ros::Duration(3).sleep();
                      temp_3_both = testangle_3.data*360/2/M_PI;
                      for(temp_3_both;temp_3_both <= 180.01 ;temp_3_both = temp_3_both+incr)
                          {
                            testangle_3.data = temp_3_both/360*2*M_PI;
                            ROS_INFO("Test_angle to publish from for loop %f",temp_3_both);
                            pub_cmd_3_.publish(testangle_3);
                            ROS_INFO("The published testing angle is: %f",testangle_3.data*360/2/M_PI);
                            ROS_INFO("Sleeping 1s");
                            ros::Duration(1).sleep();
                          }



                    }
                }

}//namespace callback dynamic reconfigure



Eigen::Matrix4f ControllerProcessor::T_world_dynamixel_to_hebi(const double& a3) {
  double a3_offset = 45.0/360.0*2.0*M_PI;

  Eigen::Matrix4f T;
  T << cos(a3-a3_offset), -sin(a3-a3_offset), 0, 0.0325,
      sin(a3-a3_offset), cos(a3-a3_offset), 0, 0,
      0, 0, 1, 0.072,
      0, 0, 0, 1;

  return T;
}

Eigen::Matrix4f ControllerProcessor::T_dynamixel_to_hebi_clamp2(const double& a2) {
  double a2_offset = 135.0/360.0*2.0*M_PI;
  double a2_fix = -90.0/360.0*2.0*M_PI;
  Eigen::Matrix3f Rx;
    Rx << 1, 0, 0,
        0, cos(a2+a2_offset), -sin(a2+a2_offset),
        0, sin(a2+a2_offset), cos(a2+a2_offset);

  Eigen::Matrix3f Rz;
  Rz << cos(a2_fix),-sin(a2_fix),0,
      sin(a2_fix),cos(a2_fix),0,
      0,0,1;

  Eigen::Matrix3f R;
  R = Rx*Rz;

  Eigen::Matrix4f T;
  T << R(0,0), R(0,1), R(0,2), 0.069,
      R(1,0), R(1,1), R(1,2), 0,
      R(2,0), R(2,1), R(2,2), -0.0297,
      0, 0, 0, 1;

  return T;
}

Eigen::Matrix4f ControllerProcessor::T_clamp2_clamp1(const double& a1) {
  double a1_offset = -135.0/360.0*2*M_PI;
  double a1_fix = 90.0/360.0*2.0*M_PI;

  Eigen::Matrix3f Rx;
  Rx << 1, 0 ,0,
        0, cos(a1_fix),-sin(a1_fix),
        0, sin(a1_fix),cos(a1_fix);

  Eigen::Matrix3f Rz;
  Rz << cos(a1+a1_offset),-sin(a1+a1_offset), 0,
        sin(a1+a1_offset), cos(a1+a1_offset), 0,
        0,0,1;

  Eigen::Matrix3f R;
  R = Rx*Rz;





  Eigen::Matrix4f T;
  T << R(0,0), R(0,1), R(0,2), 0,
      R(1,0), R(1,1), R(1,2), -0.0110,
      R(2,0), R(2,1), R(2,2), -0.4624,
      0, 0, 0, 1;

  return T;

}

Eigen::Matrix4f ControllerProcessor::T_clamp1_tip() {

  double angle_fix_x = -90.0/360.0*2.0*M_PI;
  double angle_fix_z = 90.0/360.0*2.0*M_PI;

  Eigen::Matrix3f Rx;
  Rx << 1, 0 ,0,
        0, cos(angle_fix_x),-sin(angle_fix_x),
        0, sin(angle_fix_x),cos(angle_fix_x);

  Eigen::Matrix3f Rz;
  Rz << cos(angle_fix_z),-sin(angle_fix_z), 0,
        sin(angle_fix_z), cos(angle_fix_z), 0,
        0,0,1;

  Eigen::Matrix3f R;


  R = Rx*Rz;

  Eigen::Matrix4f T;
  T << R(0,0), R(0,1), R(0,2), 0,
      R(1,0), R(1,1), R(1,2), 0.616,
      R(2,0), R(2,1), R(2,2), 0.022,
      0, 0, 0, 1;



  return T;

}

Eigen::Matrix4f ControllerProcessor::T_world_tip(const double& a3,const double& a2,const double& a1) {

  Eigen::Matrix4f T;
  T = T_world_dynamixel_to_hebi(a3)*T_dynamixel_to_hebi_clamp2(a2)*T_clamp2_clamp1(a1)*T_clamp1_tip();
  return T;

}

Eigen::Matrix3f ControllerProcessor::Joint_to_rotation_mat(const double& input1, const double& input2, const double& input3) {
  Eigen::Matrix4f T;
  T = T_world_tip(input3,input2,input1);

  Eigen::Matrix3f R;
  R(0,0) = T(0,0);
  R(0,1) = T(0,1);
  R(0,2) = T(0,2);

  R(1,0) = T(1,0);
  R(1,1) = T(1,1);
  R(1,2) = T(1,2);

  R(2,0) = T(2,0);
  R(2,1) = T(2,1);
  R(2,2) = T(2,2);

  return R;
}

Eigen::Vector3f ControllerProcessor::Joint_to_position(const double& input1, const double& input2, const double& input3) {

  Eigen::Matrix4f T;
  T = T_world_tip(input3,input2,input1);
  Eigen::Vector3f pos;

  pos(0) = T(0,3);
  pos(1) = T(1,3);
  pos(2) = T(2,3);


  return pos;
}

Eigen::Quaternionf ControllerProcessor::Joint_to_quaternion(const double& input1, const double& input2, const double& input3) {
  Eigen::Matrix3f Rot;
  Rot = Joint_to_rotation_mat(input1,input2,input3);

  Eigen::Quaternionf q;
  q = Rot;

  return q;
}

Eigen::Matrix3f ControllerProcessor::Quaternion_to_rotation_mat(const Eigen::Quaternionf& q) {

  Eigen::Matrix3f mat;
  mat = q.normalized().toRotationMatrix();
  return mat;
}

Eigen::Quaternionf ControllerProcessor::Multiply_quaternions(const Eigen::Quaternionf& q1,const Eigen::Quaternionf& q2) {
 // ROS uses Hamiltionian
  Eigen::Quaternionf q3;
  q3 = q1*q2;
  return q3;
}

Eigen::Vector3f ControllerProcessor::Rotate_vec_with_quaternion(const Eigen::Quaternionf& q,const Eigen::Vector3f& v) {

  Eigen::Matrix3f Rotation_matrix = Quaternion_to_rotation_mat(q);
  Eigen::Vector3f rotated_vec;
  rotated_vec = Rotation_matrix*v;
  return rotated_vec;
}

Eigen::Matrix3f ControllerProcessor::Joint_to_position_jacobian(const double& input1, const double& input2, const double& input3) {
  Eigen::Matrix4f T01 = T_world_dynamixel_to_hebi(input3);
  Eigen::Matrix4f T12 = T_dynamixel_to_hebi_clamp2(input2);
  Eigen::Matrix4f T23 = T_clamp2_clamp1(input1);
  Eigen::Matrix4f T34 = T_clamp1_tip();

  Eigen::Matrix4f TW1 = T01;
  Eigen::Matrix4f TW2 = TW1*T12;
  Eigen::Matrix4f TW3 = TW2*T23;
  Eigen::Matrix4f TW4 = TW3*T34;

  Eigen::Matrix3f RW1 = TW1.block(0,0,3,3);
  Eigen::Matrix3f RW2 = TW2.block(0,0,3,3);
  Eigen::Matrix3f RW3 = TW3.block(0,0,3,3);
  Eigen::Matrix3f RW4 = TW4.block(0,0,3,3);

  Eigen::Vector3f r_W1 = TW1.block(0,3,3,1);
  Eigen::Vector3f r_W2 = TW2.block(0,3,3,1);
  Eigen::Vector3f r_W3 = TW3.block(0,3,3,1);
  Eigen::Vector3f r_W4 = TW4.block(0,3,3,1);


  Eigen::Vector3f omega_hat_1(0.0, 0.0, 1.0);

  Eigen::Vector3f omega_hat_2(0.0, 1.0, 0.0);

  Eigen::Vector3f omega_hat_3(0.0, 0.0, 1.0);

  Eigen::Vector3f r_Wtip = Joint_to_position(input1,input2,input3);

  Eigen::Vector3f Resultvector_1 = (RW1*omega_hat_1).cross(r_Wtip-r_W1);
  Eigen::Vector3f Resultvector_2 = (RW2*omega_hat_2).cross(r_Wtip-r_W2);
  Eigen::Vector3f Resultvector_3 = (RW3*omega_hat_3).cross(r_Wtip-r_W3);

  Eigen::Matrix3f J;
  J << Resultvector_1(0),Resultvector_2(0),Resultvector_3(0),
        Resultvector_1(1),Resultvector_2(1),Resultvector_3(1),
        Resultvector_1(2),Resultvector_2(2),Resultvector_3(2);




  return J;

}

Eigen::Matrix3f ControllerProcessor::Joint_to_rotation_jacobian(const double& input1, const double& input2, const double& input3) {
  Eigen::Matrix4f T01 = T_world_dynamixel_to_hebi(input3);
  Eigen::Matrix4f T12 = T_dynamixel_to_hebi_clamp2(input2);
  Eigen::Matrix4f T23 = T_clamp2_clamp1(input1);
  Eigen::Matrix4f T34 = T_clamp1_tip();

  Eigen::Matrix4f TW1 = T01;
  Eigen::Matrix4f TW2 = TW1*T12;
  Eigen::Matrix4f TW3 = TW2*T23;
  Eigen::Matrix4f TW4 = TW3*T34;

  Eigen::Matrix3f RW1 = TW1.block(0,0,3,3);
  Eigen::Matrix3f RW2 = TW2.block(0,0,3,3);
  Eigen::Matrix3f RW3 = TW3.block(0,0,3,3);
  Eigen::Matrix3f RW4 = TW4.block(0,0,3,3);


  Eigen::Vector3f omega_hat_1(0.0, 0.0, 1.0);
  Eigen::Vector3f omega_hat_2(0.0, 1.0, 0.0);
  Eigen::Vector3f omega_hat_3(0.0, 0.0, 1.0);


  Eigen::Vector3f Resultvector_1 = RW1*omega_hat_1;
  Eigen::Vector3f Resultvector_2 = RW2*omega_hat_2;
  Eigen::Vector3f Resultvector_3 = RW3*omega_hat_3;

  Eigen::Matrix3f J;
  J << Resultvector_1(0),Resultvector_2(0),Resultvector_3(0),
        Resultvector_1(1),Resultvector_2(1),Resultvector_3(1),
        Resultvector_1(2),Resultvector_2(2),Resultvector_3(2);





  return J;

}

Eigen::Matrix3f ControllerProcessor::SkewMatrix(const Eigen::Vector3f& v) {

  Eigen::Matrix3f mat;

  mat << 0,-v(2),v(1),
          v(2),0,-v(0),
          -v(1),v(0),0;

  return mat;
}

Eigen::MatrixXf ControllerProcessor::map_loc_rot_vel(const Eigen::Quaternionf& q) {


  Eigen::Matrix3f temp_mat;
  temp_mat << q.w()*Eigen::Matrix3f::Identity();

  Eigen::Vector3f temp_vec(q.x(),q.y(),q.z());
  Eigen::Matrix3f temp_mat_2;
  temp_mat_2 << SkewMatrix(temp_vec);

  Eigen::Matrix3f temp_mat_3;
  temp_mat_3 = temp_mat+temp_mat_2;


  Eigen::MatrixXf mat(4,3);
  mat.resize(4,3);
  mat << -q.x(),-q.y(),-q.z(),
        temp_mat_3(0,0),temp_mat_3(0,1),temp_mat_3(0,2),
        temp_mat_3(1,0),temp_mat_3(1,1),temp_mat_3(1,2),
        temp_mat_3(2,0),temp_mat_3(2,1),temp_mat_3(2,2);

  return mat;
}

Eigen::Vector3f ControllerProcessor::inverse_kinematics(const Eigen::Vector3f& r_des,const Eigen::Quaternionf& q_des_IE,const Eigen::Vector3f& r_init,const double& epsilon) {

  int iterations = 0;
  Eigen::Vector3f smallnumber(100,100,100);
  double max_iterations = 100;

  Eigen::Quaternionf q_des_EI = q_des_IE.inverse();

  double input1 = r_init(0);
  double input2 = r_init(1);
  double input3 = r_init(2);


  while(smallnumber.norm()>epsilon && iterations<max_iterations){

    Eigen::Quaternionf q_temp = Joint_to_quaternion(input1,input2,input3);
    Eigen::Quaternionf q_EI = q_temp.inverse();

    Eigen::Vector3f dr = r_des-Joint_to_position(input1,input2,input3);
    Eigen::Vector4f dq;
    dq(0)= q_des_EI.w()-q_EI.w();
    dq(1)= q_des_EI.x()-q_EI.x();
    dq(2)= q_des_EI.y()-q_EI.y();
    dq(3)= q_des_EI.z()-q_EI.z();


    Eigen::Matrix3f R = Joint_to_rotation_mat(input1,input2,input3);
    Eigen::Matrix3f R_trans = R.transpose();


    Eigen::Matrix3f Apos = Joint_to_position_jacobian(input1,input2,input3);
    Eigen::MatrixXf Arot(4,3);
    Arot = map_loc_rot_vel(q_EI)*R_trans*Joint_to_rotation_jacobian(input1,input2,input3);

    Eigen::MatrixXf A(7,3);
    A << Apos(0,0),Apos(0,1),Apos(0,2),
        Apos(1,0),Apos(1,1),Apos(1,2),
        Apos(2,0),Apos(2,1),Apos(2,2),
        Arot(0,0),Arot(0,1),Arot(0,1),
        Arot(1,0),Arot(1,1),Arot(1,2),
        Arot(2,0),Arot(2,1),Arot(2,2),
        Arot(3,0),Arot(3,1),Arot(3,2);

    Eigen::VectorXf vec(7);

      vec(0) = dr(0);
      vec(1) = dr(1);
      vec(2) = dr(2);
      vec(3) = dq(0);
      vec(4) = dq(1);
      vec(5) = dq(2);
      vec(6) = dq(3);

      // largenumbers = A\vec;
      smallnumber = A.colPivHouseholderQr().solve(vec);
      input1 = input1+smallnumber(0);
      input2 = input2+smallnumber(1);
      input3 = input3+smallnumber(2);

      iterations = iterations+1;
  }

  double found_angle_1 = input1;
  double found_angle_2 = input2;
  double found_angle_3 = input3;


  Eigen::Vector3f pos_error = r_des - Joint_to_position(found_angle_1,found_angle_2,found_angle_3);
  Eigen::Quaternionf quaterror_temp_1 = Multiply_quaternions(q_des_EI,Joint_to_quaternion(found_angle_1,found_angle_2,found_angle_3));
  double quaterror_temp_2 = quaterror_temp_1.norm();
  double quat_error = 1-quaterror_temp_2;

  ROS_INFO("Inverse Kinematics terminated after %d iterations",iterations);
  ROS_INFO("Position error %f",pos_error.norm());
  ROS_INFO("Attitude error %f",quat_error);

  Eigen::Vector3f res_vector(found_angle_1,found_angle_2,found_angle_3);
  return res_vector;

}















// Median filter with 3 elements
float median_n_3(float a,float b,float c){
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

  if(only_once_dyn_1_ == true){
    dynam_state_1_initial_ = dyn_state_1.current_pos/2/M_PI*360;
    only_once_dyn_1_ = false;
  }
  dynam_angle_1_ = dyn_state_1.current_pos/2/M_PI*360;
  //ROS_INFO("Received message from dynamixel 1: [%f]",state_1);
  //ROS_INFO("Received initial value froonly_once_enc_1_m dynamixel 1: [%f]",dynam_state_1_initial_);
}

void ControllerProcessor::CallbackDyn3(const dynamixel_msgs::JointState& dyn_state_3){
//float state_3 = dyn_state_3.current_pos;

  if(only_once_dyn_3_ == true){
    dynam_state_3_initial_ = dyn_state_3.current_pos/2/M_PI*360;
    only_once_dyn_3_ = false;
  }
  dynam_angle_3_ = dyn_state_3.current_pos/2/M_PI*360;
  //ROS_INFO("Received message from dynamixel 3: [%f]",state_3/2/M_PI*360);
  //ROS_INFO("Received initial value from dynamixel 3: [%f]",dynam_state_3_initial_);
}


//Callback for Encoder 1
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
    ROS_INFO("Encoder 1 filtered and offset corrected: [%f]",enc_1_angle_filt_offset_);

    // Calculate angle difference
    angle_diff_a_1_ = enc_1_angle_filt_offset_-dynam_angle_1_;

    // ROS_INFO("enc_1_angle_filt_offset_ = [%f]",enc_1_angle_filt_offset_);
    // ROS_INFO("dynam_angle_1_ = [%f]",dynam_angle_1_);
    ROS_INFO("angle_diff_a_1_ = [%f]",angle_diff_a_1_);

    // Calculate torque estimate
    t_est_1_ = angle_diff_a_1_*k_1_;
    ROS_INFO("Torque estimate in Nm = [%f]",t_est_1_);

    // Create message from value
    std_msgs::Float64 t_est_1_msg;
    std_msgs::Float64 angle_1_deg_filtered_o_msg;

    t_est_1_msg.data = t_est_1_;
    angle_1_deg_filtered_o_msg.data = enc_1_angle_filt_offset_;

    // Publish
    pub_angle_1_.publish(t_est_1_msg);
    pub_angle_1_filtered_o_.publish(angle_1_deg_filtered_o_msg);


} // namespace Callback 1


//Callback for Encoder 3
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
