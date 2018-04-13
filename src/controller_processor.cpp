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

  sub_hebi_2_ = nh_.subscribe(parameter_.sub_rostopic_hebi_2,
                          parameter_.queue_size_sub_rostopic_hebi_2,
                          &ControllerProcessor::CallbackHebi2,this);

  sub_enc_3_ = nh_.subscribe(parameter_.sub_rostopic_enc_3,
                          parameter_.queue_size_sub_enc_3,
                          &ControllerProcessor::CallbackEnc3, this);



  sub_tip_pos_ = nh_.subscribe(parameter_.sub_rostopic_tip_position,
                          parameter_.queue_size_sub_rostopic_tip_position,
                          &ControllerProcessor::Callback_tip_position,this);




  // Create ROS publisher

  // Commands
  pub_cmd_1_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_1,
      parameter_.queue_size_pub_command_1);

  pub_cmd_1_dynamixel_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_1_dynamixel,
      parameter_.queue_size_pub_command_1_dynamixel);

  pub_cmd_2_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_2,
      parameter_.queue_size_pub_command_2);

  pub_cmd_2_hebi_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
      parameter_.pub_rostopic_command_2_hebi,
      parameter_.queue_size_pub_command_2_hebi);

  pub_cmd_3_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_3,
      parameter_.queue_size_pub_command_3);

  pub_cmd_3_dynamixel_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_command_3_dynamixel,
      parameter_.queue_size_pub_command_3_dynamixel);




  // Angles
  pub_torque_1_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_1,
      parameter_.queue_size_pub_rostopic_1);

  pub_torque_2_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_2,
      parameter_.queue_size_pub_rostopic_2);

  pub_torque_3_ = nh_.advertise<std_msgs::Float64>(
      parameter_.pub_rostopic_3,
      parameter_.queue_size_pub_rostopic_3);




  dynamic_reconfigure::Server<dynamixel_controller::controllerConfig>::CallbackType f;
  f = boost::bind(&ControllerProcessor::ConfigCallback, this, _1, _2);
  server_.setCallback(f);
}

// Callback for dynamic reconfigure
void ControllerProcessor::ConfigCallback(
  dynamixel_controller::controllerConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f %f %f %f %d %d %d %d %d %d %d %d %d %f %f %f %f %f",

            config.double_param_1,
            config.double_param_2,
            config.double_param_3,

            config.double_point_x,
            config.double_point_y,
            config.double_point_z,

            config.double_angle_init_3,
            config.double_angle_init_2,
            config.double_angle_init_1,

            config.bool_wand,

            config.bool_traj,

            config.bool_fk,

            config.bool_ik,

            config.bool_home,

            config.bool_testbenchsettings,
            config.bool_start_positive,
            config.bool_start_negative,
            config.bool_start_both,
            config.double_param_inc,
            config.max_angle_pos,
            config.max_angle_neg,
            config.max_angle_both_pos,
            config.max_angle_both_neg);

            is_wand_=config.bool_wand;
            bool traj=config.bool_traj;
            bool forward_kin=config.bool_fk;
            bool inverse_kin=config.bool_ik;
            bool homeing_state=config.bool_home;

            bool testbench_settings=config.bool_testbenchsettings;
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

            double ik_initial_angle_3 = config.double_angle_init_3*2.0*M_PI/360.0;
            double ik_initial_angle_2 = config.double_angle_init_2*2.0*M_PI/360.0;
            double ik_initial_angle_1 = config.double_angle_init_1*2.0*M_PI/360.0;





            if(traj) {
              Eigen::Vector3f r_start(ik_testing_position_x,ik_testing_position_y,ik_testing_position_z);
              ROS_INFO("Input coordinates at start x = %f || y = %f || z = %f",r_start(0),r_start(1),r_start(2));
              Eigen::Vector3f r_init(ik_initial_angle_3,ik_initial_angle_2,ik_initial_angle_1);
              ROS_INFO("Input initial angles a1 = %f || a2 = %f || a3 = %f",r_init(0),r_init(1),r_init(2));

              ROS_INFO("Inverse Kinematics");
              Eigen::Vector3f q = inverse_kinematics(r_start,r_init,0.001);
              ROS_INFO("Found angles in rad a3 = %f || a2 = %f || a1 = %f",q(0),q(1),q(2));
              ROS_INFO("Found angles in deg a3 = %f || a2 = %f || a1 = %f",q(0)/2.0/M_PI*360.0,q(1)/2.0/M_PI*360.0,q(2)/2.0/M_PI*360.0);


              ROS_INFO("Now publishing joint commands");
              std_msgs::Float64 ik_angle_3;
              std_msgs::Float64 ik_angle_2;
              std_msgs::Float64 ik_angle_1;

              //  HEBI
              trajectory_msgs::JointTrajectory angle_2_msg;
              angle_2_msg.joint_names.resize(1);
              angle_2_msg.joint_names[0] = "X5-4/M1";
              angle_2_msg.points.resize(1);
              angle_2_msg.points[0].positions.push_back(-q(1));
              // Dynamixel
              std_msgs::Float64 angle_3_dyn;
              std_msgs::Float64 angle_1_dyn;
              angle_3_dyn.data = q(0)+M_PI;
              angle_1_dyn.data =  q(2)+M_PI;


              ik_angle_3.data = q(0);
              ik_angle_2.data = q(1);
              ik_angle_1.data = q(2);

              // Publish
              pub_cmd_3_.publish(ik_angle_3);
              pub_cmd_2_.publish(ik_angle_2);
              pub_cmd_1_.publish(ik_angle_1);

              pub_cmd_3_dynamixel_.publish(angle_3_dyn);
              pub_cmd_2_hebi_.publish(angle_2_msg);
              pub_cmd_1_dynamixel_.publish(angle_1_dyn);

              ROS_INFO("IK published commands a3 = %f || a2 = %f || a1 = %f",ik_angle_3.data/2.0/M_PI*360.0,ik_angle_2.data/2.0/M_PI*360.0,ik_angle_1.data/2.0/M_PI*360.0);

              float sample_time = 0.05;
              // ROS_INFO("sample_time %f",sample_time);
              Eigen::Vector3f r_goal(0.7,0.7,0.4);
              float velocity = 0.001;

              Eigen::Vector3f dr = r_goal-r_start;
              ROS_INFO("dr0 %f ||dr1 %f ||dr2 %f",dr(0),dr(1),dr(2));
              float traj_time = dr.norm()/velocity;
              ROS_INFO("traj_time %f",traj_time);
              float time_steps = floor(traj_time/sample_time);
              ROS_INFO("time_steps %f",time_steps);

              Eigen::VectorXf t = Eigen::VectorXf::LinSpaced(time_steps,1,time_steps)*sample_time;

              Eigen::VectorXf xtraj = Eigen::VectorXf::LinSpaced(time_steps,r_start(0),r_goal(0));
              Eigen::VectorXf ytraj = Eigen::VectorXf::LinSpaced(time_steps,r_start(1),r_goal(1));
              Eigen::VectorXf ztraj = Eigen::VectorXf::LinSpaced(time_steps,r_start(2),r_goal(2));
              ROS_INFO("t %f ||t %f ||t %f",t(0),t(1),t(2));

              ROS_INFO("xtraj %f ||xtraj %f ||xtraj %f",xtraj(0),xtraj(1),xtraj(2));
              ROS_INFO("ytraj %f ||ytraj %f ||ytraj %f",ytraj(0),ytraj(1),ytraj(2));
              ROS_INFO("ztraj %f ||ztraj %f ||ztraj %f",ztraj(0),ztraj(1),ztraj(2));

              for(int i = 0;i<time_steps;i++) {
                Eigen::Vector3f pos(xtraj(i),ytraj(i),ztraj(i));
                Eigen::Vector3f q = inverse_kinematics(pos,r_init,0.001);
                // ROS_INFO("At i = %i",i);
                ROS_INFO("Found angles in deg a3 = %f || a2 = %f || a1 = %f",q(0)/2.0/M_PI*360.0,q(1)/2.0/M_PI*360.0,q(2)/2.0/M_PI*360.0);


                ROS_INFO("Now publishing joint commands");
                std_msgs::Float64 ik_angle_3;
                std_msgs::Float64 ik_angle_2;
                std_msgs::Float64 ik_angle_1;

                //  HEBI
                trajectory_msgs::JointTrajectory angle_2_msg;
                angle_2_msg.joint_names.resize(1);
                angle_2_msg.joint_names[0] = "X5-4/M1";
                angle_2_msg.points.resize(1);
                angle_2_msg.points[0].positions.push_back(-q(1));
                // Dynamixel
                std_msgs::Float64 angle_3_dyn;
                std_msgs::Float64 angle_1_dyn;
                angle_3_dyn.data = q(0)+M_PI;
                angle_1_dyn.data =  q(2)+M_PI;


                ik_angle_3.data = q(0);
                ik_angle_2.data = q(1);
                ik_angle_1.data = q(2);

                // Publish
                pub_cmd_3_.publish(ik_angle_3);
                pub_cmd_2_.publish(ik_angle_2);
                pub_cmd_1_.publish(ik_angle_1);

                pub_cmd_3_dynamixel_.publish(angle_3_dyn);
                pub_cmd_2_hebi_.publish(angle_2_msg);
                pub_cmd_1_dynamixel_.publish(angle_1_dyn);

                // ros::Duration(1).sleep();

                r_init(0) = q(0);
                r_init(1) = q(1);
                r_init(2) = q(2);
                ROS_INFO("New initial angles a1 = %f || a2 = %f || a3 = %f",r_init(0)/2.0/M_PI*360.0,r_init(1)/2.0/M_PI*360.0,r_init(2)/2.0/M_PI*360.0);

              }



            }

            if(forward_kin) {


              std_msgs::Float64 angle_3;
              std_msgs::Float64 angle_2;
              std_msgs::Float64 angle_1;

              angle_3.data = fk_testing_angle_3;
              angle_2.data = fk_testing_angle_2;
              angle_1.data = fk_testing_angle_1;

              //  HEBI
              trajectory_msgs::JointTrajectory angle_2_msg;
              angle_2_msg.joint_names.resize(1);
              angle_2_msg.joint_names[0] = "X5-4/M1";
              angle_2_msg.points.resize(1);
              angle_2_msg.points[0].positions.push_back(-fk_testing_angle_2);
              // Dynamixel
              std_msgs::Float64 angle_3_dyn;
              std_msgs::Float64 angle_1_dyn;
              angle_3_dyn.data = fk_testing_angle_3+M_PI;
              angle_1_dyn.data = fk_testing_angle_1+M_PI;



              // Publish
              pub_cmd_3_.publish(angle_3);
              pub_cmd_2_.publish(angle_2);
              pub_cmd_1_.publish(angle_1);

              pub_cmd_3_dynamixel_.publish(angle_3_dyn);
              pub_cmd_2_hebi_.publish(angle_2_msg);
              pub_cmd_1_dynamixel_.publish(angle_1_dyn);



              ROS_INFO("FK published angles in deg a3 = %f || a2 = %f || a1 = %f",angle_3.data/2.0/M_PI*360.0,angle_2.data/2.0/M_PI*360.0,angle_1.data/2.0/M_PI*360.0);
              ROS_INFO("Published angle HEBI a1 = %f",angle_2_msg.points[0].positions[0]/2.0/M_PI*360.0);
              ROS_INFO("Published angle Dynamixel a3 = %f || a1 = %f",angle_3_dyn.data/2.0/M_PI*360.0,angle_1_dyn.data/2.0/M_PI*360.0);

            }

            if(inverse_kin) {
              Eigen::Vector3f r_des(ik_testing_position_x,ik_testing_position_y,ik_testing_position_z);
              ROS_INFO("Input coordinates to reach x = %f || y = %f || z = %f",r_des(0),r_des(1),r_des(2));
              Eigen::Vector3f r_init(ik_initial_angle_3,ik_initial_angle_2,ik_initial_angle_1);
              ROS_INFO("Input initial angles a1 = %f || a2 = %f || a3 = %f",r_init(0),r_init(1),r_init(2));

              ROS_INFO("Inverse Kinematics");
              Eigen::Vector3f q = inverse_kinematics(r_des,r_init,0.001);
              ROS_INFO("Found angles in rad a3 = %f || a2 = %f || a1 = %f",q(0),q(1),q(2));
              ROS_INFO("Found angles in deg a3 = %f || a2 = %f || a1 = %f",q(0)/2.0/M_PI*360.0,q(1)/2.0/M_PI*360.0,q(2)/2.0/M_PI*360.0);


              ROS_INFO("Now publishing joint commands");
              // ros::Duration(5).sleep();

              // Create message from value
              std_msgs::Float64 ik_angle_3;
              std_msgs::Float64 ik_angle_2;
              std_msgs::Float64 ik_angle_1;

              //  HEBI
              trajectory_msgs::JointTrajectory angle_2_msg;
              angle_2_msg.joint_names.resize(1);
              angle_2_msg.joint_names[0] = "X5-4/M1";
              angle_2_msg.points.resize(1);
              angle_2_msg.points[0].positions.push_back(-q(1));
              // Dynamixel
              std_msgs::Float64 angle_3_dyn;
              std_msgs::Float64 angle_1_dyn;
              angle_3_dyn.data = q(0)+M_PI;
              angle_1_dyn.data =  q(2)+M_PI;


              ik_angle_3.data = q(0);
              ik_angle_2.data = q(1);
              ik_angle_1.data = q(2);

              // Publish
              pub_cmd_3_.publish(ik_angle_3);
              pub_cmd_2_.publish(ik_angle_2);
              pub_cmd_1_.publish(ik_angle_1);

              pub_cmd_3_dynamixel_.publish(angle_3_dyn);
              pub_cmd_2_hebi_.publish(angle_2_msg);
              pub_cmd_1_dynamixel_.publish(angle_1_dyn);

              ROS_INFO("IK published commands a3 = %f || a2 = %f || a1 = %f",ik_angle_3.data/2.0/M_PI*360.0,ik_angle_2.data/2.0/M_PI*360.0,ik_angle_1.data/2.0/M_PI*360.0);
            }

            if(homeing_state) {

              // Create message
              std_msgs::Float64 home_angle_3;
              std_msgs::Float64 home_angle_2;
              std_msgs::Float64 home_angle_1;

              home_angle_3.data = -3.0*M_PI/4.0;
              home_angle_2.data = -3.0*M_PI/4.0;
              home_angle_1.data = M_PI;

              trajectory_msgs::JointTrajectory angle_2_msg;
              angle_2_msg.joint_names.resize(1);
              angle_2_msg.joint_names[0] = "X5-4/M1";
              angle_2_msg.points.resize(1);
              // neg
              angle_2_msg.points[0].positions.push_back(-(-3.0*M_PI/4.0));
              // Dynamixel
              std_msgs::Float64 angle_3_dyn;
              std_msgs::Float64 angle_1_dyn;
              angle_3_dyn.data = -3.0*M_PI/4.0+M_PI;
              angle_1_dyn.data =  M_PI+M_PI;





              // Publish
              pub_cmd_3_.publish(home_angle_3);
              pub_cmd_2_.publish(home_angle_2);
              pub_cmd_1_.publish(home_angle_1);

              pub_cmd_3_dynamixel_.publish(angle_3_dyn);
              pub_cmd_2_hebi_.publish(angle_2_msg);
              pub_cmd_1_dynamixel_.publish(angle_1_dyn);

              ROS_INFO("Homeing state published commands a3 = %f || a2 = %f || a1 = %f",home_angle_3.data/2.0/M_PI*360.0,home_angle_2.data/2.0/M_PI*360.0,home_angle_1.data/2.0/M_PI*360.0);


            }

            if(testbench_settings) {
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

Eigen::Matrix4f ControllerProcessor::T_world_to_dynamixel() {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0,3) = base_position_(0);
  T(1,3) = base_position_(1);
  T(2,3) = base_position_(2);

  return T;
}

Eigen::Matrix4f ControllerProcessor::T_dynamixel_to_hebi(const double& a3) {
  double a3_offset = 45.0/360.0*2.0*M_PI;

  Eigen::Matrix4f T;
  T << cos(a3-a3_offset), -sin(a3-a3_offset), 0, 0.0325,
      sin(a3-a3_offset), cos(a3-a3_offset), 0, 0,
      0, 0, 1, 0.072,
      0, 0, 0, 1;

  return T;
}

Eigen::Matrix4f ControllerProcessor::T_dynamixel_to_hebi_clamp2(const double& a2) {
  double a2_offset = 112.5/360.0*2.0*M_PI;
  double a2_fix = -90.0/360.0*2.0*M_PI;
  Eigen::Matrix3f Rx;
    Rx << 1, 0, 0,
        0, cos(-a2+a2_offset), -sin(-a2+a2_offset),
        0, sin(-a2+a2_offset), cos(-a2+a2_offset);

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
  double a1_offset = -225.0/360.0*2.0*M_PI;
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
  T = T_world_to_dynamixel()*T_dynamixel_to_hebi(a3)*T_dynamixel_to_hebi_clamp2(a2)*T_clamp2_clamp1(a1)*T_clamp1_tip();
  return T;

}

Eigen::Matrix4f ControllerProcessor::T_dynamixel_to_tip(const double& a3,const double& a2,const double& a1) {
  Eigen::Matrix4f T;
  T = T_dynamixel_to_hebi(a3)*T_dynamixel_to_hebi_clamp2(a2)*T_clamp2_clamp1(a1)*T_clamp1_tip();
  return T;
}

Eigen::Matrix3f ControllerProcessor::Joint_to_rotation_mat(const double& input3, const double& input2, const double& input1) {
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

Eigen::Vector3f ControllerProcessor::Joint_to_position(const double& input3,const double& input2, const double& input1) {

  Eigen::Matrix4f T;
  T = T_world_tip(input3,input2,input1);
  Eigen::Vector3f pos;

  pos(0) = T(0,3);
  pos(1) = T(1,3);
  pos(2) = T(2,3);


  return pos;
}

Eigen::Quaternionf ControllerProcessor::Joint_to_quaternion(const double& input3, const double& input2, const double& input1) {
  Eigen::Matrix3f Rot;
  Rot = Joint_to_rotation_mat(input3,input2,input1);

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

Eigen::Matrix3f ControllerProcessor::Joint_to_position_jacobian(const double& input3, const double& input2, const double& input1) {
  Eigen::Matrix4f T00 = T_world_to_dynamixel();
  Eigen::Matrix4f T01 = T_dynamixel_to_hebi(input3);
  Eigen::Matrix4f T12 = T_dynamixel_to_hebi_clamp2(input2);
  Eigen::Matrix4f T23 = T_clamp2_clamp1(input1);
  Eigen::Matrix4f T34 = T_clamp1_tip();

  Eigen::Matrix4f TW1 = T00*T01;
  Eigen::Matrix4f TW2 = TW1*T12;
  Eigen::Matrix4f TW3 = TW2*T23;
  Eigen::Matrix4f TW4 = TW3*T34;

  Eigen::Matrix3f RW1 = TW1.block(0,0,3,3);
  Eigen::Matrix3f RW2 = TW2.block(0,0,3,3);
  Eigen::Matrix3f RW3 = TW3.block(0,0,3,3);
  // Eigen::Matrix3f RW4 = TW4.block(0,0,3,3);


  Eigen::Vector3f r_W1 = TW1.block(0,3,3,1);
  Eigen::Vector3f r_W2 = TW2.block(0,3,3,1);
  Eigen::Vector3f r_W3 = TW3.block(0,3,3,1);
  // Eigen::Vector3f r_W4 = TW4.block(0,3,3,1);


  Eigen::Vector3f omega_hat_1(0.0, 0.0, 1.0);

  Eigen::Vector3f omega_hat_2(1.0, 0.0, 0.0);

  Eigen::Vector3f omega_hat_3(0.0, 0.0, 1.0);

  Eigen::Vector3f r_Wtip = Joint_to_position(input3,input2,input1);

  Eigen::Vector3f Resultvector_1 = (RW1*omega_hat_1).cross(r_Wtip-r_W1);
  Eigen::Vector3f Resultvector_2 = (RW2*omega_hat_2).cross(r_Wtip-r_W2);
  Eigen::Vector3f Resultvector_3 = (RW3*omega_hat_3).cross(r_Wtip-r_W3);


  Eigen::Matrix3f J;
  J << Resultvector_1(0),Resultvector_2(0),Resultvector_3(0),
        Resultvector_1(1),Resultvector_2(1),Resultvector_3(1),
        Resultvector_1(2),Resultvector_2(2),Resultvector_3(2);



  return J;

}

Eigen::Matrix3f ControllerProcessor::Joint_to_rotation_jacobian(const double& input3, const double& input2, const double& input1) {
  Eigen::Matrix4f T00 = T_world_to_dynamixel();
  Eigen::Matrix4f T01 = T_dynamixel_to_hebi(input3);
  Eigen::Matrix4f T12 = T_dynamixel_to_hebi_clamp2(input2);
  Eigen::Matrix4f T23 = T_clamp2_clamp1(input1);
  Eigen::Matrix4f T34 = T_clamp1_tip();

  Eigen::Matrix4f TW1 = T00*T01;
  Eigen::Matrix4f TW2 = TW1*T12;
  Eigen::Matrix4f TW3 = TW2*T23;
  Eigen::Matrix4f TW4 = TW3*T34;

  Eigen::Matrix3f RW1 = TW1.block(0,0,3,3);
  Eigen::Matrix3f RW2 = TW2.block(0,0,3,3);
  Eigen::Matrix3f RW3 = TW3.block(0,0,3,3);
  // Eigen::Matrix3f RW4 = TW4.block(0,0,3,3);


  Eigen::Vector3f omega_hat_1(0.0, 0.0, 1.0);
  Eigen::Vector3f omega_hat_2(1.0, 0.0, 0.0);
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

Eigen::Vector3f ControllerProcessor::inverse_kinematics(const Eigen::Vector3f& r_des,const Eigen::Vector3f& r_init,const float& epsilon) {

  float iterations = 0;
  float max_iterations = 100;
  float lambda = 0.001; // Damping factor
  float alpha = 0.05; // Update rate

  Eigen::Vector3f q = r_init; // current
  // ROS_INFO("q start %f %f %f",q(0),q(1),q(2));
  Eigen::Vector3f dr(2,2,2); // initialize error
  // ROS_INFO("dr %f %f %f",dr(0),dr(1),dr(2));
  Eigen::Matrix3f I_J;
  Eigen::Matrix3f I_J_pinv;
  Eigen::Vector3f r_current;
  Eigen::Vector3f temp;

  while(iterations==0 || dr.norm()>epsilon && iterations < max_iterations) {
      I_J = Joint_to_position_jacobian(q(0),q(1),q(2)); // Evaluate current jacobians
      // ROS_INFO("I_J");
      // ROS_INFO("%f || %f || %f",I_J(0,0),I_J(0,1),I_J(0,2));
      // ROS_INFO("%f || %f || %f",I_J(1,0),I_J(1,1),I_J(1,2));
      // ROS_INFO("%f || %f || %f",I_J(2,0),I_J(2,1),I_J(2,2));
      I_J_pinv = psuedoInverseMat(I_J,lambda); // Take the psuedo inverse
      // ROS_INFO("I_J_pinv");
      // ROS_INFO("%f || %f || %f",I_J_pinv(0,0),I_J_pinv(0,1),I_J_pinv(0,2));
      // ROS_INFO("%f || %f || %f",I_J_pinv(1,0),I_J_pinv(1,1),I_J_pinv(1,2));
      // ROS_INFO("%f || %f || %f",I_J_pinv(2,0),I_J_pinv(2,1),I_J_pinv(2,2));
      r_current = Joint_to_position(q(0),q(1),q(2)); // current position
      // ROS_INFO("r_current %f || %f || %f",r_current(0),r_current(1),r_current(2));
      dr = r_des-r_current; // position error
      // ROS_INFO("dr %f %f %f",dr(0),dr(1),dr(2));
      temp = alpha*I_J_pinv*dr;

      q = q+temp; // Update generalized coordinates

      iterations = iterations+1;
    }
    r_current = Joint_to_position(q(0),q(1),q(2));
    dr = r_des-r_current;
    ROS_INFO("Inverse Kinematics terminated after %f iterations",iterations);
    ROS_INFO("Position error %f",dr.norm());
    return q;
}

Eigen::Matrix3f ControllerProcessor::psuedoInverseMat(const Eigen::Matrix3f& A,const float& lambda) {

    Eigen::Matrix3f identiymatrix = Eigen::Matrix3f::Identity();
    // ROS_INFO("A");
    // ROS_INFO("%f || %f || %f",A(0,0),A(0,1),A(0,2));
    // ROS_INFO("%f || %f || %f",A(1,0),A(1,1),A(1,2));
    // ROS_INFO("%f || %f || %f",A(2,0),A(2,1),A(2,2));
    Eigen::Matrix3f A_transposed = A.transpose();
    // ROS_INFO("A_transposed");
    // ROS_INFO("%f || %f || %f",A_transposed(0,0),A_transposed(0,1),A_transposed(0,2));
    // ROS_INFO("%f || %f || %f",A_transposed(1,0),A_transposed(1,1),A_transposed(1,2));
    // ROS_INFO("%f || %f || %f",A_transposed(2,0),A_transposed(2,1),A_transposed(2,2));
    Eigen::Matrix3f temp_1 = lambda*lambda*identiymatrix;
    // ROS_INFO("temp_1");
    // ROS_INFO("%f || %f || %f",temp_1(0,0),temp_1(0,1),temp_1(0,2));
    // ROS_INFO("%f || %f || %f",temp_1(1,0),temp_1(1,1),temp_1(1,2));
    // ROS_INFO("%f || %f || %f",temp_1(2,0),temp_1(2,1),temp_1(2,2));
    Eigen::Matrix3f temp_2 = A*A_transposed+temp_1;
    // ROS_INFO("temp_2");
    // ROS_INFO("%f || %f || %f",temp_2(0,0),temp_2(0,1),temp_2(0,2));
    // ROS_INFO("%f || %f || %f",temp_2(1,0),temp_2(1,1),temp_2(1,2));
    // ROS_INFO("%f || %f || %f",temp_2(2,0),temp_2(2,1),temp_2(2,2));

    // Eigen::Matrix3d pinv = temp_2.jacobiSvd().solve(A_transposed);
    // ROS_INFO("SVD inverse");
    // ROS_INFO("%f || %f || %f",pinv(0,0),pinv(0,1),pinv(0,2));
    // ROS_INFO("%f || %f || %f",pinv(1,0),pinv(1,1),pinv(1,2));
    // ROS_INFO("%f || %f || %f",pinv(2,0),pinv(2,1),pinv(2,2));
    //
    // Eigen::Matrix3d pinv5 = temp_2.partialPivLu().solve(A_transposed);
    // ROS_INFO("partialPivLu inverse");
    // ROS_INFO("%f || %f || %f",pinv5(0,0),pinv5(0,1),pinv5(0,2));
    // ROS_INFO("%f || %f || %f",pinv5(1,0),pinv5(1,1),pinv5(1,2));
    // ROS_INFO("%f || %f || %f",pinv5(2,0),pinv5(2,1),pinv5(2,2));
    //
    // Eigen::Matrix3d pinv6 = temp_2.fullPivLu().solve(A_transposed);
    // ROS_INFO("fullPivLu inverse");
    // ROS_INFO("%f || %f || %f",pinv6(0,0),pinv6(0,1),pinv6(0,2));
    // ROS_INFO("%f || %f || %f",pinv6(1,0),pinv6(1,1),pinv6(1,2));
    // ROS_INFO("%f || %f || %f",pinv6(2,0),pinv6(2,1),pinv6(2,2));
    //
    // Eigen::Matrix3d pinv7 = temp_2.householderQr().solve(A_transposed);
    // ROS_INFO("householderQr inverse");
    // ROS_INFO("%f || %f || %f",pinv7(0,0),pinv7(0,1),pinv7(0,2));
    // ROS_INFO("%f || %f || %f",pinv7(1,0),pinv7(1,1),pinv7(1,2));
    // ROS_INFO("%f || %f || %f",pinv7(2,0),pinv7(2,1),pinv7(2,2));
    //
    // Eigen::Matrix3d pinv1 = temp_2.colPivHouseholderQr().solve(A_transposed);
    // ROS_INFO("colPivHouseholderQr inverse");
    // ROS_INFO("%f || %f || %f",pinv1(0,0),pinv1(0,1),pinv1(0,2));
    // ROS_INFO("%f || %f || %f",pinv1(1,0),pinv1(1,1),pinv1(1,2));
    // ROS_INFO("%f || %f || %f",pinv1(2,0),pinv1(2,1),pinv1(2,2));
    //
    // Eigen::Matrix3d pinv2 = temp_2.fullPivHouseholderQr().solve(A_transposed);
    // ROS_INFO("fullPivHouseholderQr inverse");
    // ROS_INFO("%f || %f || %f",pinv2(0,0),pinv2(0,1),pinv2(0,2));
    // ROS_INFO("%f || %f || %f",pinv2(1,0),pinv2(1,1),pinv2(1,2));
    // ROS_INFO("%f || %f || %f",pinv2(2,0),pinv2(2,1),pinv2(2,2));
    //
    // Eigen::Matrix3d pinv3 = temp_2.llt().solve(A_transposed);
    // ROS_INFO("llt inverse");
    // ROS_INFO("%f || %f || %f",pinv3(0,0),pinv3(0,1),pinv3(0,2));
    // ROS_INFO("%f || %f || %f",pinv3(1,0),pinv3(1,1),pinv3(1,2));
    // ROS_INFO("%f || %f || %f",pinv3(2,0),pinv3(2,1),pinv3(2,2));
    //
    // Eigen::Matrix3d pinv8 = temp_2.ldlt().solve(A_transposed);
    // ROS_INFO("ldlt inverse");
    // ROS_INFO("%f || %f || %f",pinv8(0,0),pinv8(0,1),pinv8(0,2));
    // ROS_INFO("%f || %f || %f",pinv8(1,0),pinv8(1,1),pinv8(1,2));
    // ROS_INFO("%f || %f || %f",pinv8(2,0),pinv8(2,1),pinv8(2,2));

    Eigen::Matrix3f pinv43 = temp_2.inverse()*A;
    // ROS_INFO("inverse inverse");
    // ROS_INFO("%f || %f || %f",pinv43(0,0),pinv43(0,1),pinv43(0,2));
    // ROS_INFO("%f || %f || %f",pinv43(1,0),pinv43(1,1),pinv43(1,2));
    // ROS_INFO("%f || %f || %f",pinv43(2,0),pinv43(2,1),pinv43(2,2));

    return pinv43;
  }












// Median filter with 3 elements
float ControllerProcessor::median_n_3(const float& a,const float& b,const float& c) {
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






// Callack VICON
void ControllerProcessor::Callback_tip_position(const geometry_msgs::TransformStamped& point_msg){


  float x = point_msg.transform.translation.x;
  float y = point_msg.transform.translation.y;
  float z = point_msg.transform.translation.z;

  float ik_initial_angle_3 = 5*2.0*M_PI/360.0;
  float ik_initial_angle_2 = -30*2.0*M_PI/360.0;
  float ik_initial_angle_1 = 100*2.0*M_PI/360.0;

  Eigen::Vector3f r_des(x,y,z);
  ROS_INFO("Input coordinates to reach x = %f || y = %f || z = %f",r_des(0),r_des(1),r_des(2));
  Eigen::Vector3f r_init(ik_initial_angle_3,ik_initial_angle_2,ik_initial_angle_1);
  ROS_INFO("Input initial angles a1 = %f || a2 = %f || a3 = %f",r_init(0),r_init(1),r_init(2));

  ROS_INFO("Inverse Kinematics");
  Eigen::Vector3f q = inverse_kinematics(r_des,r_init,0.01);
  ROS_INFO("Found angles in rad a3 = %f || a2 = %f || a1 = %f",q(0),q(1),q(2));
  ROS_INFO("Found angles in deg a3 = %f || a2 = %f || a1 = %f",q(0)/2.0/M_PI*360.0,q(1)/2.0/M_PI*360.0,q(2)/2.0/M_PI*360.0);


  ROS_INFO("Now publishing joint commands");
  // ros::Duration(5).sleep();

  // Create message from value
  std_msgs::Float64 ik_angle_3;
  std_msgs::Float64 ik_angle_2;
  std_msgs::Float64 ik_angle_1;

  //  HEBI
  trajectory_msgs::JointTrajectory angle_2_msg;
  angle_2_msg.joint_names.resize(1);
  angle_2_msg.joint_names[0] = "X5-4/M1";
  angle_2_msg.points.resize(1);
  angle_2_msg.points[0].positions.push_back(-q(1));
  // Dynamixel
  std_msgs::Float64 angle_3_dyn;
  std_msgs::Float64 angle_1_dyn;
  angle_3_dyn.data = q(0)+M_PI;
  angle_1_dyn.data =  q(2)+M_PI;


  ik_angle_3.data = q(0);
  ik_angle_2.data = q(1);
  ik_angle_1.data = q(2);

  if(is_wand_) {
    // Publish
    pub_cmd_3_.publish(ik_angle_3);
    pub_cmd_2_.publish(ik_angle_2);
    pub_cmd_1_.publish(ik_angle_1);

    pub_cmd_3_dynamixel_.publish(angle_3_dyn);
    pub_cmd_2_hebi_.publish(angle_2_msg);
    pub_cmd_1_dynamixel_.publish(angle_1_dyn);

    ROS_INFO("IK published commands a3 = %f || a2 = %f || a1 = %f",ik_angle_3.data/2.0/M_PI*360.0,ik_angle_2.data/2.0/M_PI*360.0,ik_angle_1.data/2.0/M_PI*360.0);
  }


}

//Callback for Encoder 1
void ControllerProcessor::CallbackEnc1(const geometry_msgs::PointStamped &pt_s_1) {
  ROS_DEBUG("Received message form encoder 1");

  // Create valiables for subscribed values
  float pulsewidth_e1 = pt_s_1.point.x;
  float period_e1 = pt_s_1.point.y;
  float dutycycle_e1 = pt_s_1.point.z;
  float uc = 1000000;

  // Calculate andle from pulsewidth
  float angle_deg_a1 = ((pulsewidth_e1*uc*4098/(period_e1*uc))-1)*360/4096;

  // Store the last 3 angles of encoder
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
              offset_calculated_1 = true;
      }
    }



  // Calculate angle difference
  float angle_diff_a_1_ = encoder_angle_1_initial_-median_val_e1;

  // Calculate torque estimate
  t_est_1_ = angle_diff_a_1_*k_1_;
  // ROS_INFO("Torque estimate in Nm = [%f]",t_est_1_);

  // Create message from value
  std_msgs::Float64 t_est_1_msg;
  std_msgs::Float64 angle_1_deg_filtered_o_msg;

  t_est_1_msg.data = t_est_1_;


  // Publish
  pub_torque_1_.publish(t_est_1_msg);



} // namespace Callback 1

//Callback HEBI
void ControllerProcessor::CallbackHebi2(const sensor_msgs::JointState& state_msg){


  // Create message from value
  std_msgs::Float64 t_2_msg;


  t_2_msg.data = state_msg.effort[0];


  // Publish
  pub_torque_2_.publish(t_2_msg);

}

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

  // Store the last 3 angles of encoder
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
            offset_calculated_3 = true;
    }
  }





  // Calculate angle difference
  float angle_diff_a_3_ = encoder_angle_3_initial_-median_val_e3;


  // Calculate torque estimate
  float t_est_3_ = angle_diff_a_3_*k_3_;

  // Create message from value
  std_msgs::Float64 t_est_3_msg;


  t_est_3_msg.data = t_est_3_;


  // Publish
  pub_torque_3_.publish(t_est_3_msg);




} // namespace callback 3











} // namespace controller
