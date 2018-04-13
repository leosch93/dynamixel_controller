//
// Created by Leo on 16.02.16.
//
#ifndef DYNAMIXEL_CONTROLLER_CONTROLLER_PROCESSOR_H
#define DYNAMIXEL_CONTROLLER_CONTROLLER_PROCESSOR_H

#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamixel_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "tf/tf.h"
#include <algorithm>
#include <cmath>
#include <angles/angles.h>
#include <tf2_ros/transform_listener.h>
#include "tf/transform_listener.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <eigen3/Eigen/Dense>
// #include "matlab.hpp"

#include <iostream>
#include <vector>

#include "dynamixel_controller/parameter/parameter_bag.h"

#include <dynamic_reconfigure/server.h>
#include <dynamixel_controller/controllerConfig.h>

namespace controller {

// Default values
static const std::string kDefaultSubTopic_1   = "/default_sub_1";
static const std::string kDefaultSubTopic_3     = "/default_sub_3";

static const std::string kDefaultSubTopic_1_dyn   = "/default_sub_1_dyn";
static const std::string kDefaultSubTopic_2_hebi    = "/default_sub_2_hebi";
static const std::string kDefaultSubTopic_3_dyn    = "/default_sub_3_dyn";

static const std::string kDefaultSubTopic_tip_position    = "/default_sub_tip_position";



static const std::string kDefaultObjectsPubTopic_1 = "/default_pub1";
static const std::string kDefaultObjectsPubTopic_1_dynamixel = "/default_pub1_dyn";
static const std::string kDefaultObjectsPubTopic_2 = "/default_pub2";
static const std::string kDefaultObjectsPubTopic_2_hebi = "/default_pub2_hebi";
static const std::string kDefaultObjectsPubTopic_3 = "/default_pub3";
static const std::string kDefaultObjectsPubTopic_3_dynamixel = "/default_pub3_dyn";



static constexpr int kDefaultSubQueueSize_1   = 1;
static constexpr int kDefaultSubQueueSize_3   = 1;

static constexpr int kDefaultSubQueueSize_1_dyn  = 1;
static constexpr int kDefaultSubQueueSize_2_hebi  = 1;
static constexpr int kDefaultSubQueueSize_3_dyn  = 1;

static constexpr int kDefaultSubQueueSize_tip_pos  = 1;


static constexpr int kDefaultObjectsPubQueueSize_1 = 1;
static constexpr int kDefaultObjectsPubQueueSize_1_dyn = 1;
static constexpr int kDefaultObjectsPubQueueSize_2 = 1;
static constexpr int kDefaultObjectsPubQueueSize_2_hebi = 1;
static constexpr int kDefaultObjectsPubQueueSize_3 = 1;
static constexpr int kDefaultObjectsPubQueueSize_3_dyn = 1;


// Torque pub
static const std::string kDefaultPubTopic_1 = "/default_pub1";
static const std::string kDefaultPubTopic_2 = "/default_pub2";
static const std::string kDefaultPubTopic_3 = "/default_pub3";

static constexpr int kDefaultPubQueueSize_1 = 1;
static constexpr int kDefaultPubQueueSize_2 = 1;
static constexpr int kDefaultPubQueueSize_3 = 1;


/**
 * \brief Processor which organises the subscription and publishing of the data.
 * @author Leonard Schai (lschai@student.ethz.ch)
 * @date February, 2018
 */
class ControllerProcessor {
 public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * \brief The Constructor.
   * @param [in] nh Ros NodeHandle.
   * @param [in] parameter Struct storing relevant parameters for the processor
   *                       and the tree structure of parameter bags for the
   *                       individual modules.
   */
  ControllerProcessor(const ros::NodeHandle& nh, const ParameterBag& parameter);

  /** \brief The Destructor. */
  virtual ~ControllerProcessor() = default;

  /**
   * \brief Callback for the encoder data.
   * @param [in] img_msg Ros message storing the encoder data.
   */
  float median_n_3(const float& a,const float& b,const float& c);

  // void CallbackDyn1(const dynamixel_msgs::JointState& dyn_state_1);
  void CallbackHebi2(const sensor_msgs::JointState& state_msg);
  // void CallbackDyn3(const dynamixel_msgs::JointState& dyn_state_3);
  void Callback_tip_position(const geometry_msgs::TransformStamped& point_msg);

  void CallbackEnc1(const geometry_msgs::PointStamped& pt_s_1);
  void CallbackEnc3(const geometry_msgs::PointStamped& pt_s_3);

  // Eigen::MatrixXf linspace(const float& a,const float& b,const float& n);
  Eigen::Matrix4f T_world_to_dynamixel();
  Eigen::Matrix4f T_dynamixel_to_tip(const double& input3,const double& input2,const double& input1);
  Eigen::Matrix4f T_dynamixel_to_hebi(const double& a3);
  Eigen::Matrix4f T_dynamixel_to_hebi_clamp2(const double& a2);
  Eigen::Matrix4f T_clamp2_clamp1(const double& a1);
  Eigen::Matrix4f T_clamp1_tip();
  Eigen::Matrix4f T_world_tip(const double& input3,const double& input2,const double& input1);
  Eigen::Matrix3f Joint_to_rotation_mat(const double& input3, const double& input2, const double& input1);
  Eigen::Vector3f Joint_to_position(const double& input3, const double& input2, const double& input1);
  Eigen::Quaternionf Joint_to_quaternion(const double& input3, const double& input2, const double& input1);
  Eigen::Matrix3f Quaternion_to_rotation_mat(const Eigen::Quaternionf& q);
  Eigen::Quaternionf Multiply_quaternions(const Eigen::Quaternionf& q1,const Eigen::Quaternionf& q2);
  Eigen::Vector3f Rotate_vec_with_quaternion(const Eigen::Quaternionf& q,const Eigen::Vector3f& v);
  Eigen::Matrix3f Joint_to_position_jacobian(const double& input3, const double& input2, const double& input1);
  Eigen::Matrix3f Joint_to_rotation_jacobian(const double& input3, const double& input2, const double& input1);
  Eigen::Matrix3f SkewMatrix(const Eigen::Vector3f& v);
  Eigen::MatrixXf map_loc_rot_vel(const Eigen::Quaternionf& q);
  Eigen::Vector3f inverse_kinematics(const Eigen::Vector3f& r_des,const Eigen::Vector3f& r_init,const float& epsilon);
  Eigen::Matrix3f psuedoInverseMat(const Eigen::Matrix3f& A,const float& lambda);







  void ConfigCallback(dynamixel_controller::controllerConfig &config, uint32_t level);

 private:
  /** \brief Ros NodeHandle. */
  ros::NodeHandle nh_;

  /**
   * \brief Struct storing the relevant parameters for the processor and the tree
   *        structure of parameter bags for other modules.
   */
  ParameterBag parameter_;

  /** \brief Ros Subscriber for the encoder. */
  ros::Subscriber sub_enc_1_;
  ros::Subscriber sub_enc_3_;

  /** \brief Ros Subscriber for the dynamixel. */
  ros::Subscriber sub_dyn_1_;
  ros::Subscriber sub_hebi_2_;
  ros::Subscriber sub_dyn_3_;

  ros::Subscriber sub_tip_pos_;


  /** \brief Ros Publisher for the commands. */
  ros::Publisher pub_cmd_1_;
  ros::Publisher pub_cmd_1_dynamixel_;
  ros::Publisher pub_cmd_2_;
  ros::Publisher pub_cmd_2_hebi_;
  ros::Publisher pub_cmd_3_;
  ros::Publisher pub_cmd_3_dynamixel_;


  /** \brief Ros Publisher for the calculated angles. */
  ros::Publisher pub_torque_1_;
  ros::Publisher pub_torque_2_;
  ros::Publisher pub_torque_3_;


 /** Global variables */
  float encoder_angle_1_initial_{};
  float encoder_angle_3_initial_{};

  // float dynam_state_1_initial_{};
  // float dynam_state_3_initial_{};

  // float offset_angle_a_1_{};
  // float offset_angle_a_3_{};

  // float enc_1_angle_filt_offset_{};
  // float enc_3_angle_filt_offset_{};


  // Dynamic reconfigure
  dynamic_reconfigure::Server<dynamixel_controller::controllerConfig> server_;

  float angle_val_e1_1_{};
  float angle_val_e1_2_{};
  float angle_val_e1_3_{};

  float angle_val_e3_1_{};
  float angle_val_e3_2_{};
  float angle_val_e3_3_{};

  // float dyn_1_state_val_1_{};
  // float dyn_1_state_val_2_{};
  // float dyn_1_state_val_3_{};
  //
  // float dyn_3_state_val_1_{};
  // float dyn_3_state_val_2_{};
  // float dyn_3_state_val_3_{};

  bool is_wand_{};

  bool only_once_enc_1_1_ = true;
  bool only_once_enc_1_2_ = true;
  bool only_once_enc_1_3_ = true;

  bool only_once_enc_3_1_ = true;
  bool only_once_enc_3_2_ = true;
  bool only_once_enc_3_3_ = true;

  bool offset_calculated_1 = false;
  bool offset_calculated_3 = false;

  // bool only_once_dyn_1_ = true;
  // bool only_once_dyn_3_ = true;

  // float dynam_angle_1_{};
  // float dynam_angle_3_{};

  // float angle_diff_a_1_{};
  // float angle_diff_a_3_{};




  // FEM simulation values
  // float k_1_ = 0.5; //Nm/deg
  // float k_3_ = 0.5; //Nm/deg

  // Tested on testbench
  float k_1_ = 0.444; //Nm/deg
  float k_3_ = 0.444; //Nm/deg

  // Torque estimate
  float t_est_1_{};
  float t_est_3_{};

  Eigen::Vector3f base_position_;
};

} // namespace controller

#endif //DYNAMIXEL_CONTROLLER_CONTROLLER_PROCESSOR_H
