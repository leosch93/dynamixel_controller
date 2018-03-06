//
// Created by Leo on 16.02.16.
//

#ifndef DYNAMIXEL_CONTROLLER_CONTROLLER_PROCESSOR_H
#define DYNAMIXEL_CONTROLLER_CONTROLLER_PROCESSOR_H

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <ros/time.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamixel_msgs/JointState.h>

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
static const std::string kDefaultSubTopic_3_dyn    = "/default_sub_3_dyn";

static const std::string kDefaultObjectsPubTopic_1 = "/default_pub1";
static const std::string kDefaultObjectsPubTopic_2 = "/default_pub2";
static const std::string kDefaultObjectsPubTopic_3 = "/default_pub3";

static const std::string kDefaultPubTopic_1 = "/default_pub1";
static const std::string kDefaultPubTopic_2 = "/default_pub2";
static const std::string kDefaultPubTopic_3 = "/default_pub3";
static const std::string kDefaultPubTopic_4 = "/default_pub4";


static constexpr int kDefaultSubQueueSize_1   = 1;
static constexpr int kDefaultSubQueueSize_3   = 1;

static constexpr int kDefaultSubQueueSize_1_dyn  = 1;
static constexpr int kDefaultSubQueueSize_3_dyn  = 1;

static constexpr int kDefaultObjectsPubQueueSize_1 = 1;
static constexpr int kDefaultObjectsPubQueueSize_2 = 1;
static constexpr int kDefaultObjectsPubQueueSize_3 = 1;

static constexpr int kDefaultPubQueueSize_1 = 1;
static constexpr int kDefaultPubQueueSize_2 = 1;
static constexpr int kDefaultPubQueueSize_3 = 1;
static constexpr int kDefaultPubQueueSize_4 = 1;

/**
 * \brief Processor which organises the subscription and publishing of the data.
 * @author Leonard Schai (lschai@student.ethz.ch)
 * @date February, 2017
 */
class ControllerProcessor {
 public:
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

  void CallbackDyn1(const dynamixel_msgs::JointState& dyn_state_1);
  void CallbackDyn3(const dynamixel_msgs::JointState& dyn_state_3);

  void CallbackEnc1(const geometry_msgs::PointStamped& pt_s_1);
  void CallbackEnc3(const geometry_msgs::PointStamped& pt_s_3);



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
  ros::Subscriber sub_dyn_3_;

  /** \brief Ros Publisher for the commands. */
  ros::Publisher pub_cmd_1_;
  ros::Publisher pub_cmd_2_;
  ros::Publisher pub_cmd_3_;

  /** \brief Ros Publisher for the calculated angles. */
  ros::Publisher pub_angle_1_;
  ros::Publisher pub_angle_3_;
  ros::Publisher pub_angle_1_filtered_;
  ros::Publisher pub_angle_3_filtered_;

 /** Global variables */
  float encoder_angle_1_initial_{};
  float encoder_angle_3_initial_{};

  float dynam_state_1_initial_{};
  float dynam_state_3_initial_{};

  float offset_angle_a_1_{};
  float offset_angle_a_3_{};

  float enc_1_angle_filt_offset_{};
  float enc_3_angle_filt_offset_{};


  // Dynamic reconfigure
  dynamic_reconfigure::Server<dynamixel_controller::controllerConfig> server_;

  float angle_val_e1_1_{};
  float angle_val_e1_2_{};
  float angle_val_e1_3_{};

  float angle_val_e3_1_{};
  float angle_val_e3_2_{};
  float angle_val_e3_3_{};

  //ros::Time start_;
  bool only_once_enc_1_1_ = true;
  bool only_once_enc_1_2_ = true;
  bool only_once_enc_1_3_ = true;

  bool only_once_enc_3_1_ = true;
  bool only_once_enc_3_2_ = true;
  bool only_once_enc_3_3_ = true;

  bool offset_calculated_1 = false;
  bool offset_calculated_3 = false;

  bool only_once_dyn_1_ = true;
  bool only_once_dyn_3_ = true;



};

} // namespace controller

#endif //DYNAMIXEL_CONTROLLER_CONTROLLER_PROCESSOR_H
