//
// Created by marius on 16.04.16.
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

#include <iostream>
#include <vector>

#include "dynamixel_controller/parameter/parameter_bag.h"

#include <dynamic_reconfigure/server.h>
#include <dynamixel_controller/controllerConfig.h>

namespace controller {

// Default values
static const std::string kDefaultSubTopic_1   = "/default_sub_1";
static const std::string kDefaultSubTopic_3     = "/default_sub_3";
static const std::string kDefaultObjectsPubTopic_1 = "/default_pub1";
static const std::string kDefaultObjectsPubTopic_2 = "/default_pub2";
static const std::string kDefaultObjectsPubTopic_3 = "/default_pub3";

static const std::string kDefaultPubTopic_1 = "/default_pub1";
static const std::string kDefaultPubTopic_2 = "/default_pub2";
static const std::string kDefaultPubTopic_3 = "/default_pub3";
static const std::string kDefaultPubTopic_4 = "/default_pub4";


static constexpr int kDefaultSubQueueSize_1   = 1;
static constexpr int kDefaultSubQueueSize_3     = 1;
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
   * \brief Callback for the image.
   * @param [in] img_msg Ros message storing the image data.
   */
  void CallbackEnc1(const geometry_msgs::PointStamped& pt_s);
  void CallbackEnc3(const geometry_msgs::PointStamped& pt_s);

  void ConfigCallback(dynamixel_controller::controllerConfig &config, uint32_t level);

 private:
  /** \brief Ros NodeHandle. */
  ros::NodeHandle nh_;

  /**
   * \brief Struct storing the relevant parameters for the processor and the tree
   *        structure of parameter bags for other modules.
   */
  ParameterBag parameter_;

  /** \brief Ros Subscriber for the camera. */
  ros::Subscriber sub_enc_1_;
  ros::Subscriber sub_enc_3_;

  /** \brief Ros Publisher for the objects. */
  ros::Publisher pub_cmd_1_;
  ros::Publisher pub_cmd_2_;
  ros::Publisher pub_cmd_3_;

  /** \brief Ros Publisher for the objects. */
  ros::Publisher pub_angle_1_;
  ros::Publisher pub_angle_3_;
  ros::Publisher pub_angle_1_filtered_;
  ros::Publisher pub_angle_3_filtered_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<dynamixel_controller::controllerConfig> server_;

  float angle_val_e1_1_{};
  float angle_val_e1_2_{};
  float angle_val_e1_3_{};

  float angle_val_e3_1_{};
  float angle_val_e3_2_{};
  float angle_val_e3_3_{};

};

} // namespace controller

#endif //DYNAMIXEL_CONTROLLER_CONTROLLER_PROCESSOR_H
