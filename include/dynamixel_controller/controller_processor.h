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




namespace controller {

// Default values
static const std::string kDefaultImageSubTopic   = "/camX/image_raw";
static const std::string kDefaultCamSubTopic     = "/camera_info";
static const std::string kDefaultObjectsPubTopic = "detection_result";

static constexpr int kDefaultImageSubQueueSize   = 1;
static constexpr int kDefaultCamSubQueueSize     = 1;
static constexpr int kDefaultObjectsPubQueueSize = 500;

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
  void CallbackImg(const sensor_msgs::ImageConstPtr& a_image_msg);

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
  ros::Publisher pub_cmd_3_;

};

} // namespace controller

#endif //DYNAMIXEL_CONTROLLER_CONTROLLER_PROCESSOR_H
