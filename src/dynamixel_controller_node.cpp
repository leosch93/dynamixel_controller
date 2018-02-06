//
// Created by marius on 16.04.16.
//

#include "dynamixel_controller/controller_processor.h"

namespace controller {

void InitializeParameters(const ros::NodeHandle& nh, ParameterBag* parameter) {
  // Retrieve all parameters or set to default
  nh.param("subscribed_rostopic_enc_1", parameter->sub_rostopic_enc_1,
           kDefaultImageSubTopic);
  nh.param("queue_size_subscriber_img", parameter->queue_size_sub_enc_1,
           kDefaultImageSubQueueSize);
  nh.param("subscribed_rostopic_cam", parameter->sub_rostopic_enc_3,
           kDefaultCamSubTopic);
  nh.param("queue_size_subscriber_cam", parameter->queue_size_sub_enc_3,
           kDefaultCamSubQueueSize);
  nh.param("pub_rostopic_objects", parameter->pub_rostopic_command_1,
           kDefaultObjectsPubTopic);
  nh.param("queue_size_pub_objects", parameter->queue_size_pub_command_1,
           kDefaultObjectsPubQueueSize);
  nh.param("pub_rostopic_img", parameter->pub_rostopic_command_3,
           kDefaultObjectsPubTopic);
  nh.param("queue_size_pub_img", parameter->queue_size_pub_command_3,
           kDefaultObjectsPubQueueSize);
}

} // namespace controller

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "dynamixel_controller_node");
  ros::NodeHandle nh;

  // Initialize parameter structure
  controller::ParameterBag parameter;
  controller::InitializeParameters(nh, &parameter);

  // Construct class detection_processor with NodeHandle and parameter structure
  controller::ControllerProcessor controller(nh, parameter);

  // Relative path to package
  std::string path = ros::package::getPath("dynamixel_controller");

  // Spin
  ros::spin ();
}
