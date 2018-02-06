//
// Created by marius on 16.04.16.
//

#include "dynamixel_controller/controller_processor.h"

namespace controller {

void InitializeParameters(const ros::NodeHandle& nh, ParameterBag* parameter) {
  // Retrieve all parameters or set to default
  nh.param("sub_rostopic_enc_1", parameter->sub_rostopic_enc_1,
           kDefaultSubTopic_1);
  nh.param("queue_size_sub_enc_1", parameter->queue_size_sub_enc_1,
           kDefaultSubQueueSize_1);
  nh.param("sub_rostopic_enc_3", parameter->sub_rostopic_enc_3,
           kDefaultSubTopic_3);
  nh.param("queue_size_sub_enc_3", parameter->queue_size_sub_enc_3,
           kDefaultSubQueueSize_3);
  nh.param("pub_rostopic_command_1", parameter->pub_rostopic_command_1,
           kDefaultObjectsPubTopic_1);
  nh.param("queue_size_pub_command_1", parameter->queue_size_pub_command_1,
           kDefaultObjectsPubQueueSize_1);
  nh.param("pub_rostopic_command_3", parameter->pub_rostopic_command_3,
           kDefaultObjectsPubTopic_3);
  nh.param("queue_size_pub_command_3", parameter->queue_size_pub_command_3,
           kDefaultObjectsPubQueueSize_3);
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
  ROS_INFO("Spinning node");
  return 0;
}
