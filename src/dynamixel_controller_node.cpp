//
// Created by Leo on 16.02.17.
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

  nh.param("sub_rostopic_dynamixel_1", parameter->sub_rostopic_dynamixel_1,
          kDefaultSubTopic_1_dyn);
  nh.param("queue_size_sub_dynamixel_1", parameter->queue_size_sub_dynamixel_1,
          kDefaultSubQueueSize_1_dyn);
  nh.param("sub_rostopic_hebi_2", parameter->sub_rostopic_hebi_2,
          kDefaultSubTopic_2_hebi);
  nh.param("queue_size_sub_rostopic_hebi_2", parameter->queue_size_sub_rostopic_hebi_2,
          kDefaultSubQueueSize_2_hebi);
  nh.param("sub_rostopic_dynamixel_3", parameter->sub_rostopic_dynamixel_3,
          kDefaultSubTopic_3_dyn);
  nh.param("queue_size_sub_dynamixel_3", parameter->queue_size_sub_dynamixel_3,
          kDefaultSubQueueSize_3_dyn);

  // Vicon Tip
  nh.param("sub_rostopic_tip_position", parameter->sub_rostopic_tip_position,
          kDefaultSubTopic_tip_position);
  nh.param("queue_size_sub_rostopic_tip_position", parameter->queue_size_sub_rostopic_tip_position,
          kDefaultSubQueueSize_tip_pos);

  // Publishing Gazebo + Hardware

  nh.param("pub_rostopic_command_1", parameter->pub_rostopic_command_1,
           kDefaultObjectsPubTopic_1);
  nh.param("queue_size_pub_command_1", parameter->queue_size_pub_command_1,
           kDefaultObjectsPubQueueSize_1);
  nh.param("pub_rostopic_command_1_dynamixel", parameter->pub_rostopic_command_1_dynamixel,
           kDefaultObjectsPubTopic_1_dynamixel);
  nh.param("queue_size_pub_command_1_dynamixel", parameter->queue_size_pub_command_1_dynamixel,
           kDefaultObjectsPubQueueSize_1_dyn);

  nh.param("pub_rostopic_command_2", parameter->pub_rostopic_command_2,
           kDefaultObjectsPubTopic_2);
  nh.param("queue_size_pub_command_2", parameter->queue_size_pub_command_2,
          kDefaultObjectsPubQueueSize_2);
  nh.param("pub_rostopic_command_2_hebi", parameter->pub_rostopic_command_2_hebi,
           kDefaultObjectsPubTopic_2_hebi);
  nh.param("queue_size_pub_command_2_hebi", parameter->queue_size_pub_command_2_hebi,
          kDefaultObjectsPubQueueSize_2_hebi);

  nh.param("pub_rostopic_command_3", parameter->pub_rostopic_command_3,
           kDefaultObjectsPubTopic_3);
  nh.param("queue_size_pub_command_3", parameter->queue_size_pub_command_3,
           kDefaultObjectsPubQueueSize_3);
  nh.param("pub_rostopic_command_3_dynamixel", parameter->pub_rostopic_command_3_dynamixel,
          kDefaultObjectsPubTopic_3_dynamixel);
  nh.param("queue_size_pub_command_3_dynamixel", parameter->queue_size_pub_command_3_dynamixel,
          kDefaultObjectsPubQueueSize_3_dyn);

  // Torque publish
  nh.param("pub_rostopic_1", parameter->pub_rostopic_1,
             kDefaultPubTopic_1);
  nh.param("pub_rostopic_2", parameter->pub_rostopic_2,
            kDefaultPubTopic_2);
  nh.param("pub_rostopic_3", parameter->pub_rostopic_3,
            kDefaultPubTopic_3);


  nh.param("queue_size_pub_rostopic_1", parameter->queue_size_pub_rostopic_1,
            kDefaultPubQueueSize_1);
  nh.param("queue_size_pub_rostopic_2", parameter->queue_size_pub_rostopic_2,
           kDefaultPubQueueSize_2);
  nh.param("queue_size_pub_rostopic_3", parameter->queue_size_pub_rostopic_3,
            kDefaultPubQueueSize_3);

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

  // Construct class controller_processor with NodeHandle and parameter structure
  controller::ControllerProcessor controller(nh, parameter);

  // Relative path to package
  std::string path = ros::package::getPath("dynamixel_controller");

  // Spin
  ros::spin ();
  ROS_INFO("Spinning node");
  return 0;
}
