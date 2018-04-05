//
// Created by Leo on 16.02.16.
//

#ifndef DYNAMIXEL_CONTROLLER_PARAMETER_BAG_H
#define DYNAMIXEL_CONTROLLER_PARAMETER_BAG_H

#include <string>

namespace controller {

/**
 * \brief Struct to store all necessary parameters for the project with
 *        substructure of parameter bags for other modules.
 * @author Leonard Schai (lschai@student.ethz.ch)
 * @date February, 2017
 */
struct ParameterBag {
  /** Node name of the controller package. */
  std::string node_name;

  /** Subscribed rostopic for the data of the Encoder 1. */
  std::string sub_rostopic_enc_1;

  /** Queue size for the encoder rostopic subscriber. */
  int queue_size_sub_enc_1;

  /** Subscribed rostopic for the data of the Encoder 3 */
  std::string sub_rostopic_enc_3;

  /** Queue size for the encoder rostopic subscriber. */
  int queue_size_sub_enc_3;



  /** Published rostopic for the dynamixel. */
  std::string pub_rostopic_command_1;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_command_1;

  /** Published rostopic for the dynamixel. */
  std::string pub_rostopic_command_1_dynamixel;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_command_1_dynamixel;

  /** Published rostopic for the HEBI. */
  std::string pub_rostopic_command_2;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_command_2;

  /** Published rostopic for the HEBI. */
  std::string pub_rostopic_command_2_hebi;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_command_2_hebi;

  /** Published rostopic for the dynamixel. */
  std::string pub_rostopic_command_3;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_command_3;

  /** Published rostopic for the dynamixel. */
  std::string pub_rostopic_command_3_dynamixel;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_command_3_dynamixel;


  /** Published rostopic angle 1. */
  std::string pub_rostopic_1;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_rostopic_1;

  /** Published rostopic angle 3. */
  std::string pub_rostopic_2;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_rostopic_2;

  /** Published rostopic angle 1 filtered. */
  std::string pub_rostopic_3;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_rostopic_3;

  /** Published rostopic angle 3 filtered. */
  std::string pub_rostopic_4;

  /** Queue size for the rostopic publisher. */
  int queue_size_pub_rostopic_4;







  /** Subscribed rostopic for the data of the internal encoder of the dynamixel 1. */
  std::string sub_rostopic_dynamixel_1;

  /** Queue size for the dynamixel rostopic subscriber. */
  int queue_size_sub_dynamixel_1;

  /** Subscribed rostopic for the data of the internal encoder of the dynamixel 3. */
  std::string sub_rostopic_dynamixel_3;

  /** Queue size for the dynamixel rostopic subscriber. */
  int queue_size_sub_dynamixel_3;





};

} // namespace controller

#endif //DYNAMIXEL_CONTROLLER_PARAMETER_BAG_H
