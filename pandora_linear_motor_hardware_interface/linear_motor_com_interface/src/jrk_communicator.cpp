#include <linear_motor_com_interface/jrk_com_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pololu_jrk_communicator_node");
  pandora_hardware_interface::linear::JrkComInterface
    pololu_jrk_driver("/dev/linear", 115200, 100);
  pololu_jrk_driver.init();
  
  int reply = pololu_jrk_driver.readFeedback();
  ROS_ERROR("Got this position feedback: %d", reply);
  reply = pololu_jrk_driver.readScaledFeedback();
  ROS_ERROR("Got this scaled feedback: %d", reply);
  reply = pololu_jrk_driver.readDutyCycle();
  ROS_ERROR("Got this speed: %d", reply);
  reply = pololu_jrk_driver.getErrors();
  ROS_ERROR("Got these errors: %d", reply);

  // Send 'target position' over the serial port
  pololu_jrk_driver.setTarget(132);
  reply = pololu_jrk_driver.readTarget();
  ROS_ERROR("Got this target: %d", reply);

  // Get feedback
  reply = pololu_jrk_driver.readFeedback();
  ROS_ERROR("Got this position feedback: %d", reply);
  reply = pololu_jrk_driver.readScaledFeedback();
  ROS_ERROR("Got this scaled feedback: %d", reply);
  pololu_jrk_driver.setTarget(3600);
  reply = pololu_jrk_driver.readTarget();
  ROS_ERROR("Got this target: %d", reply);

  // Get feedback
  reply = pololu_jrk_driver.readFeedback();
  ROS_ERROR("Got this position feedback: %d", reply);
  reply = pololu_jrk_driver.readScaledFeedback();
  ROS_ERROR("Got this scaled feedback: %d", reply);
  pololu_jrk_driver.closeDevice();
  ros::spinOnce();
}
