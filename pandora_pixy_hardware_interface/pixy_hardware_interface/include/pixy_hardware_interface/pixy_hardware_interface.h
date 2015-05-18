#ifndef PIXY_HARDWARE_INTERFACE_PIXY_HARDWARE_INTERFACE_H
#define PIXY_HARDWARE_INTERFACE_PIXY_HARDWARE_INTERFACE_H


#include "ros/ros.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include "pixycam.h"
#include "pixy.h"

namespace pandora_hardware_interface
{
  namespace pixy
  {
    class PixyHardwareInterface : public hardware_interface::RobotHW
    {
      public:
        explicit PixyHardwareInterface(
        ros::NodeHandle nodeHandle,PixyCam* cam);
        ~PixyHardwareInterface();
        PixyCam* cam_;
        void read();
        void write();
        void get_frame();
        void readJointNameFromParamServer();
      private:
        sensor_msgs::Image img_;
        ros::NodeHandle nodeHandle_;
        image_transport::Publisher image_pub;
        image_transport::ImageTransport it_;
        hardware_interface::JointStateInterface jointStateInterface_;
        hardware_interface::PositionJointInterface positionJointInterface_;
        std::vector<std::string> jointNames_;
        double command_[2];
        double position_[2];
        double velocity_[2];
        double effort_[2];
    };
    
  }
}








#endif //PIXY_HARDWARE_INTERFACE_PIXY_HARDWARE_INTERFACE_H
