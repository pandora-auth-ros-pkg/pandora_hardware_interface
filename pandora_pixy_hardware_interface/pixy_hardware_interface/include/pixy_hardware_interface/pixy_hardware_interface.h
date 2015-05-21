#ifndef PIXY_HARDWARE_INTERFACE_PIXY_HARDWARE_INTERFACE_H
#define PIXY_HARDWARE_INTERFACE_PIXY_HARDWARE_INTERFACE_H


#include "ros/ros.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <image_transport/image_transport.h>
#include "pixy.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>

namespace pandora_hardware_interface
{
  namespace pixy
  {
    class PixyHardwareInterface : public hardware_interface::RobotHW
    {
      public:
       
         explicit PixyHardwareInterface(
        ros::NodeHandle nodeHandle);
        ~PixyHardwareInterface();
        void read();
        void write();
        void readJointNameFromParamServer();
        void get_frame()
          {
            //~ ROS_INFO("YOOOOOOOOOOO");
            //~ pixy_stop();
            pixy_get_frame(frame_);
            fillImage(img_, "bgr8", frame_.rows, frame_.cols, frame_.channels() * frame_.cols, frame_.data);
            //~ pixy_run();
            image_pub.publish(img_);
        }
        sensor_msgs::Image img_;
        image_transport::Publisher image_pub;
        cv::Mat frame_;
      private:
        ros::NodeHandle nodeHandle_;
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
