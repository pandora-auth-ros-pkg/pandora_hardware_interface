

#include "pixy_hardware_interface/pixy_hardware_interface.h"


namespace pandora_hardware_interface
{
  namespace pixy
  {
    
    PixyHardwareInterface::PixyHardwareInterface(
    ros::NodeHandle nodeHandle,PixyCam* cam)
    :
      nodeHandle_(nodeHandle),
      it_(nodeHandle),
      cam_(cam)
    {
      
      readJointNameFromParamServer();
      image_pub = it_.advertise("image_raw", 1);
      //~ int ret = pixy_init();
      //~ if (ret != 0)
      //~ {
        //~ ROS_FATAL("PixyNode - %s - Failed to open with the USB error %d!",
        //~ __FUNCTION__, ret);
        //~ ROS_BREAK();
      //~ }
      
    

    // connect and register the joint state interface
    for(int ii=0;ii<2;ii++)
    {
      position_[ii] = 0;
      velocity_[ii] = 0;
      effort_[ii] = 0;
      hardware_interface::JointStateHandle jointStateHandle(
        jointNames_[ii],
        &position_[ii],
        &velocity_[ii],
        &effort_[ii]);
      jointStateInterface_.registerHandle(jointStateHandle);
    }
      registerInterface(&jointStateInterface_);

    // connect and register the joint position interface
    for(int ii=0;ii<2;ii++)
    {
      hardware_interface::JointHandle jointPositionHandle(
        jointStateInterface_.getHandle(jointNames_[ii]),
        &command_[ii]);
      positionJointInterface_.registerHandle(jointPositionHandle);
    }
    registerInterface(&positionJointInterface_);
    }
    
    PixyHardwareInterface::~PixyHardwareInterface()
    {
    }
    
    void PixyHardwareInterface::read()
    {
      int feedback[2];
      feedback[0] = pixy_rcs_get_position(0);
      feedback[1] = pixy_rcs_get_position(1);
      position_[0] = static_cast<float>(feedback[0]);
      position_[1] = static_cast<float>(feedback[1]);
    }
    
    
    void PixyHardwareInterface::write()
    {
      uint16_t target[2];
      target[0] = static_cast<uint16_t>(command_[0]);
      target[1] = static_cast<uint16_t>(command_[1]);
      if (target[0] >= 0 && target[0] <= 999   &&  target[1] >= 0 && target[1] <= 999)
      {
        pixy_rcs_set_position(0, target[0]);
        pixy_rcs_set_position(1, target[1]);
      }
      else
      {
        ROS_DEBUG_STREAM("Pixy servo command out of bounds");
      }
    }
    
    void PixyHardwareInterface::get_frame()
    {
      cv::Mat frame = cam_->getImage();
      fillImage(img_, "bgr8", frame.rows, frame.cols, frame.channels() * frame.cols, frame.data);

      image_pub.publish(img_);
      
    }
    
    void PixyHardwareInterface::readJointNameFromParamServer()
  {
    std::string name;
    nodeHandle_.getParam(
      "pixy_servo_joint/pan_joint",
      name);
    jointNames_.push_back(name);
    nodeHandle_.getParam(
      "pixy_servo_joint/tilt_joint",
      name);
    jointNames_.push_back(name);
  }
    
    
  }
}
