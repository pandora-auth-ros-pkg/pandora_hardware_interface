#include "ros/ros.h"
#include "pixy_hardware_interface.h"
#include <sensor_msgs/fill_image.h>
#include <self_test/self_test.h>
#include <image_transport/image_transport.h>

#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pixycam.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "pixy_hardware_interface_node");

  ros::NodeHandle nodeHandle;
  
  PixyCam cam;
  
  pandora_hardware_interface::pixy::PixyHardwareInterface
    pixyHardwareInterface(
    nodeHandle,&cam);
  controller_manager::ControllerManager controllerManager(
    &pixyHardwareInterface,
    nodeHandle);

  ros::Time
    last,
    now;

    now = last = ros::Time::now();
    ros::Duration period(1.0);

    ros::AsyncSpinner spinner(2);
    spinner.start();


  ros::Rate loop_rate(10);

  boost::thread workerThread(&PixyCam::run, pixyHardwareInterface.cam_);
//    cam.run();

  int count = 0;
  while (ros::ok()){

      pixyHardwareInterface.get_frame();

      //~ pixyHardwareInterface.read();
      //~ controllerManager.update(now, period);
      //~ pixyHardwareInterface.write();

      loop_rate.sleep();
      ++count;
  }
  spinner.stop();
  return 0;
}
