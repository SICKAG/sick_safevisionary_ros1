// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann <grosse@fzi.de>
 * \date    2023-06-06
 */
//----------------------------------------------------------------------

#include <ros/ros.h>
#include <sick_safevisionary_driver/sick_safevisionary.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sick_safevisionary_ros_node");
  SickSafeVisionary safevisionary_ros;
  safevisionary_ros.run();
  ros::spin();
  safevisionary_ros.stop();
  return 0;
}
