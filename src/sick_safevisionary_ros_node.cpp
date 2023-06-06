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
#include <sick_safevisionary_ros/SickSafeVisionaryROS.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sick_safevisionary_ros_node");
  SickSafeVisionaryROS safevisionary_ros;
  ros::spin();
  return 0;
}
