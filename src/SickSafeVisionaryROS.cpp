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

SickSafeVisionaryROS::SickSafeVisionaryROS()
{
  m_data_handle = std::make_shared<visionary::SafeVisionaryData>();
  m_data_stream = std::make_shared<visionary::SafeVisionaryDataStream>(m_data_handle);
  if (!m_data_stream->openUdpConnection(htons(6060)))
  {
    ROS_INFO_STREAM("Could not open udp connection");
    return;
  }
}
