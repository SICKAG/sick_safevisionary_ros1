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
  run();
}

bool SickSafeVisionaryROS::run()
{
  m_udp_client_thread_ptr =
    std::make_unique<std::thread>(&SickSafeVisionaryROS::udpClientThread, this);
}

bool SickSafeVisionaryROS::udpClientThread()
{
  while (m_udp_running)
  {
    // If a n ew valid frame is fully received
    if (m_data_stream->getNextBlobUdp())
    {
      std::cout << "Publishing new frame" << std::endl;
      std::cout << m_data_handle->getCameraParameters().k1 << std::endl;
      std::cout << m_data_handle->getCameraParameters().k2 << std::endl;
      std::cout << m_data_handle->getCameraParameters().p1 << std::endl;
      std::cout << m_data_handle->getCameraParameters().p2 << std::endl;
      std::cout << m_data_handle->getCameraParameters().k3 << std::endl;
      std::cout << "" << std::endl; 
      std::cout << m_data_handle->getCameraParameters().fx << std::endl;
      std::cout << m_data_handle->getCameraParameters().fy << std::endl;
      std::cout << m_data_handle->getCameraParameters().cx << std::endl;
      std::cout << m_data_handle->getCameraParameters().cy << std::endl;
    }
  }
}
