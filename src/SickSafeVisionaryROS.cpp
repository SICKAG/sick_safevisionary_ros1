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

#include "sensor_msgs/CameraInfo.h"
#include <ros/ros.h>
#include <sick_safevisionary_ros/SickSafeVisionaryROS.h>

SickSafeVisionaryROS::SickSafeVisionaryROS() :
  m_priv_nh("~/")
{
  m_data_handle = std::make_shared<visionary::SafeVisionaryData>();
  m_data_stream = std::make_shared<visionary::SafeVisionaryDataStream>(m_data_handle);

  m_camera_info_pub = m_priv_nh.advertise<sensor_msgs::CameraInfo>("camera_info",1);



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
      processUDPPacket();
    }
  }
}

void SickSafeVisionaryROS::processUDPPacket()
{
  publishCameraInfo();
}

void SickSafeVisionaryROS::publishCameraInfo()
{
  sensor_msgs::CameraInfo camera_info;

  camera_info.height = m_data_handle->getHeight();
  camera_info.width  = m_data_handle->getWidth();
  camera_info.D      = std::vector<double>(5, 0);
  camera_info.D[0]   = m_data_handle->getCameraParameters().k1;
  camera_info.D[1]   = m_data_handle->getCameraParameters().k2;
  camera_info.D[2]   = m_data_handle->getCameraParameters().p1;
  camera_info.D[3]   = m_data_handle->getCameraParameters().p2;
  camera_info.D[4]   = m_data_handle->getCameraParameters().k3;

  camera_info.K[0] = m_data_handle->getCameraParameters().fx;
  camera_info.K[2] = m_data_handle->getCameraParameters().cx;
  camera_info.K[4] = m_data_handle->getCameraParameters().fy;
  camera_info.K[5] = m_data_handle->getCameraParameters().cy;
  camera_info.K[8] = 1;
  // TODO add missing parameter in Projection Matrix
  m_camera_info_pub.publish(camera_info);
}
