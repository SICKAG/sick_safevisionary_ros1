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

#include <sick_safevisionary_ros/SickSafeVisionaryROS.h>

SickSafeVisionaryROS::SickSafeVisionaryROS()
  : m_priv_nh("~/")
{
  // TODO fill header correctly
  m_header.frame_id = "camera";
  m_udp_running     = true;

  m_data_handle = std::make_shared<visionary::SafeVisionaryData>();
  m_data_stream = std::make_shared<visionary::SafeVisionaryDataStream>(m_data_handle);

  m_camera_info_pub = m_priv_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);


  image_transport::ImageTransport image_transport(m_priv_nh);
  m_depth_pub     = image_transport.advertise("depth", 1);
  m_intensity_pub = image_transport.advertise("intensity", 1);
  m_state_pub     = image_transport.advertise("state", 1);

  if (!m_data_stream->openUdpConnection(htons(6060)))
  {
    ROS_INFO_STREAM("Could not open udp connection");
    return;
  }
}

void SickSafeVisionaryROS::run()
{
  // check if tcp or udp
  m_udp_client_thread_ptr =
    std::make_unique<std::thread>(&SickSafeVisionaryROS::udpClientThread, this);
}

void SickSafeVisionaryROS::stop()
{
  m_udp_client_thread_ptr->join();
}

void SickSafeVisionaryROS::udpClientThread()
{
  while (m_udp_running)
  {
    if (m_data_stream->getNextBlobUdp())
    {
      processFrame();
    }
  }
}

void SickSafeVisionaryROS::processFrame()
{
  // TODO add header
  m_header.stamp = ros::Time::now();
  publishCameraInfo();
  publishDepthImage();
  publishIntensityImage();
  publishStateMap();
}

void SickSafeVisionaryROS::publishCameraInfo()
{
  sensor_msgs::CameraInfo camera_info;
  camera_info.header = m_header;

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

void SickSafeVisionaryROS::publishDepthImage()
{
  m_depth_pub.publish(Vec16ToImage(m_data_handle->getDistanceMap()));
}

void SickSafeVisionaryROS::publishIntensityImage()
{
  m_intensity_pub.publish(Vec16ToImage(m_data_handle->getIntensityMap()));
}

void SickSafeVisionaryROS::publishStateMap()
{
  m_state_pub.publish(Vec8ToImage(m_data_handle->getStateMap()));
}

sensor_msgs::ImagePtr SickSafeVisionaryROS::Vec16ToImage(std::vector<uint16_t> vec)
{
  cv::Mat image = cv::Mat(m_data_handle->getHeight(), m_data_handle->getWidth(), CV_16UC1);
  std::memcpy(image.data, vec.data(), vec.size() * sizeof(uint16_t));
  return cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::TYPE_16UC1, image)
    .toImageMsg();
}
sensor_msgs::ImagePtr SickSafeVisionaryROS::Vec8ToImage(std::vector<uint8_t> vec)
{
  cv::Mat image = cv::Mat(m_data_handle->getHeight(), m_data_handle->getWidth(), CV_8UC1);
  std::memcpy(image.data, vec.data(), vec.size() * sizeof(uint8_t));
  return cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::TYPE_8UC1, image)
    .toImageMsg();
}
