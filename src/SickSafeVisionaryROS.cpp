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
  m_pointcloud_pub  = m_priv_nh.advertise<sensor_msgs::PointCloud2>("points", 1);


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
  publishPointCloud();
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

void SickSafeVisionaryROS::publishPointCloud()
{
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header       = m_header;
  cloud_msg->height       = m_data_handle->getHeight();
  cloud_msg->width        = m_data_handle->getWidth();
  cloud_msg->is_dense     = false;
  cloud_msg->is_bigendian = false;

  cloud_msg->fields.resize(4);
  cloud_msg->fields[0].name = "x";
  cloud_msg->fields[1].name = "y";
  cloud_msg->fields[2].name = "z";
  cloud_msg->fields[3].name = "intensity";


  int offset = 0;
  for (size_t i = 0; i < 3; ++i)
  {
    cloud_msg->fields[i].offset   = offset;
    cloud_msg->fields[i].datatype = int(sensor_msgs::PointField::FLOAT32);
    cloud_msg->fields[i].count    = 1;
    offset += sizeof(float);
  }

  cloud_msg->fields[3].offset   = offset;
  cloud_msg->fields[3].datatype = int(sensor_msgs::PointField::UINT16);
  cloud_msg->fields[3].count    = 1;
  offset += sizeof(uint16_t);

  cloud_msg->point_step = offset;
  cloud_msg->row_step   = cloud_msg->point_step * cloud_msg->width;
  cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);

  std::vector<visionary::PointXYZ> point_vec;
  m_data_handle->generatePointCloud(point_vec);
  m_data_handle->transformPointCloud(point_vec);

  std::vector<uint16_t>::const_iterator intensity_it = m_data_handle->getIntensityMap().begin();
  std::vector<visionary::PointXYZ>::const_iterator point_it = point_vec.begin();
  // TODO check if both vector sizes align

  for (size_t i = 0; i < point_vec.size(); ++i, ++intensity_it, ++point_it)
  {
    memcpy(&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[0].offset],
           &*point_it,
           sizeof(visionary::PointXYZ));
    memcpy(&cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[3].offset],
           &*intensity_it,
           sizeof(uint16_t));
  }
  m_pointcloud_pub.publish(cloud_msg);
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
