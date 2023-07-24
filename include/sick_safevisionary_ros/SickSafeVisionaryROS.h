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

#ifndef SICK_SAFEVISIONARY_ROS_SICK_SAFE_VISIONARY_H_INCLUDED
#define SICK_SAFEVISIONARY_ROS_SICK_SAFE_VISIONARY_H_INCLUDED

#include "image_transport/publisher.h"
#include "sick_safevisionary_base/PointXYZ.h"
#include "sick_safevisionary_base/SafeVisionaryData.h"
#include "sick_safevisionary_base/SafeVisionaryDataStream.h"
#include <mutex>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/ros.h>

class SickSafeVisionaryROS
{
public:
  SickSafeVisionaryROS();
  virtual ~SickSafeVisionaryROS(){};
  void run();
  void stop();

private:
  void receiveThread();
  void publishThread();
  void processFrame();

  void publishCameraInfo();
  void publishPointCloud();
  void publishDepthImage();
  void publishIntensityImage();
  void publishStateMap();
  void publishIMUData();

  sensor_msgs::ImagePtr Vec16ToImage(std::vector<uint16_t> vec);
  sensor_msgs::ImagePtr Vec8ToImage(std::vector<uint8_t> vec);

  std::shared_ptr<visionary::SafeVisionaryData> m_data_handle;
  visionary::SafeVisionaryData m_last_handle;
  std::shared_ptr<visionary::SafeVisionaryDataStream> m_data_stream;

  ros::Publisher m_camera_info_pub;
  ros::Publisher m_pointcloud_pub;
  ros::Publisher m_imu_pub;

  image_transport::Publisher m_depth_pub;
  image_transport::Publisher m_intensity_pub;
  image_transport::Publisher m_state_pub;

  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;
  std_msgs::Header m_header;
  std::string m_frame_id;

  std::string m_connection_type;
  std::string m_ip;
  int m_udp_port;
  // TODO add tcp param
  std::unique_ptr<std::thread> m_receive_thread_ptr;
  std::unique_ptr<std::thread> m_publish_thread_ptr;
  std::mutex m_data_mutex;
  bool m_data_available;
};

#endif /* SICK_SAFEVISIONARY_ROS_SICK_SAFE_VISIONARY_H_INCLUDED */
