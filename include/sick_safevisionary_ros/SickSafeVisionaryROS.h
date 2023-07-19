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

#include "sick_safevisionary_base/SafeVisionaryData.h"
#include "sick_safevisionary_base/SafeVisionaryDataStream.h"
#include <thread>

#include <sensor_msgs/CameraInfo.h>

class SickSafeVisionaryROS
{
public:
  SickSafeVisionaryROS();
  virtual ~SickSafeVisionaryROS(){};
  void run();
  void stop();

private:
  void udpClientThread();
  void processFrame();

  // TODO createPointcloud
  void publishCameraInfo();
  // TODO createDepthImage

  std::shared_ptr<visionary::SafeVisionaryData> m_data_handle;
  std::shared_ptr<visionary::SafeVisionaryDataStream> m_data_stream;

  ros::Publisher m_camera_info_pub;
  ros::Publisher m_pointcloud_pub;

  // TODO not sure if necessary
  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;

  std::string m_frame_id;
  std::string m_connection_type;
  std::string m_ip;
  int32_t m_udp_port;
  //TODO add tcp param
  std::unique_ptr<std::thread> m_udp_client_thread_ptr;
  bool m_udp_running;

};

#endif /* SICK_SAFEVISIONARY_ROS_SICK_SAFE_VISIONARY_H_INCLUDED */
