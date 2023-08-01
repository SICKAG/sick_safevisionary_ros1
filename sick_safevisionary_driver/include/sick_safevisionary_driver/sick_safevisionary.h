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

#ifndef SICK_SAFEVISIONARY_DRIVER_SICK_SAFEVISIONARY_H_INCLUDED
#define SICK_SAFEVISIONARY_DRIVER_SICK_SAFEVISIONARY_H_INCLUDED

#include "image_transport/publisher.h"
#include "sick_safevisionary_base/PointXYZ.h"
#include "sick_safevisionary_base/SafeVisionaryData.h"
#include "sick_safevisionary_base/SafeVisionaryDataStream.h"
#include "sick_safevisionary_msgs/CameraIOs.h"
#include "sick_safevisionary_msgs/DeviceStatus.h"
#include "sick_safevisionary_msgs/FieldInformationArray.h"
#include "sick_safevisionary_msgs/ROIArray.h"
#include <boost/lockfree/policies.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/ros.h>


class SickSafeVisionary
{
public:
  SickSafeVisionary();
  virtual ~SickSafeVisionary(){};
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
  void publishDeviceStatus();
  void publishIOs();
  void publishROI();
  void publishFieldInformation();

  sensor_msgs::ImagePtr Vec16ToImage(std::vector<uint16_t> vec);
  sensor_msgs::ImagePtr Vec8ToImage(std::vector<uint8_t> vec);

  std::shared_ptr<visionary::SafeVisionaryData> data_handle_;
  visionary::SafeVisionaryData last_handle_;
  std::shared_ptr<visionary::SafeVisionaryDataStream> data_stream_;

  ros::Publisher camera_info_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher device_status_pub_;
  ros::Publisher io_pub_;
  ros::Publisher roi_pub_;
  ros::Publisher field_pub_;

  image_transport::Publisher depth_pub_;
  image_transport::Publisher intensity_pub_;
  image_transport::Publisher state_pub_;

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  std_msgs::Header header_;
  std::string frame_id_;

  int udp_port_;
  std::unique_ptr<std::thread> receive_thread_ptr_;
  std::unique_ptr<std::thread> publish_thread_ptr_;
  std::atomic<bool> data_available_;

  boost::lockfree::spsc_queue<visionary::SafeVisionaryData, boost::lockfree::capacity<10> >
    spsc_queue_;
};

#endif /* SICK_SAFEVISIONARY_DRIVER_SICK_SAFEVISIONARY_H_INCLUDED */
