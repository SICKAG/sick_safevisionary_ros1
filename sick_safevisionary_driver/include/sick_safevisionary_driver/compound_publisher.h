// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann <grosse@fzi.de>
 * \date    2023-08-01
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFEVISIONARY_DRIVER_COMPOUND_PUBLISHER_H_INCLUDED
#define SICK_SAFEVISIONARY_DRIVER_COMPOUND_PUBLISHER_H_INCLUDED


#include <image_transport/publisher.h>

#include <image_transport/image_transport.h>
#include <ros/message_traits.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


#include <sick_safevisionary_base/PointXYZ.h>
#include <sick_safevisionary_base/SafeVisionaryData.h>
#include <sick_safevisionary_msgs/CameraIOs.h>
#include <sick_safevisionary_msgs/DeviceStatus.h>
#include <sick_safevisionary_msgs/FieldInformationArray.h>
#include <sick_safevisionary_msgs/ROIArray.h>

class CompoundPublisher
{
public:
  CompoundPublisher();
  virtual ~CompoundPublisher(){};
  void publish(const std_msgs::Header& header, visionary::SafeVisionaryData& frame_data);

private:
  void publishCameraInfo(const std_msgs::Header& header,
                         const visionary::SafeVisionaryData& frame_data);
  void publishPointCloud(const std_msgs::Header& header, visionary::SafeVisionaryData& frame_data);
  void publishDepthImage(const std_msgs::Header& header,
                         const visionary::SafeVisionaryData& frame_data);
  void publishIntensityImage(const std_msgs::Header& header,
                             const visionary::SafeVisionaryData& frame_data);
  void publishStateMap(const std_msgs::Header& header,
                       const visionary::SafeVisionaryData& frame_data);
  void publishIMUData(const std_msgs::Header& header,
                      const visionary::SafeVisionaryData& frame_data);
  void publishDeviceStatus(const std_msgs::Header& header,
                           const visionary::SafeVisionaryData& frame_data);
  void publishIOs(const std_msgs::Header& header, const visionary::SafeVisionaryData& frame_data);
  void publishROI(const std_msgs::Header& header, const visionary::SafeVisionaryData& frame_data);
  void publishFieldInformation(const std_msgs::Header& header,
                               const visionary::SafeVisionaryData& frame_data);
  sensor_msgs::ImagePtr Vec16ToImage(const std_msgs::Header& header,
                                     const visionary::SafeVisionaryData& frame_data,
                                     std::vector<uint16_t> vec);
  sensor_msgs::ImagePtr Vec8ToImage(const std_msgs::Header& header,
                                    const visionary::SafeVisionaryData& frame_data,
                                    std::vector<uint8_t> vec);

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
  ros::NodeHandle priv_nh_;
};

#endif /* SICK_SAFEVISIONARY_DRIVER_COMPOUND_PUBLISHER_H_INCLUDED */
