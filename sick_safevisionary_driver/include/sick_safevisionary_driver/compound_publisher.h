// -- BEGIN LICENSE BLOCK ----------------------------------------------
/*!
*  Copyright (C) 2023, SICK AG, Waldkirch, Germany
*  Copyright (C) 2023, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann <grosse@fzi.de>
 * \author  Stefan Scherzinger <scherzin@fzi.de>
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
#include <sick_safevisionary_msgs/CameraIO.h>
#include <sick_safevisionary_msgs/DeviceStatus.h>
#include <sick_safevisionary_msgs/FieldInformationArray.h>
#include <sick_safevisionary_msgs/ROIArray.h>

/**
 * @brief A compound publisher for the driver's topics
 *
 * This class encapsulates all necessary publishers to keep the driver's
 * thread management nice and clear.
 */
class CompoundPublisher
{
public:
  CompoundPublisher();
  virtual ~CompoundPublisher(){};
  /**
   * @brief Publish once with all registered publishers
   *
   * @param header Header for this data
   * @param frame_data The sensor's data to publish
   */
  void publish(const std_msgs::Header& header, visionary::SafeVisionaryData& frame_data);

private:
  /*!
   * \brief Publishes the camera information of the sensor
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishCameraInfo(const std_msgs::Header& header,
                         const visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the generated 3D Pointcloud
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishPointCloud(const std_msgs::Header& header, visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the values of the IMU
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishIMUData(const std_msgs::Header& header,
                      const visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the device status
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishDeviceStatus(const std_msgs::Header& header,
                           const visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the state of the IO ports
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishIOs(const std_msgs::Header& header, const visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the regions of interest
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishROI(const std_msgs::Header& header, const visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the evaluation of the configured safety fields
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishFieldInformation(const std_msgs::Header& header,
                               const visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the raw depth image
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishDepthImage(const std_msgs::Header& header,
                         const visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the raw intensity image
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishIntensityImage(const std_msgs::Header& header,
                             const visionary::SafeVisionaryData& frame_data);
  /*!
   * \brief Publishes the raw state map of each pixel
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   */
  void publishStateMap(const std_msgs::Header& header,
                       const visionary::SafeVisionaryData& frame_data);

  /*!
   * \brief Creates an image from a data vector
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   * \param vec The converted data vector
   *
   * \returns Image projection of input data vector
   */
  sensor_msgs::ImagePtr Vec16ToImage(const std_msgs::Header& header,
                                     const visionary::SafeVisionaryData& frame_data,
                                     std::vector<uint16_t> vec);
  /*!
   * \brief Creates an image from a data vector
   *
   * \param header Header for this data
   * \param frame_data The sensor's data to publish
   * \param vec The converted data vector
   *
   * \returns Image projection of input data vector
   */
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
