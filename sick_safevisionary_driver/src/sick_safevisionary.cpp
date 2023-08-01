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

#include <sick_safevisionary_msgs/CameraIOs.h>
#include <sick_safevisionary_msgs/DeviceStatus.h>
#include <sick_safevisionary_msgs/ROI.h>
#include <sick_safevisionary_msgs/ROIArray.h>
#include <sick_safevisionary_driver/sick_safevisionary.h>
#include <thread>

SickSafeVisionary::SickSafeVisionary()
  : m_priv_nh("~/")
  , m_data_available(false)
{
  m_priv_nh.param<std::string>("frame_id", m_frame_id, "camera");
  m_header.frame_id = m_frame_id;
  m_priv_nh.param<int>("udp_port", m_udp_port, 6060);

  m_data_handle = std::make_shared<visionary::SafeVisionaryData>();
  m_data_stream = std::make_shared<visionary::SafeVisionaryDataStream>(m_data_handle);

  m_camera_info_pub = m_priv_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
  m_pointcloud_pub  = m_priv_nh.advertise<sensor_msgs::PointCloud2>("points", 1);
  m_imu_pub         = m_priv_nh.advertise<sensor_msgs::Imu>("imu_data", 1);
  m_io_pub          = m_priv_nh.advertise<sick_safevisionary_msgs::CameraIOs>("camera_io", 1);
  m_roi_pub   = m_priv_nh.advertise<sick_safevisionary_msgs::ROIArray>("region_of_interest", 1);
  m_field_pub = m_priv_nh.advertise<sick_safevisionary_msgs::FieldInformationArray>("fields", 1);
  m_device_status_pub =
    m_priv_nh.advertise<sick_safevisionary_msgs::DeviceStatus>("device_status", 1);

  image_transport::ImageTransport image_transport(m_priv_nh);
  m_depth_pub     = image_transport.advertise("depth", 1);
  m_intensity_pub = image_transport.advertise("intensity", 1);
  m_state_pub     = image_transport.advertise("state", 1);

  if (m_data_stream->openUdpConnection(htons((int)m_udp_port)))
  {
    ROS_INFO_STREAM("Opening UDP Connection to Sensor.");
    bool waiting_for_connection = true;
    while (waiting_for_connection)
    {
      ROS_INFO_STREAM("Waiting for camera connection.");
      if (m_data_stream->getNextBlobUdp())
      {
        waiting_for_connection = false;
        ROS_INFO_STREAM("Received first frame, starting to publish data.");
      }
      else
      {
        ROS_INFO_STREAM("No UDP packages received until now. Please make sure that your sensor is "
                        "connected and correctly configured.");
      }
    }
  }
  else
  {
    ROS_INFO_STREAM("Could not open udp connection");
    return;
  }
}

void SickSafeVisionary::run()
{
  m_receive_thread_ptr = std::make_unique<std::thread>(&SickSafeVisionary::receiveThread, this);
  m_publish_thread_ptr = std::make_unique<std::thread>(&SickSafeVisionary::publishThread, this);
}

void SickSafeVisionary::stop()
{
  m_receive_thread_ptr->join();
  m_publish_thread_ptr->join();
  m_data_stream->closeUdpConnection();
}

void SickSafeVisionary::receiveThread()
{
  while (ros::ok())
  {
    if (m_data_stream->getNextBlobUdp())
    {
      m_spsc_queue.push(*m_data_handle);
      m_data_available = true;
    }
    else
    {
      ROS_DEBUG_STREAM("UDP stream was incomplete. Skipping frame and waiting for new data.");
      continue;
    }
  }
}

void SickSafeVisionary::publishThread()
{
  while (ros::ok())
  {
    if (m_data_available)
    {
      m_spsc_queue.pop(m_last_handle);
      processFrame();
      m_data_available = false;
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }
}

void SickSafeVisionary::processFrame()
{
  m_header.stamp = ros::Time::now();

  if (m_camera_info_pub.getNumSubscribers() > 0)
  {
    publishCameraInfo();
  }
  if (m_pointcloud_pub.getNumSubscribers() > 0)
  {
    publishPointCloud();
  }
  if (m_depth_pub.getNumSubscribers() > 0)
  {
    publishDepthImage();
  }
  if (m_intensity_pub.getNumSubscribers() > 0)
  {
    publishIntensityImage();
  }
  if (m_state_pub.getNumSubscribers() > 0)
  {
    publishStateMap();
  }
  if (m_imu_pub.getNumSubscribers() > 0)
  {
    publishIMUData();
  }
  if (m_device_status_pub.getNumSubscribers() > 0)
  {
    publishDeviceStatus();
  }
  if (m_io_pub.getNumSubscribers() > 0)
  {
    publishIOs();
  }
  if (m_roi_pub.getNumSubscribers() > 0)
  {
    publishROI();
  }
  if (m_field_pub.getNumSubscribers() > 0)
  {
    publishFieldInformation();
  }
}

void SickSafeVisionary::publishCameraInfo()
{
  sensor_msgs::CameraInfo camera_info;
  camera_info.header = m_header;

  camera_info.height = m_last_handle.getHeight();
  camera_info.width  = m_last_handle.getWidth();
  camera_info.D      = std::vector<double>(5, 0);
  camera_info.D[0]   = m_last_handle.getCameraParameters().k1;
  camera_info.D[1]   = m_last_handle.getCameraParameters().k2;
  camera_info.D[2]   = m_last_handle.getCameraParameters().p1;
  camera_info.D[3]   = m_last_handle.getCameraParameters().p2;
  camera_info.D[4]   = m_last_handle.getCameraParameters().k3;

  camera_info.K[0] = m_last_handle.getCameraParameters().fx;
  camera_info.K[2] = m_last_handle.getCameraParameters().cx;
  camera_info.K[4] = m_last_handle.getCameraParameters().fy;
  camera_info.K[5] = m_last_handle.getCameraParameters().cy;
  camera_info.K[8] = 1;
  // TODO add missing parameter in Projection Matrix
  m_camera_info_pub.publish(camera_info);
}

void SickSafeVisionary::publishPointCloud()
{
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header       = m_header;
  cloud_msg->height       = m_last_handle.getHeight();
  cloud_msg->width        = m_last_handle.getWidth();
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
  m_last_handle.generatePointCloud(point_vec);
  m_last_handle.transformPointCloud(point_vec);

  std::vector<uint16_t>::const_iterator intensity_it = m_last_handle.getIntensityMap().begin();
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

void SickSafeVisionary::publishIMUData()
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header                = m_header;
  imu_msg.angular_velocity.x    = m_last_handle.getIMUData().angularVelocity.X;
  imu_msg.angular_velocity.y    = m_last_handle.getIMUData().angularVelocity.Y;
  imu_msg.angular_velocity.z    = m_last_handle.getIMUData().angularVelocity.Z;
  imu_msg.linear_acceleration.x = m_last_handle.getIMUData().acceleration.X;
  imu_msg.linear_acceleration.y = m_last_handle.getIMUData().acceleration.Y;
  imu_msg.linear_acceleration.z = m_last_handle.getIMUData().acceleration.Z;
  imu_msg.orientation.x         = m_last_handle.getIMUData().orientation.X;
  imu_msg.orientation.y         = m_last_handle.getIMUData().orientation.Y;
  imu_msg.orientation.z         = m_last_handle.getIMUData().orientation.Z;
  imu_msg.orientation.w         = m_last_handle.getIMUData().orientation.W;
  m_imu_pub.publish(imu_msg);
}

void SickSafeVisionary::publishDepthImage()
{
  m_depth_pub.publish(Vec16ToImage(m_last_handle.getDistanceMap()));
}

void SickSafeVisionary::publishIntensityImage()
{
  m_intensity_pub.publish(Vec16ToImage(m_last_handle.getIntensityMap()));
}

void SickSafeVisionary::publishStateMap()
{
  m_state_pub.publish(Vec8ToImage(m_last_handle.getStateMap()));
}

sensor_msgs::ImagePtr SickSafeVisionary::Vec16ToImage(std::vector<uint16_t> vec)
{
  cv::Mat image = cv::Mat(m_last_handle.getHeight(), m_last_handle.getWidth(), CV_16UC1);
  std::memcpy(image.data, vec.data(), vec.size() * sizeof(uint16_t));
  return cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::TYPE_16UC1, image).toImageMsg();
}
sensor_msgs::ImagePtr SickSafeVisionary::Vec8ToImage(std::vector<uint8_t> vec)
{
  cv::Mat image = cv::Mat(m_last_handle.getHeight(), m_last_handle.getWidth(), CV_8UC1);
  std::memcpy(image.data, vec.data(), vec.size() * sizeof(uint8_t));
  return cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::TYPE_8UC1, image).toImageMsg();
}


void SickSafeVisionary::publishDeviceStatus()
{
  sick_safevisionary_msgs::DeviceStatus status;
  status.status = static_cast<uint8_t>(m_last_handle.getDeviceStatus());
  status.general_status.application_error =
    m_last_handle.getDeviceStatusData().generalStatus.applicationError;
  status.general_status.contamination_error =
    m_last_handle.getDeviceStatusData().generalStatus.contaminationError;
  status.general_status.contamination_warning =
    m_last_handle.getDeviceStatusData().generalStatus.contaminationWarning;
  status.general_status.dead_zone_detection =
    m_last_handle.getDeviceStatusData().generalStatus.deadZoneDetection;
  status.general_status.device_error =
    m_last_handle.getDeviceStatusData().generalStatus.deviceError;
  status.general_status.temperature_warning =
    m_last_handle.getDeviceStatusData().generalStatus.temperatureWarning;
  status.general_status.run_mode_active =
    m_last_handle.getDeviceStatusData().generalStatus.runModeActive;
  status.general_status.wait_for_cluster =
    m_last_handle.getDeviceStatusData().generalStatus.waitForCluster;
  status.general_status.wait_for_input =
    m_last_handle.getDeviceStatusData().generalStatus.waitForInput;
  status.COP_non_safety_related = m_last_handle.getDeviceStatusData().COPNonSaftyRelated;
  status.COP_safety_related     = m_last_handle.getDeviceStatusData().COPSaftyRelated;
  status.COP_reset_required     = m_last_handle.getDeviceStatusData().COPResetRequired;
  status.active_monitoring_case.monitoring_case_1 =
    m_last_handle.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase1;
  status.active_monitoring_case.monitoring_case_2 =
    m_last_handle.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase2;
  status.active_monitoring_case.monitoring_case_3 =
    m_last_handle.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase3;
  status.active_monitoring_case.monitoring_case_4 =
    m_last_handle.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase4;
  status.contamination_level = m_last_handle.getDeviceStatusData().contaminationLevel;
  m_device_status_pub.publish(status);
}

void SickSafeVisionary::publishIOs()
{
  sick_safevisionary_msgs::CameraIOs camera_ios;
  camera_ios.configured.pin_5 =
    m_last_handle.getLocalIOData().universalIOConfigured.configuredUniIOPin5;
  camera_ios.configured.pin_6 =
    m_last_handle.getLocalIOData().universalIOConfigured.configuredUniIOPin6;
  camera_ios.configured.pin_7 =
    m_last_handle.getLocalIOData().universalIOConfigured.configuredUniIOPin7;
  camera_ios.configured.pin_8 =
    m_last_handle.getLocalIOData().universalIOConfigured.configuredUniIOPin8;
  camera_ios.direction.pin_5 =
    m_last_handle.getLocalIOData().universalIODirection.directionValueUniIOPin5;
  camera_ios.direction.pin_6 =
    m_last_handle.getLocalIOData().universalIODirection.directionValueUniIOPin6;
  camera_ios.direction.pin_7 =
    m_last_handle.getLocalIOData().universalIODirection.directionValueUniIOPin7;
  camera_ios.direction.pin_8 =
    m_last_handle.getLocalIOData().universalIODirection.directionValueUniIOPin8;
  camera_ios.input_values.pin_5 =
    m_last_handle.getLocalIOData().universalIOInputValue.logicalValueUniIOPin5;
  camera_ios.input_values.pin_6 =
    m_last_handle.getLocalIOData().universalIOInputValue.logicalValueUniIOPin6;
  camera_ios.input_values.pin_7 =
    m_last_handle.getLocalIOData().universalIOInputValue.logicalValueUniIOPin7;
  camera_ios.input_values.pin_8 =
    m_last_handle.getLocalIOData().universalIOInputValue.logicalValueUniIOPin8;
  camera_ios.output_values.pin_5 =
    m_last_handle.getLocalIOData().universalIOOutputValue.localOutput1Pin5;
  camera_ios.output_values.pin_6 =
    m_last_handle.getLocalIOData().universalIOOutputValue.localOutput2Pin6;
  camera_ios.output_values.pin_7 =
    m_last_handle.getLocalIOData().universalIOOutputValue.localOutput3Pin7;
  camera_ios.output_values.pin_8 =
    m_last_handle.getLocalIOData().universalIOOutputValue.localOutput4Pin8;
  camera_ios.ossds_state.OSSD1A  = m_last_handle.getLocalIOData().ossdsState.stateOSSD1A;
  camera_ios.ossds_state.OSSD1B  = m_last_handle.getLocalIOData().ossdsState.stateOSSD1B;
  camera_ios.ossds_state.OSSD2A  = m_last_handle.getLocalIOData().ossdsState.stateOSSD2A;
  camera_ios.ossds_state.OSSD2B  = m_last_handle.getLocalIOData().ossdsState.stateOSSD2B;
  camera_ios.ossds_dyn_count     = m_last_handle.getLocalIOData().ossdsDynCount;
  camera_ios.ossds_crc           = m_last_handle.getLocalIOData().ossdsCRC;
  camera_ios.ossds_io_status     = m_last_handle.getLocalIOData().ossdsIOStatus;
  camera_ios.dynamic_speed_a     = m_last_handle.getLocalIOData().dynamicSpeedA;
  camera_ios.dynamic_speed_b     = m_last_handle.getLocalIOData().dynamicSpeedB;
  camera_ios.dynamic_valid_flags = m_last_handle.getLocalIOData().DynamicValidFlags;
  m_io_pub.publish(camera_ios);
}

void SickSafeVisionary::publishROI()
{
  sick_safevisionary_msgs::ROIArray roi_array_msg;
  for (auto& roi : m_last_handle.getRoiData().roiData)
  {
    sick_safevisionary_msgs::ROI roi_msg;
    roi_msg.id                         = roi.id;
    roi_msg.distance_value             = roi.distanceValue;
    roi_msg.result_data.distance_safe  = roi.result.distanceSafe;
    roi_msg.result_data.distance_valid = roi.result.distanceValid;
    roi_msg.result_data.result_safe    = roi.result.resultSafe;
    roi_msg.result_data.result_valid   = roi.result.resultValid;
    roi_msg.result_data.task_result    = roi.result.taskResult;
    roi_msg.safety_data.invalid_due_to_invalid_pixels =
      roi.safetyRelatedData.tMembers.invalidDueToInvalidPixels;
    roi_msg.safety_data.invalid_due_to_variance =
      roi.safetyRelatedData.tMembers.invalidDueToVariance;
    roi_msg.safety_data.invalid_due_to_overexposure =
      roi.safetyRelatedData.tMembers.invalidDueToOverexposure;
    roi_msg.safety_data.invalid_due_to_underexposure =
      roi.safetyRelatedData.tMembers.invalidDueToUnderexposure;
    roi_msg.safety_data.invalid_due_to_temporal_variance =
      roi.safetyRelatedData.tMembers.invalidDueToTemporalVariance;
    roi_msg.safety_data.invalid_due_to_outside_of_measurement_range =
      roi.safetyRelatedData.tMembers.invalidDueToOutsideOfMeasurementRange;
    roi_msg.safety_data.invalid_due_to_retro_reflector_interference =
      roi.safetyRelatedData.tMembers.invalidDueToRetroReflectorInterference;
    roi_msg.safety_data.contamination_error = roi.safetyRelatedData.tMembers.contaminationError;
    roi_msg.safety_data.quality_class       = roi.safetyRelatedData.tMembers.qualityClass;
    roi_msg.safety_data.slot_active         = roi.safetyRelatedData.tMembers.slotActive;
    roi_array_msg.rois.push_back(roi_msg);
  }
  m_roi_pub.publish(roi_array_msg);
}

void SickSafeVisionary::publishFieldInformation()
{
  sick_safevisionary_msgs::FieldInformationArray field_array_msg;
  for (auto& field : m_last_handle.getFieldInformationData().fieldInformation)
  {
    sick_safevisionary_msgs::FieldInformation field_msg;
    field_msg.field_id     = field.fieldID;
    field_msg.field_set_id = field.fieldSetID;
    field_msg.field_active = field.fieldActive;
    field_msg.field_result = field.fieldResult;
    field_msg.eval_method  = field.evalMethod;
    field_array_msg.fields.push_back(field_msg);
  }
  m_field_pub.publish(field_array_msg);
}
