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

#include <sick_safevisionary_driver/sick_safevisionary.h>
#include <sick_safevisionary_msgs/CameraIOs.h>
#include <sick_safevisionary_msgs/DeviceStatus.h>
#include <sick_safevisionary_msgs/ROI.h>
#include <sick_safevisionary_msgs/ROIArray.h>
#include <thread>

SickSafeVisionary::SickSafeVisionary()
  : priv_nh_("~/")
  , data_available_(false)
{
  priv_nh_.param<std::string>("frame_id", frame_id_, "camera");
  header_.frame_id = frame_id_;
  priv_nh_.param<int>("udp_port", udp_port_, 6060);

  data_handle_ = std::make_shared<visionary::SafeVisionaryData>();
  data_stream_ = std::make_shared<visionary::SafeVisionaryDataStream>(data_handle_);

  camera_info_pub_ = priv_nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
  pointcloud_pub_  = priv_nh_.advertise<sensor_msgs::PointCloud2>("points", 1);
  imu_pub_         = priv_nh_.advertise<sensor_msgs::Imu>("imu_data", 1);
  io_pub_          = priv_nh_.advertise<sick_safevisionary_msgs::CameraIOs>("camera_io", 1);
  roi_pub_         = priv_nh_.advertise<sick_safevisionary_msgs::ROIArray>("region_of_interest", 1);
  field_pub_ = priv_nh_.advertise<sick_safevisionary_msgs::FieldInformationArray>("fields", 1);
  device_status_pub_ =
    priv_nh_.advertise<sick_safevisionary_msgs::DeviceStatus>("device_status", 1);

  image_transport::ImageTransport image_transport(priv_nh_);
  depth_pub_     = image_transport.advertise("depth", 1);
  intensity_pub_ = image_transport.advertise("intensity", 1);
  state_pub_     = image_transport.advertise("state", 1);

  if (data_stream_->openUdpConnection(htons((int)udp_port_)))
  {
    ROS_INFO_STREAM("Opening UDP Connection to Sensor.");
    bool waiting_for_connection = true;
    while (waiting_for_connection)
    {
      ROS_INFO_STREAM("Waiting for camera connection.");
      if (data_stream_->getNextBlobUdp())
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
  receive_thread_ptr_ = std::make_unique<std::thread>(&SickSafeVisionary::receiveThread, this);
  publish_thread_ptr_ = std::make_unique<std::thread>(&SickSafeVisionary::publishThread, this);
}

void SickSafeVisionary::stop()
{
  receive_thread_ptr_->join();
  publish_thread_ptr_->join();
  data_stream_->closeUdpConnection();
}

void SickSafeVisionary::receiveThread()
{
  while (ros::ok())
  {
    if (data_stream_->getNextBlobUdp())
    {
      spsc_queue_.push(*data_handle_);
      data_available_ = true;
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
    if (data_available_)
    {
      spsc_queue_.pop(last_handle_);
      processFrame();
      data_available_ = false;
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }
}

void SickSafeVisionary::processFrame()
{
  header_.stamp = ros::Time::now();

  if (camera_info_pub_.getNumSubscribers() > 0)
  {
    publishCameraInfo();
  }
  if (pointcloud_pub_.getNumSubscribers() > 0)
  {
    publishPointCloud();
  }
  if (depth_pub_.getNumSubscribers() > 0)
  {
    publishDepthImage();
  }
  if (intensity_pub_.getNumSubscribers() > 0)
  {
    publishIntensityImage();
  }
  if (state_pub_.getNumSubscribers() > 0)
  {
    publishStateMap();
  }
  if (imu_pub_.getNumSubscribers() > 0)
  {
    publishIMUData();
  }
  if (device_status_pub_.getNumSubscribers() > 0)
  {
    publishDeviceStatus();
  }
  if (io_pub_.getNumSubscribers() > 0)
  {
    publishIOs();
  }
  if (roi_pub_.getNumSubscribers() > 0)
  {
    publishROI();
  }
  if (field_pub_.getNumSubscribers() > 0)
  {
    publishFieldInformation();
  }
}

void SickSafeVisionary::publishCameraInfo()
{
  sensor_msgs::CameraInfo camera_info;
  camera_info.header = header_;

  camera_info.height = last_handle_.getHeight();
  camera_info.width  = last_handle_.getWidth();
  camera_info.D      = std::vector<double>(5, 0);
  camera_info.D[0]   = last_handle_.getCameraParameters().k1;
  camera_info.D[1]   = last_handle_.getCameraParameters().k2;
  camera_info.D[2]   = last_handle_.getCameraParameters().p1;
  camera_info.D[3]   = last_handle_.getCameraParameters().p2;
  camera_info.D[4]   = last_handle_.getCameraParameters().k3;

  camera_info.K[0] = last_handle_.getCameraParameters().fx;
  camera_info.K[2] = last_handle_.getCameraParameters().cx;
  camera_info.K[4] = last_handle_.getCameraParameters().fy;
  camera_info.K[5] = last_handle_.getCameraParameters().cy;
  camera_info.K[8] = 1;
  // TODO add missing parameter in Projection Matrix
  camera_info_pub_.publish(camera_info);
}

void SickSafeVisionary::publishPointCloud()
{
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header       = header_;
  cloud_msg->height       = last_handle_.getHeight();
  cloud_msg->width        = last_handle_.getWidth();
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
  last_handle_.generatePointCloud(point_vec);
  last_handle_.transformPointCloud(point_vec);

  std::vector<uint16_t>::const_iterator intensity_it = last_handle_.getIntensityMap().begin();
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
  pointcloud_pub_.publish(cloud_msg);
}

void SickSafeVisionary::publishIMUData()
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header                = header_;
  imu_msg.angular_velocity.x    = last_handle_.getIMUData().angularVelocity.X;
  imu_msg.angular_velocity.y    = last_handle_.getIMUData().angularVelocity.Y;
  imu_msg.angular_velocity.z    = last_handle_.getIMUData().angularVelocity.Z;
  imu_msg.linear_acceleration.x = last_handle_.getIMUData().acceleration.X;
  imu_msg.linear_acceleration.y = last_handle_.getIMUData().acceleration.Y;
  imu_msg.linear_acceleration.z = last_handle_.getIMUData().acceleration.Z;
  imu_msg.orientation.x         = last_handle_.getIMUData().orientation.X;
  imu_msg.orientation.y         = last_handle_.getIMUData().orientation.Y;
  imu_msg.orientation.z         = last_handle_.getIMUData().orientation.Z;
  imu_msg.orientation.w         = last_handle_.getIMUData().orientation.W;
  imu_pub_.publish(imu_msg);
}

void SickSafeVisionary::publishDepthImage()
{
  depth_pub_.publish(Vec16ToImage(last_handle_.getDistanceMap()));
}

void SickSafeVisionary::publishIntensityImage()
{
  intensity_pub_.publish(Vec16ToImage(last_handle_.getIntensityMap()));
}

void SickSafeVisionary::publishStateMap()
{
  state_pub_.publish(Vec8ToImage(last_handle_.getStateMap()));
}

sensor_msgs::ImagePtr SickSafeVisionary::Vec16ToImage(std::vector<uint16_t> vec)
{
  cv::Mat image = cv::Mat(last_handle_.getHeight(), last_handle_.getWidth(), CV_16UC1);
  std::memcpy(image.data, vec.data(), vec.size() * sizeof(uint16_t));
  return cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_16UC1, image).toImageMsg();
}
sensor_msgs::ImagePtr SickSafeVisionary::Vec8ToImage(std::vector<uint8_t> vec)
{
  cv::Mat image = cv::Mat(last_handle_.getHeight(), last_handle_.getWidth(), CV_8UC1);
  std::memcpy(image.data, vec.data(), vec.size() * sizeof(uint8_t));
  return cv_bridge::CvImage(header_, sensor_msgs::image_encodings::TYPE_8UC1, image).toImageMsg();
}


void SickSafeVisionary::publishDeviceStatus()
{
  sick_safevisionary_msgs::DeviceStatus status;
  status.status = static_cast<uint8_t>(last_handle_.getDeviceStatus());
  status.general_status.application_error =
    last_handle_.getDeviceStatusData().generalStatus.applicationError;
  status.general_status.contamination_error =
    last_handle_.getDeviceStatusData().generalStatus.contaminationError;
  status.general_status.contamination_warning =
    last_handle_.getDeviceStatusData().generalStatus.contaminationWarning;
  status.general_status.dead_zone_detection =
    last_handle_.getDeviceStatusData().generalStatus.deadZoneDetection;
  status.general_status.device_error = last_handle_.getDeviceStatusData().generalStatus.deviceError;
  status.general_status.temperature_warning =
    last_handle_.getDeviceStatusData().generalStatus.temperatureWarning;
  status.general_status.run_mode_active =
    last_handle_.getDeviceStatusData().generalStatus.runModeActive;
  status.general_status.wait_for_cluster =
    last_handle_.getDeviceStatusData().generalStatus.waitForCluster;
  status.general_status.wait_for_input =
    last_handle_.getDeviceStatusData().generalStatus.waitForInput;
  status.COP_non_safety_related = last_handle_.getDeviceStatusData().COPNonSaftyRelated;
  status.COP_safety_related     = last_handle_.getDeviceStatusData().COPSaftyRelated;
  status.COP_reset_required     = last_handle_.getDeviceStatusData().COPResetRequired;
  status.active_monitoring_case.monitoring_case_1 =
    last_handle_.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase1;
  status.active_monitoring_case.monitoring_case_2 =
    last_handle_.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase2;
  status.active_monitoring_case.monitoring_case_3 =
    last_handle_.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase3;
  status.active_monitoring_case.monitoring_case_4 =
    last_handle_.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase4;
  status.contamination_level = last_handle_.getDeviceStatusData().contaminationLevel;
  device_status_pub_.publish(status);
}

void SickSafeVisionary::publishIOs()
{
  sick_safevisionary_msgs::CameraIOs camera_ios;
  camera_ios.configured.pin_5 =
    last_handle_.getLocalIOData().universalIOConfigured.configuredUniIOPin5;
  camera_ios.configured.pin_6 =
    last_handle_.getLocalIOData().universalIOConfigured.configuredUniIOPin6;
  camera_ios.configured.pin_7 =
    last_handle_.getLocalIOData().universalIOConfigured.configuredUniIOPin7;
  camera_ios.configured.pin_8 =
    last_handle_.getLocalIOData().universalIOConfigured.configuredUniIOPin8;
  camera_ios.direction.pin_5 =
    last_handle_.getLocalIOData().universalIODirection.directionValueUniIOPin5;
  camera_ios.direction.pin_6 =
    last_handle_.getLocalIOData().universalIODirection.directionValueUniIOPin6;
  camera_ios.direction.pin_7 =
    last_handle_.getLocalIOData().universalIODirection.directionValueUniIOPin7;
  camera_ios.direction.pin_8 =
    last_handle_.getLocalIOData().universalIODirection.directionValueUniIOPin8;
  camera_ios.input_values.pin_5 =
    last_handle_.getLocalIOData().universalIOInputValue.logicalValueUniIOPin5;
  camera_ios.input_values.pin_6 =
    last_handle_.getLocalIOData().universalIOInputValue.logicalValueUniIOPin6;
  camera_ios.input_values.pin_7 =
    last_handle_.getLocalIOData().universalIOInputValue.logicalValueUniIOPin7;
  camera_ios.input_values.pin_8 =
    last_handle_.getLocalIOData().universalIOInputValue.logicalValueUniIOPin8;
  camera_ios.output_values.pin_5 =
    last_handle_.getLocalIOData().universalIOOutputValue.localOutput1Pin5;
  camera_ios.output_values.pin_6 =
    last_handle_.getLocalIOData().universalIOOutputValue.localOutput2Pin6;
  camera_ios.output_values.pin_7 =
    last_handle_.getLocalIOData().universalIOOutputValue.localOutput3Pin7;
  camera_ios.output_values.pin_8 =
    last_handle_.getLocalIOData().universalIOOutputValue.localOutput4Pin8;
  camera_ios.ossds_state.OSSD1A  = last_handle_.getLocalIOData().ossdsState.stateOSSD1A;
  camera_ios.ossds_state.OSSD1B  = last_handle_.getLocalIOData().ossdsState.stateOSSD1B;
  camera_ios.ossds_state.OSSD2A  = last_handle_.getLocalIOData().ossdsState.stateOSSD2A;
  camera_ios.ossds_state.OSSD2B  = last_handle_.getLocalIOData().ossdsState.stateOSSD2B;
  camera_ios.ossds_dyn_count     = last_handle_.getLocalIOData().ossdsDynCount;
  camera_ios.ossds_crc           = last_handle_.getLocalIOData().ossdsCRC;
  camera_ios.ossds_io_status     = last_handle_.getLocalIOData().ossdsIOStatus;
  camera_ios.dynamic_speed_a     = last_handle_.getLocalIOData().dynamicSpeedA;
  camera_ios.dynamic_speed_b     = last_handle_.getLocalIOData().dynamicSpeedB;
  camera_ios.dynamic_valid_flags = last_handle_.getLocalIOData().DynamicValidFlags;
  io_pub_.publish(camera_ios);
}

void SickSafeVisionary::publishROI()
{
  sick_safevisionary_msgs::ROIArray roi_array_msg;
  for (auto& roi : last_handle_.getRoiData().roiData)
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
  roi_pub_.publish(roi_array_msg);
}

void SickSafeVisionary::publishFieldInformation()
{
  sick_safevisionary_msgs::FieldInformationArray field_array_msg;
  for (auto& field : last_handle_.getFieldInformationData().fieldInformation)
  {
    sick_safevisionary_msgs::FieldInformation field_msg;
    field_msg.field_id     = field.fieldID;
    field_msg.field_set_id = field.fieldSetID;
    field_msg.field_active = field.fieldActive;
    field_msg.field_result = field.fieldResult;
    field_msg.eval_method  = field.evalMethod;
    field_array_msg.fields.push_back(field_msg);
  }
  field_pub_.publish(field_array_msg);
}
