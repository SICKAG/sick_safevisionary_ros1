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
#include <thread>

SickSafeVisionary::SickSafeVisionary()
  : priv_nh_("~/")
  , data_available_(false)
{
  priv_nh_.param<std::string>("frame_id", frame_id_, "camera");
  priv_nh_.param<int>("udp_port", udp_port_, 6060);

  data_handle_ = std::make_shared<visionary::SafeVisionaryData>();
  data_stream_ = std::make_shared<visionary::SafeVisionaryDataStream>(data_handle_);

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
      visionary::SafeVisionaryData frame_data;
      spsc_queue_.pop(frame_data);
      std_msgs::Header header;
      header.frame_id = frame_id_;
      header.stamp    = ros::Time::now();
      compound_publisher_.publish(header, frame_data);
      data_available_ = false;
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }
}
