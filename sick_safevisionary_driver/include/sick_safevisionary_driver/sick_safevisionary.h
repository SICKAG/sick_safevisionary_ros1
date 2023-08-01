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

#include <sick_safevisionary_base/SafeVisionaryData.h>
#include <sick_safevisionary_base/SafeVisionaryDataStream.h>
#include <sick_safevisionary_driver/compound_publisher.h>

#include <boost/lockfree/policies.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <ros/ros.h>
#include <thread>


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

  CompoundPublisher compound_publisher_;

  std::shared_ptr<visionary::SafeVisionaryData> data_handle_;
  std::shared_ptr<visionary::SafeVisionaryDataStream> data_stream_;

  ros::NodeHandle priv_nh_;
  std::string frame_id_;
  int udp_port_;
  std::unique_ptr<std::thread> receive_thread_ptr_;
  std::unique_ptr<std::thread> publish_thread_ptr_;
  std::atomic<bool> data_available_;

  boost::lockfree::spsc_queue<visionary::SafeVisionaryData, boost::lockfree::capacity<10> >
    spsc_queue_;
};

#endif /* SICK_SAFEVISIONARY_DRIVER_SICK_SAFEVISIONARY_H_INCLUDED */
