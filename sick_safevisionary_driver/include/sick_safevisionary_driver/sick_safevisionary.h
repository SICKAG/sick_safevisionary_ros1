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
  /*!
   * \brief Starts the driver functionality
   */
  void run();
  /*!
   * \brief Stops the driver functionality
   */
  void stop();

private:
  /*!
   * \brief Thread for receiving sensor data
   */
  void receiveThread();
  /*!
   * \brief Thread for publishing sensor data
   */
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
