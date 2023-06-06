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


class SickSafeVisionaryROS
{
public:
  SickSafeVisionaryROS();
  virtual ~SickSafeVisionaryROS(){};
private:
  std::shared_ptr<visionary::SafeVisionaryData> m_data_handle;
  std::shared_ptr<visionary::SafeVisionaryDataStream> m_data_stream;

  // TODO not sure if necessary
  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;

};

#endif /* SICK_SAFEVISIONARY_ROS_SICK_SAFE_VISIONARY_H_INCLUDED */
