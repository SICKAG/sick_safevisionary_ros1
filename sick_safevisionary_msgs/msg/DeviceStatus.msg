# ROS msg definition for internal DEVICE_STATUS struct
std_msgs/Header header
uint8 status
GeneralStatus general_status
uint32 COP_safety_related
uint32 COP_non_safety_related
uint32 COP_reset_required
ActiveMonitoringCase active_monitoring_case
uint8 contamination_level
