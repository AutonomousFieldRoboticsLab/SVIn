# dvl_msgs 
ROS2 custom message structure for use in the Water Linked DVL-A50 Sensor.
## Messages (.msg)
- [DVL](msg/DVL.msg): Holds speed, altitude and diagnostic information.
- [DVLBeam](msg/DVLBeam.msg): Data structure for each beam.
- [DVLDR](msg/DVLDR.msg): Holds data from an IMU (Inertial Measurement Unit) and position.
- [ConfigCommand](msg/ConfigCommand.msg): Command structure used to configurate the sensor.
- [ConfigStatus](msg/ConfigStatus.msg): Shows the current status of the sensor parameters.
- [CommandResponse](msg/CommandResponse.msg): Response after using [ConfigCommand](msg/ConfigCommand.msg).

