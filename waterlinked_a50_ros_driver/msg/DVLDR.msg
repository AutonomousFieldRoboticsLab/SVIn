#Header
std_msgs/Header header

#Time
float64 time

#Measured position [m]
geometry_msgs/Vector3 position

#Estimated position
float64 pos_std

#IMU sensor
float64 roll
float64 pitch
float64 yaw

#Message type
string type

#Status message
int64 status

#Formatting of json
string format
