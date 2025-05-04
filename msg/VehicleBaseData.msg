# VehicleBaseData.msg

std_msgs/Header header
int32 vin
geometry_msgs/PointStamped position
float64 direction # in °
float64 speed
int8 indicator_state  # (0 = off, 1 = left, 2 = right, 3 = warning)
int8 engine_state     # (0 = on, 1 = off)
