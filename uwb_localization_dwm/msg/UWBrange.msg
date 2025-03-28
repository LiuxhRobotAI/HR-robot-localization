Header header                           # ROS header
string anchor_id
string to_tag_id
geometry_msgs/Point anchor_position     # 3d position of the anchor(if known; explicitly declared as anchor by rosparam or the dwm appliction; otherwise 9999 indicates unknown).
float64 distance_to_tag