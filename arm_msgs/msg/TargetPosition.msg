# TargetPosition.msg
# Custom message for tracking 3D target positions with distances

string name           # Target identifier/name
geometry_msgs/Point position
float32 distance     # Straight-line distance in meters
