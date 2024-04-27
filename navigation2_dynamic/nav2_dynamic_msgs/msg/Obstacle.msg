unique_identifier_msgs/UUID uuid
float32 score # detection confidence
geometry_msgs/Point position # center
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 size

# Fields not used by Hungarian Tracker -> needed for Semantic Rules
int32 class_id
string class_name
geometry_msgs/Pose orientation  # We do not use this in tracking, but is kept for semantic rules

# other detections fields that we may start using later
string detection_id
string frame_id