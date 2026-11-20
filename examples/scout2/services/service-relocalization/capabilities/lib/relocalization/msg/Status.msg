# active describes lifecycle availability, not successful localization.
string process_id
string request_id
bool active
bool busy
bool localization_valid
# Empty/zero identity means no snapshot has been verified, even when active.
string map_id
uint64 generation
string database_sha256
# ROS-clock publication time; a consumer must also expire samples locally.
uint64 published_stamp_ns
# Acquisition time of the observations used for localization; zero if absent.
uint64 observation_stamp_ns
# map-frame planar pose, meters/radians; unusable unless localization_valid.
float64 x
float64 y
float64 theta
# residual in meters (-1 if absent); overlap in [0,1], not a probability.
float64 residual_m
float64 overlap
bool ambiguous
string detail
