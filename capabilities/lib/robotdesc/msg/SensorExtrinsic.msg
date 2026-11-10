# One static-mounted sensor's extrinsics: where it sits relative to a
# parent body frame declared by the deployment. Read from the URDF at robotdesc
# startup; consumers (World's camera-to-world projection, multi-cam
# fusion) call `system/robotdesc/sensor_extrinsics` instead of looking up
# tf2 themselves so the dependency graph is auditable via src.

string sensor_name        # deployment-owned stable sensor name
string parent_frame       # body frame in which the transform is expressed
string child_frame        # sensor frame reached by the transform
geometry_msgs/Transform transform   # parent → child
