int32 COMMAND_SET=0
int32 COMMAND_DELETE=2

string name
string geometry_path
string frame
int32 command 0

float32 geometry_unit_in_meter 1.0
bool geometry_counterclockwise true
bool geometry_enable_culling false
float32[16] geometry2body_pose [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 ]
float32[16] detector_world_pose [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.05, 0.0, 1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0 ]
