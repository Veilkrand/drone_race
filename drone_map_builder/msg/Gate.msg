# Universal gate representation in a gate map

uint16 id  # Unique ID in the map
string name  # Descriptive unique name
geometry_msgs/Pose pose  # Pose of the gate looking forward
float32 confidence  # Level of confidence [0,1]
geometry_msgs/Vector3 scale  # (optional) Size of the gate in meters. It can be empty or null
uint16 next_gate_id  # (optional) next gate id in a particular direction. It can be empty or null
uint16 prev_gate_id  # (optional) previous gate id in a particular direction. It can be empty or null
