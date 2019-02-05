# Map Builder

## Gates Ground Truth publisher

Run: 
`$ rosrun drone_map_builder gt_gates_publisher_node.py`
`$ rosrun drone_map_builder gt_ego_position.py`

### Topics
Visualization topic for RViz: `gt_gates_publisher/gt_visualization`
Ground truth gate poses as PoseArray: `gt_gates_publisher/gt_gates`
Rviz position: `gt_ego_position/trajectory_viz_markers`
 
 
## Observed Gates Map

It publishes drone poses into Rviz.

TODO: Build map from gate observation messages

Run:
`$ rosrun drone_map_builder observed_gates_map_node.py` 

