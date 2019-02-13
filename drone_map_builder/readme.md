# Map Builder Package
 
## Observed Gates Map

Build a map of gates based on visual observations.

Run:
`$ rosrun drone_map_builder observed_gates_map_node.py` 


### TODO: 
- Custom Map messages
- Move params to YAML file
- Delete ghost gates (the ones with very low confidence detected long time ago)
- Set maximum observations to stop reconstructing map: When the map is well known, we don't need to process more observations. Uncertainty can't be lowered and we will be wasting computing resources.
- Navigate gates to setup direction of the tracks and nodes connections, based on current ego pose and depth. Detect closed loop. Stop computing when it's a well known network.


## Ground Truth publishers

Run: 

`$ rosrun drone_map_builder gt_gates_publisher_node.py`

`$ rosrun drone_map_builder gt_ego_position.py`

### Topics

- Visualization topic for RViz: `gt_gates_publisher/gt_visualization`
- Ground truth gate poses as PoseArray: `gt_gates_publisher/gt_gates`
- Rviz position: `gt_ego_position/trajectory_viz_markers`
 
 
## Observed Gates Map

Run:`$ rosrun drone_map_builder observed_gates_map_node.py` 
>>>>>>> aa875e3efe4d1bbbc1e2baf81b31724a9008d283

### TODO:
- RMSE Error calculation from gates ground truth: error per gate (using data assoc), detect missing gates, detect ghost gates, total error of the map.  