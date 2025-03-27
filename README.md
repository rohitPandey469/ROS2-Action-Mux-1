# ROS2 Action Priority Mux System

A ROS2 package implementing a priority-based action multiplexer system.

## Installation

### Clone and build the repository:
```bash
git clone https://github.com/rohitPandey469/ROS2-Action-Mux-1
cd ROS2-Action-Mux-1
source /opt/ros/humble/setup.bash 
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build
source install/setup.bash
```
### Terminal 1 (High priority Server)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run priority_action_system high_priority_server
```

### Terminal 2 (Low Priority Server)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run priority_action_system low_priority_server
```

### Terminal 3 for Priority Client
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run priority_action_system priority_client
```

### Terminal 4 task_command Publisher
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub /task_commands std_msgs/msg/Int32 "{data: 0}"  # Low priority task
ros2 topic pub /task_commands std_msgs/msg/Int32 "{data: 1}"  # High priority task
```
