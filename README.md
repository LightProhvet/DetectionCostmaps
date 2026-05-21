# DetectionCostmaps

A ROS 2 (Humble) workspace for semantic object detection-driven navigation costmaps. Converts real-time object detections into dynamic Nav2 obstacles using configurable semantic rules — enabling indoor robots to navigate safely around people and dynamic objects.

## Packages

| Package | Description |
|---|---|
| [`semantic_rules`](https://github.com/LightProhvet/semantic_rules) | Nav2 costmap_2d plugin — core library |
| `yolov8_ros` | YOLOv8 object detection node |
| `navigation2_dynamic` | Kalman Filter + Hungarian algorithm multi-object tracker and message definitions |
| `rules_bringup` | Launch files for the full pipeline |

## Prerequisites

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Nav2](https://docs.nav2.org/getting_started/index.html): `sudo apt install ros-humble-nav2-*`
- [vcstool](https://github.com/dirk-thomas/vcstool): `pip install vcstool`
- [rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html): `sudo apt install python3-rosdep`

## Setup

```bash
# Clone the workspace
git clone https://github.com/LightProhvet/DetectionCostmaps.git
cd DetectionCostmaps

# Pull all packages into src/
vcs import src < semantic_rules.repos

# Install ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

## Usage

```bash
# Full pipeline: detection → tracking → costmap
ros2 launch rules_bringup costmap_rules.launch.py

# Detection to obstacle conversion only
ros2 launch rules_bringup detection_to_obstacle.launch.py
```

## Pipeline

```
Camera input
    ↓
YOLOv8 detection        (yolov8_ros)
    ↓
Kalman Filter tracker   (navigation2_dynamic — kf_hungarian_tracker)
    ↓
Detection converter     (semantic_rules)
    ↓
Semantic layer plugin   (semantic_rules — Nav2 costmap_2d plugin)
    ↓
Nav2 costmap            (dynamic obstacle layer for path planning)
```

## Roadmap

- [ ] Simulation environment (Gazebo/Isaac)
- [ ] ML-based rule optimisation
- [ ] Unit and integration tests
- [ ] ROS Index release

## License

Apache 2.0 — see [LICENSE](LICENSE).
