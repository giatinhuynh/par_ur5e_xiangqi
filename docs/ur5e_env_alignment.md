# UR5e_Env vs `par_ur5e_xiangqi` codebase alignment

This note ties the **bundled reference tree** [`UR5e_Env-main/`](../UR5e_Env-main/) and a normal **`~/UR5e_Env`** clone to this repository’s expectations. Use it when integrating or debugging topic/action mismatches.

---

## Docker and workspace layout

| Item | UR5e_Env (`UR5e_Env-main`) | This repo |
|------|---------------------------|-----------|
| Compose service | `ros2` → **`./workspace` → `/home/rosuser/workspace`** | You **`cp workspace/src/*`** into that mounted tree; `~/workspace` inside the container matches [`README.md`](../README.md). |
| Image tag after `./docker-build.sh` | **`ros:humble`** ([`docker-compose.yml`](../UR5e_Env-main/docker-compose.yml)) | [`Dockerfile`](../Dockerfile) defaults to **`FROM ros:humble`** so the Xiangqi layer builds **on top of the lab image**. |
| Extended runtime | Compose still says `image: ros:humble` until you switch it | Point Compose at **`ur5e_xiangqi:latest`** (and drop `build:` if needed) — see root **README Quick start**. |

---

## Lab aliases (helper scripts)

Defined in **`workspace/.helper_scripts/helper-aliases.sh`** (not `testing_scripts/`):

| Alias | Script |
|-------|--------|
| `arm_drivers` | `drivers-arm-start.sh` |
| `moveit_config_driver` | `driver-moveit-start.sh` |
| `realsense_driver` | `driver-realsense-start.sh` |
| `find_object_2d` | `find_object_2d.sh` |
| `build_workspace` | `build-workspace.sh` |

[`drivers-arm-start.sh`](../UR5e_Env-main/workspace/.helper_scripts/drivers-arm-start.sh) passes **`robot_ip:=10.234.6.49`**, matching [`ur5evxlabdoc.md`](../ur5evxlabdoc.md).

Older lab text may say **`ur_driver`**; current drivers entrypoint is **`arm_drivers`**.

---

## ROS interfaces used by Xiangqi (verified against `UR5e_Env-main`)

### MoveIt arm motion

| | UR5e_Env source | Xiangqi consumer |
|---|-----------------|------------------|
| **OMPL (all moves)** | **`move_group`** → **`/move_action`** (`moveit_msgs/action/MoveGroup`) — same as RViz | [`moveit_ompl_client.py`](../workspace/src/xiangqi_manipulation/xiangqi_manipulation/moveit_ompl_client.py) |
| Not used by Xiangqi | **`/par_moveit/waypoint_move`** (Cartesian only; lab legacy) | — |

### RG2 gripper

| | UR5e_Env | Xiangqi |
|---|----------|---------|
| Action | **`/rg2/set_width`** (`GripperSetWidth`) | [`manipulation_node.py`](../workspace/src/xiangqi_manipulation/xiangqi_manipulation/manipulation_node.py), [`gripper_controller_node.py`](../workspace/src/xiangqi_manipulation/xiangqi_manipulation/gripper_controller_node.py) |
| State | **`/rg2/state`** | `gripper_controller_node` subscription |

### Camera (RealSense)

Drivers are launched from **`par_pkg`** when **`enable_camera:=true`** in `arm_drivers`. The Xiangqi stack expects **`/camera/color/image_raw`** ([`vision_config.yaml`](../workspace/src/xiangqi_bringup/config/vision_config.yaml)). If your lab remaps the topic, change **`camera_topic`** there.

---

## Packages: what stays from UR5e_Env vs what you copy from this repo

**Stay on the lab volume** (already under `~/UR5e_Env/workspace/src/`):  
`par_pkg`, `par_interfaces`, `par_moveit_config`, `onrobot_rg2_*`, etc.

**Copy from this repo** into `workspace/src/`:  
`xiangqi_*` packages only ([`README.md`](../README.md) Quick start).

`xiangqi_vision` still uses **`par_interfaces/srv/CurrentWaypointPose`** (optional calibration fallback). **`par_interfaces`** must remain built from UR5e_Env. Xiangqi arm motion uses **`moveit_msgs`** + **`/move_action`**, not `WaypointMove`.

---

## Extra dependencies (not in stock UR5e image)

Installed by this repo’s **`Dockerfile`** on top of the lab base:  
**py_trees**, **py_trees_ros**, **ultralytics**, **pyffish**, **Flask/SocketIO**, **Fairy-Stockfish** build, etc.

Without that extended image, `ros2 launch xiangqi_bringup …` will miss libraries unless installed manually.

---

## Safety monitor caveat

[`safety_monitor_node`](../workspace/src/xiangqi_manipulation/xiangqi_manipulation/safety_monitor_node.py) subscribes to **`/ur_hardware_interface/safety_mode`** when `ur_dashboard_msgs` is importable. Availability depends on the **UR ROS 2 driver** stack your launch brings up—not asserted inside `UR5e_Env-main` sources here. If the topic is absent, e-stop from UR hardware may not propagate; dashboard **`/xiangqi/emergency_stop`** still works.

---

## Bundled `UR5e_Env-main` folder

It is a **reference snapshot** only (see root README). Behaviour should match a **`git clone`** of [Kibibibit/UR5e_Env](https://github.com/Kibibibit/UR5e_Env) at the revision you froze; always verify **`docker-compose.yml`** and helper scripts on the machine you actually run.
