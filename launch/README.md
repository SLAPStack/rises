# launch/

This is a **ROS 2 / colcon workspace**, so launch files live inside the package that
owns them and are installed from there (`ament` convention). This pointer page maps the
§3.2.4 `launch/` item to their real locations.

## Aggregated bring-up — [`rises_bringup/launch/`](../rises_bringup/launch/)

| Launch file | Brings up |
|---|---|
| `geofence.launch.py` | Core geofence stack for a single AGV |
| `rises_geofence.launch.py` | **Full ARISE stack** — geofence + skill bridge + mission controller + heatmap + FIWARE bridge |
| `multi_agv_geofence.launch.py` | Multi-AGV (namespaced `agv_0..agv_N`) in one container |
| `central.launch.py` | Central translator + rosbag player |
| `unity.launch.py` | ROS-TCP-Endpoint + translator (Unity bridge) |

## Per-package launch files

`rises_skill_bridge/launch/`, `rises_mission_controller/launch/`,
`rises_leg_filter/launch/`, `fiware_bridge/launch/`,
`laserscan_preprocessor/launch/`, `message_translator/launch/`,
`obstacle_heatmap_predictor/launch/`.

> The hardware-free demos do not invoke these launch files directly — the
> `orchestration/` harness selects the right one per container. See
> [`docs/03_installation_and_hello_world.md`](../docs/03_installation_and_hello_world.md).
