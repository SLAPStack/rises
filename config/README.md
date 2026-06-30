# config/

Configuration is split by subsystem and (per ROS convention) by owning package. This
pointer page maps the §3.2.4 `config/` item to their real locations.

| Location | What |
|---|---|
| [`rises_bringup/config/`](../rises_bringup/config/) | ROS node parameters per scenario: `params_default.yaml` (real robot), `params_rosbag.yaml` (hardware-free replay), `params_unity.yaml`, `central_params.yaml`, `rosbag_qos_override.yaml` |
| [`fiware/config/`](../fiware/config/) | Middleware bridging: `fastdds_profiles.xml` (Fast DDS / XTypes), `dds-enabler.json` (DDS↔NGSI-LD), `orionld.json` (Orion-LD context), `ros_types.xml` (ROS↔DDS type map) |
| [`orchestration/configs/`](../orchestration/configs/) | Scenario configs for the hardware-free run harness — e.g. `rises_demo.yaml` (full ARISE stack), `andros_moving_obstacles_smoke.yaml` (basic demo), `geofence_current_bag.yaml` |
| [`resources/`](../resources/) | RViz layouts (`*.rviz`), `mqtt_bridge_params.yaml`, and the `rises_demo/*.json` map fixtures |

> The DDS-Enabler config (`fiware/config/dds-enabler.json`) is mounted into the AGV
> container at runtime rather than baked into the image — see
> `orchestration/container_manager.py` and
> [`docs/02_interfaces.md`](../docs/02_interfaces.md).
