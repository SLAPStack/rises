# Orchestration Module

Multi-container ROS2 simulation orchestration supporting central (rosbag) and unity (TCP bridge) deploy modes.

## Architecture

Docker containers are built from `central.dockerfile` (multi-stage):

| Stage   | Image         | Container         | Entrypoint                | Launch file          |
|---------|---------------|-------------------|---------------------------|----------------------|
| `base`  | `rises:base`  | `agv_0`, `agv_1`… | `entrypoint.sh`          | `geofence.launch.py` |
| `central` | `rises:central` | `central_bridge` | `central_entrypoint.sh` | `central.launch.py`  |
| `unity` | `rises:unity` | `unity_bridge`    | `unity_entrypoint.sh`    | `unity.launch.py`    |

## Deploy Modes

**`central`** — Rosbag replay. Bridge container runs centralized translator + rosbag player.

**`unity`** — Unity simulation. Bridge container runs ROS-TCP-Endpoint (port 10000) + centralized translator.

Set via `mode:` in config YAML or `--mode` on CLI.

## AGV Scenarios

Each AGV container selects a parameter file based on `scenario`:

| Scenario  | Params file          | Key differences                        |
|-----------|----------------------|----------------------------------------|
| `default` | `params_default.yaml`| Real robot, no sim time, no transforms |
| `rosbag`  | `params_rosbag.yaml` | `use_sim_time: true`, validation on    |
| `unity`   | `params_unity.yaml`  | Coordinate transforms, validation on   |

Set via `agv.scenario:` in config YAML or `--scenario` on CLI.

## Usage

```bash
# Unity simulation
python3 -m orchestration.cli --config orchestration/configs/unity_sim.yaml --build

# Central/rosbag mode (2 AGVs)
python3 -m orchestration.cli --config orchestration/configs/unity_2speed_2agv_fixedlayout.yaml --build

# Override from CLI
python3 -m orchestration.cli --config orchestration/configs/unity_sim.yaml \
    --mode unity --scenario unity --agvs "agv_0,agv_1"

# Cleanup
python3 -m orchestration.cli --cleanup
```

## CLI Arguments

```
--config PATH            Configuration YAML (required)
--mode central|unity     Deploy mode override
--scenario default|rosbag|unity  AGV scenario override
--agvs "agv_0,agv_1"    Override AGV namespaces
--build                  Build Docker images first
--no-cache               Build without Docker cache
--cleanup                Force-remove containers and exit
--ros-domain-id N        Override ROS_DOMAIN_ID
--bag-file PATH          Override rosbag path
--rosbag-remaps STR      Topic remaps for rosbag
--launch-rviz            Launch RViz in bridge container
--rviz-namespace NS      AGV namespace for RViz
--tcp-ip IP              TCP IP (unity, default: 0.0.0.0)
--tcp-port PORT          TCP port (unity, default: 10000)
--obstacles-json PATH    Pre-init obstacles JSON
--contours-json PATH     Pre-init contours JSON
```

## Config Structure

```yaml
mode: unity              # "central" or "unity"
agv_namespaces: [agv_0]

bridge:                  # (also accepts legacy key "central")
  tcp_ip: "0.0.0.0"     # unity mode only
  tcp_port: 10000
  enable_buffering: true
  play_rosbag: false

agv:
  scenario: "unity"      # default | rosbag | unity
  publish_unity_tf: true
  log_level: "info"
```

## Structure

```
orchestration/
├── __init__.py
├── config.py                # DeployMode, BridgeConfig, AGVConfig, SimulationConfig
├── container_manager.py     # Docker lifecycle (build, start, stop, cleanup)
├── orchestrator.py          # Main orchestration flow
├── utils.py                 # YAML loading, namespace parsing
├── cli.py                   # CLI entry point
└── configs/
    ├── unity_sim.yaml
    ├── unity_2speed_2agv_fixedlayout.yaml
    └── geofensing_map_updates_rises.yaml
```

## Environment Variable Flow

The orchestrator sets env vars on each Docker container. These flow through the
entrypoint scripts into the launch files and YAML parameter files:

```
Config YAML → orchestrator → docker -e → entrypoint.sh → launch args / $(env …) in YAML
```

Key env vars per container type:

**AGV containers**: `SCENARIO`, `NAMESPACE`, `TF_PREFIX`, `ROBOT_ID`, `GEOFENCE_*`, `LASERSCAN_*`, `PUBLISH_STATIC_TF`, `PUBLISH_UNITY_TF`, etc.

**Bridge containers**: `AGV_COUNT`, `TRANSLATOR_*`, `USE_SIM_TIME`, `ROSBAG_*`, `LAUNCH_RVIZ`, `ROS_TCP_IP` / `ROS_TCP_PORT` (unity only).
