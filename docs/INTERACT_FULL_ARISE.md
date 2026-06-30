# Full ARISE mode — reviewer interaction guide

This describes how to run the hardware-free bag demo with the **full ARISE
integration layer** (geofence + **skill bridge** + **mission controller** + heatmap +
FIWARE bridge) and how to drive it through **intents**, **skills**, and **tasks**.

## 1. Start the stack

```bash
python3 -m orchestration.cli -c orchestration/configs/rises_demo.yaml
```

This brings up one AGV container (`agv_0`) running `rises_geofence.launch.py` plus the
central bridge replaying `resources/discerning_safety_humble`, and the FIWARE stack.
The geofence node auto-activates (lifecycle ACTIVE), so its services are live and the
skill bridge can call them.

All ROS 2 commands below run **inside the AGV container** with the workspace sourced:

```bash
docker exec -it agv_0 bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
```

## 2. Confirm the full layer is up and healthy

```bash
ros2 node list            # expect /agv_0/geofence_node, /agv_0/skill_bridge_node, /agv_0/rises_mission_controller
ros2 lifecycle get /agv_0/geofence_node          # expect: active
ros2 action list | grep /agv_0/skill            # expect the 9 skills below
ros2 topic echo /agv_0/mission_status            # leave running in a 2nd shell to watch missions
```

The 9 skill action servers (`/agv_0/skill/*`): `validate_path`, `update_map`,
`set_area_state`, `set_safety_radius`, `update_warehouse_layout`, `get_area_state`,
`get_safety_radius`, `get_map_info`, `set_geofence_enabled`.

## 3. Drive it through INTENTS (mission controller path)

The mission controller subscribes to the **global** `/intents` topic and routes
`START_ACTIVITY` intents whose `data` JSON contains `lock_area`, `unlock_area`, or
`update_map`. The Intent→Mission→Task→Skill→geofence chain runs and reports on
`/agv_0/mission_status` (ACTIVE → SUCCEEDED).

### Easiest: the helper

```bash
# inside agv_0 (copy the script in first if the repo is not mounted):
#   docker cp scripts/publish_intent.py agv_0:/tmp/
python3 /tmp/publish_intent.py lock_area --area 1     # lock area 1
python3 /tmp/publish_intent.py unlock_area --area 1   # unlock it
python3 /tmp/publish_intent.py update_map             # query map info + safety radius
```

Watch the `mission_status` echo from step 2: you should see a mission go
`ACTIVE` then `SUCCEEDED`, and the skill bridge log a `set_area_state` call on the
geofence.

### Raw alternative (no helper)

`Intent.intent` is a string equal to the message's `START_ACTIVITY` constant — look up
its exact value first, then publish:

```bash
ros2 interface show hri_actions_msgs/msg/Intent   # find the START_ACTIVITY constant value
ros2 topic pub --once /intents hri_actions_msgs/msg/Intent \
  "{intent: '<START_ACTIVITY value from above>', \
    data: '{\"goal\":\"lock_area\",\"area_id\":1}', \
    source: 'reviewer', modality: 'cli', confidence: 1.0, priority: 0}"
```

## 4. Drive a SKILL directly (bypass the mission controller)

```bash
ros2 action send_goal /agv_0/skill/get_map_info     rises_interfaces/action/GetMapInfo "{}"
ros2 action send_goal /agv_0/skill/set_area_state   rises_interfaces/action/SetAreaState "{area_id: 1, lock: true}"
ros2 action send_goal /agv_0/skill/set_safety_radius rises_interfaces/action/SetSafetyRadius "{radius: 1.0}"
```

## 5. Base demo is unaffected

The geofence keeps publishing `/agv_0/obstacle_report` and `/agv_0/obstacle_alert`, the
FIWARE bridge keeps updating `urn:ngsi-ld:AGV:agv_0` in Orion-LD, and Grafana
(http://localhost:3000, admin/admin) shows live state.

## 7. Tear down

```bash
# Ctrl-C the orchestrator, then:
docker compose -f fiware/docker-compose.yaml -p rises-fiware down -v
```

## Notes
- `/intents` is **global** (not namespaced); `/skill/*`, `mission_status`, and the
  geofence services are under the `agv_0` namespace.
- With more than one AGV, every mission controller listens to the same global
  `/intents`; per-AGV intent routing is future work.
