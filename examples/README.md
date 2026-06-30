# examples/

Minimal inputs and helper payloads for the **hello-world** and the **basic demo**.
The full, step-by-step run instructions live in the docs — this folder is the
"where to start" index plus the small sample payloads the demos consume.

## Start here

| Goal | Read | Run with |
|---|---|---|
| Prove the install (hardware-free hello-world) | [`docs/03_installation_and_hello_world.md`](../docs/03_installation_and_hello_world.md) | `python3 -m orchestration.cli -c orchestration/configs/andros_moving_obstacles_smoke.yaml` |
| See the geofence detect intruders + export KPIs (basic demo) | [`docs/04_basic_demo_how_to_use.md`](../docs/04_basic_demo_how_to_use.md) | same config; KPIs land in `logs/kpis_*.csv` |
| Drive the **full ARISE stack** (skill bridge + mission controller) via intents | [`docs/INTERACT_FULL_ARISE.md`](../docs/INTERACT_FULL_ARISE.md) | `orchestration/configs/rises_demo.yaml` + the intent helper below |

## Recorded bag (Release asset)

The demos replay a recorded warehouse run. The bag is ~1 GB — too large for git — so it
is published as a **GitHub Release asset** rather than committed:

- Asset: `arise_andros_moving_obstacles_20260505_223330.tar.gz`
- Download: <https://github.com/SLAPStack/rises/releases>

Extract it into `resources/` and point the scenario config's `bag_file` at the
extracted directory:

```bash
mkdir -p resources
tar -xzf arise_andros_moving_obstacles_20260505_223330.tar.gz -C resources/
# then set `bag_file:` in your chosen orchestration/configs/*.yaml to the extracted path
```

## Sample payloads in this folder

- [`sample_intent.json`](sample_intent.json) — the `data` payload of the ARISE
  `START_ACTIVITY` intent that drives the mission controller (lock an area). The
  recommended way to send it is the helper, which fills the full
  `hri_actions_msgs/Intent` message correctly:

  ```bash
  # inside the running AGV container (hri_actions_msgs is available there)
  docker exec -it agv_0 python3 /workspace/scripts/publish_intent.py lock_area --area 1
  ```

  `publish_intent.py` accepts `lock_area`, `unlock_area`, and `update_map`
  (see [`scripts/publish_intent.py`](../scripts/publish_intent.py)).

## Map / obstacle fixtures

The geofence's pre-init map fixtures used by the demo scenarios:

- [`../resources/rises_demo/warehouse_contours.json`](../resources/rises_demo/warehouse_contours.json) — warehouse boundary + static structure
- [`../resources/rises_demo/obstacles.json`](../resources/rises_demo/obstacles.json) — known static obstacles
