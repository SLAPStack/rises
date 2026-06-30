# RISES System Design

Single reference document for the RISES (geofencing / safety) ROS 2 system and its alignment with the ARISE middleware architecture. Replaces the previous separate documents `ARISE_ARCHITECTURE.md`, `ARISE_INTEGRATION_CHANGES.md`, and `ARISE_MIDDLEWARE.md`.

---

## 1. Overview

The RISES system is a safety / geofencing module for autonomous forklifts in warehouse environments. It detects unknown obstacles (including humans) via LiDAR and enforces safety policies. The system is designed in three layers:

1. **Core System** — runs independently with no ARISE dependencies.
2. **ARISE Integration Layer** — optional overlay that exposes capabilities as ROS4HRI skills and connects to the intent system.
3. **FIWARE IT Layer** — optional infrastructure that bridges ROS 2 data to NGSI-LD for dashboards, digital twins, and historical analysis.

```
┌─────────────────────────────────────────────────────────────┐
│                    ARISE Integration Layer                   │
│                                                              │
│  /intents ──► rises_mission_controller                       │
│                  │ decomposes intent → tasks → skill calls   │
│                  ▼                                           │
│              rises_skill_bridge                              │
│                  │ /skill/validate_path                      │
│                  │ /skill/set_area_state                     │
│                  │ /skill/get_area_state                     │
│                  │ /skill/update_map                         │
│                  │ /skill/update_warehouse_layout            │
│                  │ /skill/set_safety_radius                  │
│                  │ /skill/get_safety_radius                  │
│                  │ /skill/get_map_info                       │
│                  ▼ (service calls to core)                   │
├──────────────────┼───────────────────────────────────────────┤
│                  ▼         Core System                       │
│                                                              │
│  laserscan_preprocessor ──► geofence_node ──► safety_node    │
│  (raw LiDAR → segments)    (scan-to-map     (halt/resume     │
│                              matching)        forklift)      │
│                                                              │
│  message_translator          fleet_interface                 │
│  (SLAPStack JSON ↔ ROS 2)   (VDA5050 orders ↔ paths)         │
├──────────────────┼───────────────────────────────────────────┤
│                  ▼         FIWARE IT Layer                   │
│                                                              │
│  DDS Enabler ──► Orion-LD ──► TimescaleDB ──► Grafana        │
│  (DDS topics      (NGSI-LD    (historical     (dashboards)   │
│   → entities)     broker)      queries)                      │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. ARISE Middleware Reference

### 2.1 What ARISE is

ARISE (Architecture for Robotics in Industrial Shared Environments) is an EU-funded framework that provides middleware for industrial robotics applications. Its goal is to bridge **OT (Operational Technology)** — robots, sensors, actuators on the factory floor — with **IT (Information Technology)** — dashboards, digital twins, analytics, cloud systems.

The middleware is not a single piece of software but a collection of interoperable open-source components. The three technology pillars are:

1. **Vulcanexus** (eProsima) — A ROS 2 distribution built on Fast DDS, used as the real-time communication layer.
2. **FIWARE** — An IT/IoT platform centered on the NGSI-LD Context Broker (Orion-LD), used for data persistence, digital twins, and IT system integration.
3. **ROS4HRI** — A standardized human-robot interaction framework (REP-155), providing perception, dialogue, intents, skills.

```
 OT Layer (Field)               ARISE Middleware                  IT Layer (Cloud/Enterprise)
 ─────────────────       ──────────────────────────────       ──────────────────────────────────
                         ┌─────────────────────────────┐
  Robots / AGVs          │      Vulcanexus (ROS 2)     │       NGSI-LD Context Broker (Orion-LD)
  LiDAR / Cameras  ───►  │      Fast DDS v3+           │  ───► Digital Twins
  Industrial Sensors     │      DDS Router / Enabler   │       Dashboards (Grafana)
  OPC UA Devices         │      ROS4HRI Stack          │       Historical Queries (Mintaka)
                         └─────────────────────────────┘       AI / Analytics
```

Core pattern:
1. OT devices publish data to **ROS 2 topics** via Fast DDS.
2. A **DDS bridge** (DDS Enabler or DDS Router) pushes selected topic data into the **NGSI-LD Context Broker**.
3. IT systems subscribe to context changes via the broker's REST API.
4. Commands from IT systems are mapped back into ROS 2 topics, closing the loop.

### 2.2 Vulcanexus (ROS 2)

Vulcanexus replaces the standard ROS 2 middleware with an enhanced version of Fast DDS (v3+). Required for ARISE.

Key features over standard ROS 2:
- **DDS Keys** — instance-based data tracking on a single topic (critical for HRI, where multiple people are tracked on one topic).
- **Discovery Server v3** — reduces peer-to-peer discovery overhead in large multi-robot deployments.
- **Zero-copy transfers** — shared-memory transport for intra-host communication.
- **DDS Router** — routes DDS traffic between geographically separated DDS networks over WAN/TCP.
- **DDS Enabler** — bridges DDS data into external platforms (FIWARE).

Available meta-packages:

| Package | Contents |
|---------|----------|
| `vulcanexus-core` | Core communication and system libraries |
| `vulcanexus-tools` | Introspection and debugging utilities |
| `vulcanexus-cloud` | DDS Router and cloud communication tools |
| `vulcanexus-micro` | Embedded ROS 2 with Micro XRCE-DDS |
| `vulcanexus-simulation` | Gazebo/Ignition plugins and models |
| `vulcanexus-desktop` | Full package with visualization and simulation |

Docker base image: `eprosima/vulcanexus:jazzy-desktop`

### 2.3 FIWARE / NGSI-LD Context Broker

FIWARE is an open-source platform for building smart applications. In ARISE, it serves as the IT integration layer. The central component is **Orion-LD**, a context broker that stores the current state of all entities (robots, sensors, zones) as NGSI-LD objects and exposes them via a REST API.

Core services:

| Service | Image | Purpose |
|---------|-------|---------|
| Orion-LD | `fiware/orion-ld:1.10.0-PRE-1711` | NGSI-LD Context Broker with DDS integration |
| MongoDB | `mongo:7.0` | Backend storage for Orion-LD |
| TimescaleDB | `timescale/timescaledb-ha:pg17-ts2.18` | Time-series database for historical entity data |
| Mintaka | `fiware/mintaka:0.6.18` | REST API for temporal queries over entity history |
| Grafana | `grafana/grafana:latest` | Dashboards and visualization |

Orion-LD configuration:
- Requires `-wip dds` flag to enable DDS integration.
- Requires `-mongocOnly` flag.
- Config file mounted at `/root/.orionld` inside the container.
- TROE (Temporal Representation of Entities) environment variables for TimescaleDB:
  - `ORIONLD_TROE=TRUE`
  - `ORIONLD_TROE_HOST`, `ORIONLD_TROE_PORT`
  - `ORIONLD_TROE_USER`, `ORIONLD_TROE_PWD`
  - `ORIONLD_MONGO_HOST`

NGSI-LD entity naming convention: `urn:ngsi-ld:<type>:<id>` — e.g. `urn:ngsi-ld:agv:0`, `urn:ngsi-ld:robot:1`, `urn:ngsi-ld:Device:sensor_42`.

NGSI-LD REST API examples:
```bash
# Get entity state
GET http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:agv:0

# Update entity attribute
PATCH http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:agv:0/attrs/position
Content-Type: application/json

# Query historical data via Mintaka
GET http://localhost:8080/temporal/entities/urn:ngsi-ld:agv:0?timeproperty=modifiedAt&timerel=between&timeAt=2026-01-01T00:00:00Z&endTimeAt=2026-01-02T00:00:00Z
```

### 2.4 DDS Enabler

eProsima's bridge between DDS (ROS 2) and the NGSI-LD Context Broker. It is the component that makes the OT/IT convergence work.

How it works:
- Listens to DDS topics on the ROS 2 network.
- Translates DDS samples into NGSI-LD entity updates.
- Pushes updates to Orion-LD via its REST API.
- Can also receive NGSI-LD updates and publish them back into DDS topics (bidirectional).

Two operational phases (per ARISE design):

**Phase I — Legacy Mode:**
- Broker passively listens to existing ROS 2 DDS communication.
- JSON configuration file provides mapping from DDS topics to NGSI-LD entities.
- Non-intrusive: no changes needed to existing ROS 2 code.

**Phase II — NGSI-LD Aware Mode:**
- For systems designed with NGSI-LD in mind.
- Topic names correspond to NGSI-LD attribute longnames.
- Entity ID and type are embedded in the DDS samples themselves.
- No configuration file needed.

Topic-to-entity mapping configuration:
```json
{
  "dds": {
    "ddsmodule": {
      "dds": {
        "domain": 0,
        "transport": "udp"
      }
    },
    "ngsild": {
      "topics": {
        "rt/agv_0/obstacle_report": {
          "entityType": "AGV",
          "entityId": "urn:ngsi-ld:agv:0",
          "attribute": "obstacle_report"
        },
        "rt/agv_0/base_link_pose": {
          "entityType": "AGV",
          "entityId": "urn:ngsi-ld:agv:0",
          "attribute": "position"
        }
      }
    }
  }
}
```

Note: ROS 2 topics get an `rt/` prefix in DDS — `/agv_0/obstacle_report` becomes `rt/agv_0/obstacle_report`.

Repository: https://github.com/eProsima/FIWARE-DDS-Enabler

### 2.5 DDS Router

A separate tool from the DDS Enabler. Routes DDS traffic between isolated DDS networks (e.g. connecting robots at different physical sites over WAN/TCP).

Use cases:
- Connecting ROS 2 robots across geographically separated sites.
- Bridging DDS domains across Kubernetes clusters.
- Mesh networking for distributed robot fleets.

It does **not** bridge to FIWARE — it is DDS-to-DDS only. Use it alongside the DDS Enabler if you need both WAN routing and FIWARE integration.

Configuration (YAML):
```yaml
allowlist:
  - name: "rt/chatter"
    type: "std_msgs::msg::dds_::String_"

internal_participant:
  type: local
  domain: 0

external_participant:
  type: local-discovery-server
  id: 201
  connection-addresses:
    - id: 200
      addresses:
        - domain: "dds-discovery-server"
          port: 11811
          transport: "udp"
```

Repository: https://github.com/eProsima/DDS-Router

### 2.6 ROS4HRI

Framework for standardized human-robot interaction in ROS 2, based on REP-155. Three layers: **perception**, **dialogue/intents**, **skills**.

#### 2.6.1 Perception layer

Detects and tracks humans (faces, bodies, voices) and publishes standardized ROS 2 messages.

Nodes:

| Node | Purpose | Hardware |
|------|---------|----------|
| `hri_face_detect` | Face detection (YuNet CNN) | CPU only |
| `hri_pose_detect` | Body pose estimation (YOLOv8-pose) | GPU required |
| `hri_emotion_detect` | Facial emotion recognition | CPU |
| `hri_id_manager` | Consistent person ID tracking across detectors | CPU |
| `hri_detection_display` | RViz2 visualization overlay | CPU |
| `hri_stt` | Speech-to-text (faster-whisper) | Nvidia GPU (CUBLAS/CUDNN) |
| `hri_tts` | Text-to-speech (Coqui TTS) | GPU recommended |

Topics (perception):

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/humans/faces` | `hri_msgs/Face2DList` | Detected faces with landmarks |
| `/humans/bodies` | `hri_msgs/Skeleton2DList` | Detected body poses with landmarks |
| `/humans/bodies/skel3D` | `hri_msgs/Skeleton3D` | 3D body poses (requires depth camera) |
| `/humans/faces/emotion` | `hri_msgs/Expression` | Recognized emotion per person |
| `/humans/voices/tracked` | `hri_msgs/IdsList` | Currently detected voices |
| `/humans/voices/<id>/speech` | `hri_msgs/LiveSpeech` | Live ASR output per voice |
| `/humans/persons/tracked` | `hri_msgs/IdsList` | Tracked persons |
| `/humans/faces/tracked` | `hri_msgs/IdsList` | Tracked faces |

These topics use **DDS Keys** (a Vulcanexus feature) for instance-based tracking — multiple people published on the same topic, distinguished by their key (person ID).

Message types from `hri_msgs`:
- `Face2D` — landmarks (eyes, nose, mouth corners) + unique ID
- `Face2DList` — list of Face2D + bounding boxes
- `Skeleton2D` — body skeleton landmarks + unique ID
- `Skeleton2DList` — list of Skeleton2D + bounding boxes + mean depth
- `Skeleton3D` — 3D keypoint coordinates
- `NormalizedRegionOfInterest2D` — bounding box
- `Expression` — emotion string + confidence + person ID
- `IdsList` — list of tracked IDs
- `LiveSpeech` — live transcription text

Body landmarks (17 keypoints): Nose, Neck, R/L Shoulder, R/L Elbow, R/L Wrist, R/L Hip, R/L Knee, R/L Ankle, L/R Eye, L/R Ear.

#### 2.6.2 Intent system

The intent system decouples **what someone wants done** from **how the robot does it**. Any input modality (speech, gesture, touchscreen, internal decision) produces a standardized `Intent` message that a mission controller can consume.

Topic: `/intents` — type: `hri_actions_msgs/msg/Intent`

Intent message fields:

| Field | Type | Description |
|-------|------|-------------|
| `intent` | `string` | Predefined constant (see table below) or custom string |
| `data` | `string` | JSON object with thematic roles: `agent`, `object`, `goal`, `recipient` |
| `source` | `string` | Who expressed the intent: a REP-155 person ID, or `ROBOT_ITSELF`, `REMOTE_SUPERVISOR`, `UNKNOWN` |
| `modality` | `string` | How: `MODALITY_SPEECH`, `MODALITY_MOTION`, `MODALITY_TOUCHSCREEN`, `MODALITY_INTERNAL`, `MODALITY_OTHER` |
| `priority` | `uint8` | 0-255, default 128 |
| `confidence` | `float32` | 0.0-1.0 |

Predefined intent constants:

| Constant | Value | Required Roles | Description |
|----------|-------|----------------|-------------|
| `ENGAGE_WITH` | `__intent_engage_with__` | `recipient` | Engage with another agent |
| `MOVE_TO` | `__intent_move_to__` | `goal` | Navigate to a location |
| `GUIDE` | `__intent_guide__` | `goal`, `recipient` | Guide someone somewhere |
| `GRAB_OBJECT` | `__intent_grab_object__` | `object` | Pick up an object |
| `BRING_OBJECT` | `__intent_bring_object__` | `object`, `recipient` | Bring object to someone |
| `PLACE_OBJECT` | `__intent_place_object__` | `recipient` | Place object on surface |
| `GREET` | `__intent_greet__` | `recipient` | Greet someone |
| `SAY` | `__intent_say__` | `object` (text) | Speak text aloud |
| `PRESENT_CONTENT` | `__intent_present_content__` | `object` (content ID) | Present predefined content |
| `PERFORM_MOTION` | `__intent_perform_motion__` | `object` (motion name) | Gesture or dance |
| `START_ACTIVITY` | `__intent_start_activity__` | `object` (activity name) | Start a scripted activity |
| `STOP_ACTIVITY` | `__intent_stop_activity__` | — | Cancel an activity |

Thematic roles (JSON keys in the `data` field):
- `agent` — who should perform the action (robot assumed if omitted)
- `object` / `theme` / `patient` — entity undergoing the effect
- `goal` — destination or target location
- `recipient` — who receives the object or action

Example:
```python
from hri_actions_msgs.msg import Intent
import json

msg = Intent()
msg.intent = Intent.MOVE_TO
msg.data = json.dumps({"goal": "zone_B"})
msg.source = "person_abc123"
msg.modality = Intent.MODALITY_SPEECH
msg.priority = 128
msg.confidence = 0.92
```

Pipeline:
```
Human (speech/gesture/touch)
  → Perception Layer (face/body/voice detection)
  → Dialogue Manager (parses speech → structured intent via chatbot/LLM)
  → publishes Intent on /intents
  → Mission Controller (subscribes, maps intents to skills)
  → Skill action servers execute the request
```

#### 2.6.3 Skills system

Skills are the execution side of the intent system. Each skill is a ROS 2 **action server** registered under `/skill/<skill_id>`.

Architecture:
- A mission controller subscribes to `/intents` and maps each intent to one or more skill calls.
- Skills report progress via action feedback and completion via action results.
- Common message types from `std_skills`:
  - `std_skills/msg/Meta` — caller name, priority (included in action goals)
  - `std_skills/msg/Result` — error codes (`ROS_ENOERR`, `ROS_EINTR`, `ROS_ECANCELED`, ...)
  - `std_skills/msg/Feedback` — generic progress feedback

Skill definition packages:
- `communication_skills` — `/skill/chat`, `/skill/ask`, `/skill/say`
- `navigation_skills` — `/skill/navigate_to_pose`
- `interaction_skills` — `/skill/greet`, `/skill/engage`
- `manipulation_skills` — `/skill/grab`, `/skill/place`

Custom skills can be defined by creating a ROS 2 action server under `/skill/<name>` and declaring it in `package.xml` via `<implements>` export tags.

Key packages:
- `hri_actions_msgs` — defines `Intent.msg`
- `std_skills` — defines `Meta.msg`, `Result.msg`, `Feedback.msg`
- `hri_msgs` — REP-155 perception messages

### 2.7 OPC UA integration

OPC UA (Open Platform Communications Unified Architecture) is a standard industrial communication protocol. Factory equipment (PLCs, sensors, actuators) exposes data via OPC UA servers.

In the ARISE middleware, an **IoT Agent OPC UA** bridges OPC UA servers to FIWARE. Industrial device data appears as NGSI-LD entities in the Context Broker alongside robot data.

Configuration modes:
- **Auto** — automatic discovery of OPC UA address space
- **Static** — manual mapping in `config.js`
- **Dynamic** — runtime configuration via REST API

Only relevant if integrating with industrial OPC UA devices.

### 2.8 Deployment via Docker Compose

Typical `docker-compose.yaml` includes:

```yaml
services:
  orion:
    image: fiware/orion-ld:1.10.0-PRE-1711
    command: -wip dds -mongocOnly
    network_mode: host
    volumes:
      - ./config-dds.json:/root/.orionld

  mongodb:
    image: mongo:7.0

  timescale:
    image: timescale/timescaledb-ha:pg17-ts2.18
    environment:
      POSTGRES_USER: postgres
      POSTGRES_PASSWORD: postgres

  mintaka:
    image: fiware/mintaka:0.6.18

  grafana:
    image: grafana/grafana:latest

  ros2:
    image: eprosima/vulcanexus:jazzy-desktop  # or custom image
    privileged: true
    ipc: host
    network_mode: host
```

Key Docker settings for ROS 2 / DDS:
- `privileged: true` — required for shared memory transport
- `ipc: host` — required for zero-copy / shared memory between containers
- `network_mode: host` — simplest option for DDS multicast discovery

### 2.9 Reference repositories

| Component | Repository |
|-----------|-----------|
| DDS Enabler | https://github.com/eProsima/FIWARE-DDS-Enabler |
| DDS Router | https://github.com/eProsima/DDS-Router |
| Orion-LD | https://github.com/FIWARE/context.Orion-LD |
| ARISE PoC | https://github.com/Engineering-Research-and-Development/arise-poc |
| Vulcanexus Docs | https://docs.vulcanexus.org/en/latest/ |
| Fast DDS | https://github.com/eProsima/Fast-DDS |

### 2.10 Documentation status note

As of early 2026, much of the official ARISE documentation at https://arise-framework-documentation.readthedocs.io is still work-in-progress. The "All-in-One Middleware", "FIWARE Components", "Getting Started", and "Tutorials" sections are stubs. Substantive content covers:
- Vulcanexus overview and HRI stack
- The ROS 2 / FIWARE basic app tutorial (turtlesim PoC)
- The PoC deployment guide

Refer to individual component repositories for deeper information.

---

## 3. RISES Mission / Task / Skill Mapping

### 3.1 Terminology

| ARISE Term | Definition | RISES Mapping |
|---|---|---|
| **Mission** | A high-level goal assigned to the robot (e.g. deliver a pallet). Composed of ordered tasks. | Defined by the fleet management system (SLAPStack) or a human supervisor. |
| **Task** | A discrete step within a mission. May involve one or more skill calls. | Decomposed by the `rises_mission_controller` node. |
| **Skill** | An atomic, reusable capability exposed as a ROS 2 action server under `/skill/<name>`. | Provided by `rises_skill_bridge` (wrapping geofence services). |
| **Intent** | A standardized expression of what someone wants done, published on `/intents`. | Consumed by `rises_mission_controller`, which maps intents to tasks and skills. |

### 3.2 Detailed system diagram

```
                            ┌─────────────────────────────────────────────────┐
                            │            ARISE Integration Layer              │
                            │              (optional overlay)                 │
                            │                                                 │
  /intents ──────────────► │  rises_mission_controller                        │
  (Intent.msg)             │    │                                             │
                           │    ├─ decomposes intent into tasks               │
                           │    ├─ calls /skill/* action servers              │
                           │    └─ reports mission status                     │
                           │                                                  │
                           │  rises_skill_bridge                              │
                           │    ├─ /skill/validate_path                       │
                           │    ├─ /skill/set_area_state                      │
                           │    ├─ /skill/update_map                          │
                           │    ├─ /skill/update_warehouse_layout             │
                           │    ├─ /skill/set_safety_radius                   │
                           │    ├─ /skill/get_area_state                      │
                           │    ├─ /skill/get_safety_radius                   │
                           │    └─ /skill/get_map_info                        │
                           │         │                                        │
                           └─────────┼────────────────────────────────────────┘
                                     │ service calls
                           ┌─────────┼────────────────────────────────────────┐
                           │         ▼     Core System (no ARISE deps)        │
                           │                                                  │
                           │  geofence_node (lifecycle)                       │
                           │    ├─ continuous scan-to-map matching            │
                           │    ├─ obstacle detection & reporting             │
                           │    ├─ area locking / unlocking                   │
                           │    └─ path validation                            │
                           │                                                  │
                           │  laserscan_preprocessor                          │
                           │    └─ raw LiDAR → segmented obstacles            │
                           │                                                  │
                           │  safety_node                                     │
                           │    ├─ monitors obstacle alerts                   │
                           │    ├─ validates paths before forwarding          │
                           │    └─ halts forklift on unsafe conditions        │
                           │                                                  │
                           │  message_translator                              │
                           │    └─ SLAPStack JSON ↔ ROS 2 messages            │
                           │                                                  │
                           │  fleet_interface                                 │
                           │    └─ VDA5050 order ↔ ROS 2 path                 │
                           │                                                  │
                           └──────────────────────────────────────────────────┘
                                     │ publishes topics
                           ┌─────────┼────────────────────────────────────────┐
                           │         ▼      FIWARE IT Layer (optional)        │
                           │                                                  │
                           │  DDS Enabler                                     │
                           │    └─ maps DDS topics → NGSI-LD entities         │
                           │                                                  │
                           │  Orion-LD Context Broker                         │
                           │    └─ stores current entity state                │
                           │                                                  │
                           │  TimescaleDB + Mintaka                           │
                           │    └─ historical queries                         │
                           │                                                  │
                           │  Grafana                                         │
                           │    └─ dashboards for obstacle alerts, status     │
                           │                                                  │
                           └──────────────────────────────────────────────────┘
```

### 3.3 Mission definitions

#### 3.3.1 Geofence skills as consumers (MOVE_TO example)

The geofence system does **not** handle navigation. Navigation intents like `MOVE_TO` are handled by an external navigation mission controller (nav2, fleet management). That controller calls our geofence skills as part of its own mission decomposition:

```
External Navigation Mission Controller receives MOVE_TO intent
│
├── Task 1: Validate path safety
│     Calls our skill: /skill/validate_path
│     If blocked → reject path, reroute
│
├── Task 2: Check area lock status
│     Calls our skill: /skill/get_area_state
│     If locked → wait or reroute
│
├── Task 3: Navigate to destination
│     Calls navigation skill (nav2, fleet interface — not ours)
│
├── Task 4: Monitor safety during transit (continuous)
│     Subscribes to our obstacle_alert topic
│     On alert → trigger halt
│
└── Task 5: Arrive / report status
```

Our skills are reusable building blocks that any ARISE-compliant mission controller can call. We provide the safety verification layer, not the navigation.

#### 3.3.2 Mission: Dynamic map update (handled by our controller)

An operator or external system needs to update the geofence map with new obstacle data or warehouse layout changes.

```
Mission: UPDATE_GEOFENCE_MAP
│
├── Task 1: Update obstacle map
│     Skill: /skill/update_map
│     Input: ObstacleUpdateArray (add/remove/modify obstacles)
│     Output: success (bool), processed count
│
├── Task 2: Update warehouse boundaries
│     Skill: /skill/update_warehouse_layout
│     Input: Contours (new warehouse boundaries)
│     Output: success (bool)
│
└── Task 3: Adjust safety parameters
      Skill: /skill/set_safety_radius
      Input: radius (float32)
      Output: success (bool)
```

#### 3.3.3 Mission: Area access control (handled by our controller)

Lock or unlock warehouse zones for coordinated multi-AGV staging area access.

```
Mission: AREA_ACCESS_CONTROL
│
├── Task 1: Query current area state
│     Skill: /skill/get_area_state
│     Input: area_id
│     Output: locked (bool), found (bool)
│
└── Task 2: Set area state
      Skill: /skill/set_area_state
      Input: area_id, locked (bool)
      Output: success (bool)
```

### 3.4 Intent mapping

Our `rises_mission_controller` handles only geofence-specific intents. Navigation intents (`MOVE_TO`) are ignored — they are handled by an external navigation controller that may call our skills.

Intents handled by our controller:

| Intent | Maps to Mission | Required Data Fields |
|---|---|---|
| `START_ACTIVITY("update_map")` | UPDATE_GEOFENCE_MAP | `object` (map update payload) |
| `START_ACTIVITY("lock_area")` | AREA_ACCESS_CONTROL | `object` (area_id) |
| `START_ACTIVITY("unlock_area")` | AREA_ACCESS_CONTROL | `object` (area_id) |
| `STOP_ACTIVITY` | Cancels active mission | — |

Intents NOT handled (consumed by external controllers that call our skills):

| Intent | External Controller | Our Skills Used |
|---|---|---|
| `MOVE_TO` | Navigation / fleet controller | `/skill/validate_path`, `/skill/get_area_state` |

Intent flow:
```
1. Intent arrives on /intents topic (hri_actions_msgs/Intent)
     Source: __myself__, __remote_supervisor__, or person_<id>
     Modality: __modality_internal__ (fleet mgmt), __modality_speech__, etc.

2a. Geofence intent (START_ACTIVITY with lock/unlock/update_map):
     → rises_mission_controller decomposes into tasks
     → Calls /skill/* action servers on rises_skill_bridge
     → Publishes progress on /mission_status

2b. Navigation intent (MOVE_TO):
     → Ignored by rises_mission_controller
     → External navigation controller handles it
     → That controller calls our /skill/validate_path, /skill/get_area_state
       as part of its own task decomposition
```

### 3.5 Skill reference

All skills exposed by `rises_skill_bridge` under the `/skill/` namespace.

| Skill | Action Type | Input | Output |
|---|---|---|---|
| `/skill/validate_path` | `rises_interfaces/action/ValidatePath` | `nav_msgs/Path` | `blocked`, `blocked_obstacles` |
| `/skill/set_area_state` | `rises_interfaces/action/SetAreaState` | `area_id`, `locked` | `success`, `message` |
| `/skill/update_map` | `rises_interfaces/action/UpdateMap` | `ObstacleUpdateArray` | `success`, `processed_count` |
| `/skill/update_warehouse_layout` | `rises_interfaces/action/UpdateWarehouseLayout` | `Contours` | `success` |
| `/skill/set_safety_radius` | `rises_interfaces/action/SetSafetyRadius` | `radius` (float32) | `success`, `previous_radius` |
| `/skill/get_area_state` | `rises_interfaces/action/GetAreaState` | `area_id` | `locked`, `found` |
| `/skill/get_safety_radius` | `rises_interfaces/action/GetSafetyRadius` | — | `radius` |
| `/skill/get_map_info` | `rises_interfaces/action/GetMapInfo` | — | `obstacle_count`, `contours_loaded` |

### 3.6 Reusable modules

Three modules are designed for standalone reuse, independent of SLAPStack:

#### 3.6.1 Geofence module (`geofence/`)

ROS 2 lifecycle node for real-time unknown obstacle detection via LiDAR scan-to-map matching. Can be integrated into any ROS 2 system that provides `sensor_msgs/LaserScan` or `sensor_msgs/PointCloud2` input and obstacle map data.

- Dependencies: ROS 2, `rises_interfaces` (message definitions only)
- No dependency on: SLAPStack, FIWARE, MQTT, VDA5050

#### 3.6.2 Safety module (`safety/`)

ROS 2 node that monitors geofence output (obstacle alerts, diagnostics) and enforces safety policies (halt on obstacle, validate paths before forwarding).

- Dependencies: ROS 2, `rises_interfaces`, `diagnostic_msgs`
- No dependency on: SLAPStack, FIWARE, MQTT, VDA5050

#### 3.6.3 HRI prediction module (`obstacle_heatmap_predictor/`)

ROS 2 node that predicts future obstacle (human) positions from tracked trajectories. Publishes a `nav_msgs/OccupancyGrid` heatmap showing predicted presence probability for the next 30-60 seconds. Extends ROS4HRI perception with predictive capabilities for path planning and predictive geofencing.

- Dependencies: ROS 2, `rises_interfaces` (message definitions), `nav_msgs`
- No dependency on: SLAPStack, FIWARE, MQTT, VDA5050, ROS4HRI runtime

---

## 4. FIWARE Integration

The system integrates with FIWARE through the DDS Enabler, which maps ROS 2 DDS topics directly to NGSI-LD entities in the Orion-LD Context Broker. This replaces the previous MQTT-based communication path.

See [fiware/](fiware/) for the docker-compose stack and DDS Enabler configuration.

### 4.1 Stack components

| Component | Port | Purpose |
|-----------|------|---------|
| MongoDB | 27017 | Orion-LD backend storage |
| TimescaleDB | 5432 | Time-series data (TROE) |
| Orion-LD | 1026 | NGSI-LD Context Broker with DDS integration |
| Mintaka | 8080 | REST API for historical/temporal entity queries |
| Grafana | 3000 | Dashboards (auto-provisioned with geofence overview) |
| Alert Receiver | 9090 | Webhook endpoint for Orion-LD subscription notifications |

### 4.2 Entity mapping

DDS Enabler config (`fiware/config/dds-enabler.json`) maps ROS 2 DDS topics directly to NGSI-LD entities — no MQTT involved:

| ROS 2 / DDS Topic | NGSI-LD Entity | Attribute |
|---|---|---|
| `/<ns>/obstacle_report` (`rt/<ns>/...`) | `urn:ngsi-ld:AGV:<ns>` | `obstacle_report` |
| `/<ns>/obstacle_alert` | `urn:ngsi-ld:AGV:<ns>` | `obstacle_alert` |
| `/<ns>/unmatched_obstacles` | `urn:ngsi-ld:AGV:<ns>` | `unmatched_obstacles` |
| `/<ns>/geofence_ready` | `urn:ngsi-ld:AGV:<ns>` | `geofence_ready` |
| `/mission_status` | `urn:ngsi-ld:MissionController:rises` | `mission_status` |

Mappings for `agv_0` and `agv_1` ship by default. Add more AGVs by duplicating the topic entries with the appropriate namespace.

### 4.3 Grafana dashboard

Auto-provisioned "RISES Geofence Overview" with panels for obstacle alert status, alert history timeline, geofence ready state, and mission status.

### 4.4 Automated alert generation (`fiware/alert_service/`)

Orion-LD subscriptions trigger webhook notifications to the alert receiver whenever:
- `obstacle_alert` changes on any AGV (obstacle detected / cleared)
- `geofence_ready` changes on any AGV (node initialized / shutdown)
- `mission_status` changes on the mission controller

The alert receiver logs events to stdout and to a JSON-lines file (`logs/alerts.jsonl`) for historical analysis. Can be extended to forward to Slack, email, or any external system.

Subscriptions are created by running `fiware/scripts/setup_subscriptions.sh` after the stack is healthy.

### 4.5 Usage

```bash
cd fiware/
cp .env.example .env   # adjust credentials if needed
docker compose up -d

# Wait for Orion-LD to be healthy, then set up subscriptions
sleep 15
./scripts/setup_subscriptions.sh

# Verify Orion-LD is running
curl http://localhost:1026/version

# Verify alert receiver is healthy
curl http://localhost:9090/

# View alert logs
docker logs -f rises-alert-receiver

# Query entity state
curl http://localhost:1026/ngsi-ld/v1/entities/urn:ngsi-ld:AGV:agv_0

# List active subscriptions
curl http://localhost:1026/ngsi-ld/v1/subscriptions | python3 -m json.tool

# Historical query via Mintaka
curl "http://localhost:8080/temporal/entities/urn:ngsi-ld:AGV:agv_0?timeproperty=modifiedAt&timerel=after&timeAt=2026-01-01T00:00:00Z"

# Grafana dashboard
xdg-open http://localhost:3000  # admin/admin
```

Key files:
- `fiware/docker-compose.yaml`
- `fiware/config/orionld.json`
- `fiware/config/dds-enabler.json`
- `fiware/.env.example`
- `fiware/scripts/setup_subscriptions.sh`
- `fiware/alert_service/alert_receiver.py`
- `fiware/grafana/provisioning/datasources/timescaledb.yaml`
- `fiware/grafana/provisioning/dashboards/dashboards.yaml`
- `fiware/grafana/dashboards/geofence_overview.json`

### 4.6 Industrial protocols summary

- **VDA5050** — used by `fleet_interface` for AGV order/state communication
- **NGSI-LD** — used by the DDS Enabler for IT system integration via Orion-LD
- **DDS / Fast DDS** — underlying transport via Vulcanexus

---

## 5. Integration changelog

What was added to align RISES with ARISE, addressing evaluator recommendations from stages 2.1 and 2.2.

### 5.1 Mission / task / skill architecture (`rises_mission_controller/`)

Why: evaluators flagged "missions, tasks, and skills are not formally defined" (2.2 rec #2) and "lacks alignment with the ARISE architecture" (2.2 general assessment).

What it does:
- Subscribes to `/intents` topic (`hri_actions_msgs/Intent` from ROS4HRI).
- Handles only geofence-specific intents (area locking, map updates).
- Navigation intents like `MOVE_TO` are not handled — an external navigation controller handles those and calls our `/skill/*` servers as part of its own task decomposition.
- Publishes mission progress on `/mission_status`.

Missions handled:

| Mission | Triggered by | Tasks |
|---------|-------------|-------|
| `AREA_ACCESS_CONTROL` | `START_ACTIVITY("lock_area"/"unlock_area")` | 1. get_area_state 2. set_area_state |
| `UPDATE_GEOFENCE_MAP` | `START_ACTIVITY("update_map")` | 1. get_map_info 2. get_safety_radius |

Files:
- `rises_mission_controller/include/rises_mission_controller/mission_controller_node.hpp`
- `rises_mission_controller/src/mission_controller_node.cpp`
- `rises_mission_controller/src/main.cpp`
- `rises_mission_controller/launch/mission_controller.launch.py`
- `rises_mission_controller/CMakeLists.txt`
- `rises_mission_controller/package.xml`

Launch:
```bash
ros2 launch rises_mission_controller mission_controller.launch.py namespace:=agv_0
```

Dependencies: `rclcpp`, `rclcpp_action`, `rises_interfaces` only.

### 5.2 MissionStatus message and hri_actions_msgs dependency

Intent message: the mission controller uses `hri_actions_msgs/msg/Intent` directly from the ROS4HRI package (`ros-jazzy-hri-actions-msgs`). No local copy — if you run without ARISE, you simply don't launch the mission controller.

New message in `rises_interfaces`:

`msg/MissionStatus.msg` — mission progress reporting:
- `mission_id`, `mission_type`
- `status` — `PENDING`, `ACTIVE`, `SUCCEEDED`, `FAILED`, `CANCELLED`
- `current_task`, `total_tasks`, `completed_tasks`
- `message`

### 5.3 Skill bridge query services (`rises_skill_bridge/`)

Why: the skill bridge only exposed write operations (set area state, update map). Evaluators need to see external systems can also query the geofence state.

New skills added:

| Skill | Purpose |
|-------|---------|
| `/skill/get_area_state` | Query whether a specific area is locked |
| `/skill/get_safety_radius` | Get the current safety circle radius |
| `/skill/get_map_info` | Get obstacle count, contour status, segment/polygon counts |

New service + action definitions in `rises_interfaces/`:
- `srv/geofencing/GetAreaState.srv`
- `srv/geofencing/GetSafetyRadius.srv`
- `srv/geofencing/GetMapInfo.srv`
- `action/geofencing/GetAreaState.action`
- `action/geofencing/GetSafetyRadius.action`
- `action/geofencing/GetMapInfo.action`

Geofence node changes: service handler declarations added to `geofencing_node.hpp` for the three query services. Plain ROS 2 services with no ARISE dependency.

### 5.4 FIWARE / NGSI-LD stack (`fiware/`)

Why: evaluators said "avoid the current default MQTT northbound path" (2.2 rec #1) and "leverage the embedded DDS Enabler in Orion-LD for NGSI-LD updates."

What it provides: see Section 4.

### 5.5 Architecture documentation

Why: evaluators requested "clear diagrams of middleware flows, HRI interactions, and ROS 2 topic structures, as well as explanations of mission/task/skill mappings" (2.2 rec #8).

This document (Section 2 + 3) covers all of the above.

### 5.6 Obstacle heatmap predictor (`obstacle_heatmap_predictor/`)

Why: evaluators recommended "develop a module that extends current ROS4HRI perception to predict potential human positions in the next 30-60 seconds (e.g., generating heatmaps of probable human presence)" (2.1 rec #1, 2.2 rec #4).

What it does:
- Subscribes to `obstacle_report` from the geofence node.
- Tracks unmatched obstacle trajectories using their persistent segment IDs (assigned by `ErrorSegmentTracker` in the geofence node).
- Estimates linear velocity via least-squares regression on the position history.
- Projects positions forward in time (configurable, default 30 seconds).
- Publishes a `nav_msgs/OccupancyGrid` on `predicted_occupancy` where each cell represents the probability (0-100) of an obstacle being present at that location.

Algorithm:
1. Each unmatched obstacle has a persistent ID from the geofence error segment tracker.
2. Observations are accumulated in a sliding window (default 10 seconds).
3. With 3+ observations, linear velocity is estimated via least-squares fit.
4. Velocity is clamped to 2.0 m/s (reasonable human walking speed).
5. Positions are projected forward at 2-second intervals up to the prediction horizon.
6. Each projected position is stamped as a Gaussian blob on the grid.
7. Confidence decays linearly with prediction time (nearer = more certain).
8. The grid is normalized to 0-100 and published as an OccupancyGrid.

Parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `observation_window_sec` | 10.0 | Sliding window for position history |
| `prediction_horizon_sec` | 30.0 | How far ahead to predict |
| `prediction_step_sec` | 2.0 | Time step between prediction samples |
| `eviction_timeout_sec` | 5.0 | Remove tracks not seen for this long |
| `min_observations` | 3 | Minimum observations before predicting |
| `grid_resolution` | 0.25 | Meters per grid cell |
| `grid_width` / `grid_height` | 30.0 | Grid dimensions in meters |
| `gaussian_sigma` | 1.0 | Spatial spread of prediction Gaussian |
| `publish_rate_hz` | 2.0 | Heatmap publish frequency |
| `frame_id` | "map" | TF frame for the occupancy grid |

Launch:
```bash
ros2 launch obstacle_heatmap_predictor heatmap_predictor.launch.py namespace:=agv_0
```

Dependencies: `rclcpp`, `rises_interfaces` (message definitions), `nav_msgs`. No ARISE/ROS4HRI/FIWARE dependency.

Visualization: the `predicted_occupancy` topic can be visualized in RViz2 as a Map display, showing a heatmap overlay of predicted obstacle presence on the warehouse floor.

---

## 6. Open work

### Must-do before next deliverable

| Item | Status | Notes |
|------|--------|-------|
| Finish geofence query service handlers (`.cpp`) | Not done | Headers are added, implementation pending |
| Finish skill bridge query action servers (`.cpp`) | Not done | Headers/includes are added, implementation pending |

### Should-do

| Item | Status | Notes |
|------|--------|-------|
| HRI prediction module (obstacle heatmaps) | Done | `obstacle_heatmap_predictor` package. Addresses 2.1 rec #1 and 2.2 rec #4. |
| End-to-end integration tests (ROS 2 → DDS → NGSI-LD) | Not started | `launch_testing` tests. Addresses 2.2 rec #7. |

### Nice-to-have

| Item | Notes |
|------|-------|
| Reduce SLAPStack coupling in `message_translator` | Make translator generic enough to work without SLAPStack-specific JSON |
| Add more AGV namespaces to DDS Enabler config | Currently agv_0 and agv_1, add as needed |
| Grafana alerting rules | Email/Slack alerts on `obstacle_alert == true` |

---

## 7. Launching the full ARISE stack

```bash
# 1. Start FIWARE infrastructure
cd fiware/ && docker compose up -d

# 2. Launch core geofence system (no ARISE deps needed)
ros2 launch rises_bringup geofence.launch.py namespace:=agv_0

# 3. Launch ARISE skill bridge (optional)
ros2 launch rises_skill_bridge skill_bridge.launch.py namespace:=agv_0

# 4. Launch ARISE mission controller (optional)
ros2 launch rises_mission_controller mission_controller.launch.py namespace:=agv_0

# 5. Run the DDS Enabler on the host (bridges DDS → Orion-LD)
# (install from https://github.com/eProsima/FIWARE-DDS-Enabler)
dds-enabler -c fiware/config/dds-enabler.json
```

Steps 3, 4, and 5 are optional. The core system (step 2) runs without any ARISE components.
