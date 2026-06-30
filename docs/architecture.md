# Architecture & interaction flows

Addresses the stage-3 recommendation to make the architecture and the
operator/system + AGV-coordination interaction flows clearer and visual.
Diagrams are Mermaid (renders on GitHub). The four interaction-flow views are
below; see **Module internals & system diagrams** further down for state
machines, module internals, fleet aggregation, the FIWARE data path, and deployment.

## Component architecture

```mermaid
flowchart LR
  subgraph PERCEPTION
    SCAN["2-D LiDAR<br/>sensor_msgs/LaserScan"]
    PRE["laserscan_preprocessor<br/>(segmentation)"]
  end
  subgraph GEOFENCE["Geofence module (this deliverable)"]
    GF["geofence_node OR geofence_gridmap_node<br/>(lifecycle)"]
    MAP[("warehouse map<br/>obstacles + boundary")]
    VAL["validation_node<br/>(KPIs, optional)"]
  end
  subgraph CONSUMERS
    FLEET["fleet / operator loop<br/>(slow / stop)"]
    LEG["rises_leg_filter<br/>(ROS4HRI, optional)"]
    BR["FIWARE bridge"]
  end
  subgraph MIDDLEWARE["ARISE middleware"]
    ORION["Orion-LD<br/>urn:ngsi-ld:AGV:&lt;id&gt;"]
    TSDB[("TimescaleDB<br/>TROE")]
    GRAF["Grafana / Mintaka"]
    KPI["export_kpis.py → logs/kpis_*.csv"]
  end

  SCAN --> PRE -->|"lidar_segments<br/>ObstacleArray"| GF
  MAP --> GF
  GF -->|"obstacle_alert (Bool)"| FLEET
  GF -->|"obstacle_report"| VAL
  GF -->|"obstacle_report / alert"| BR
  GF -->|"unmatched_obstacles"| LEG
  LEG -->|"/humans/bodies/*"| FLEET
  VAL -->|"validation_result"| BR
  BR -->|"NGSI-LD attrs (PATCH)"| ORION
  ORION --> TSDB --> GRAF
  TSDB --> KPI
```

Backends are mutually exclusive (`gridmap_enabled` selects spatial XOR gridmap).
The `validation_node`, `rises_leg_filter`, and the DDS-Enabler path are optional.

## Per-scan detection sequence

```mermaid
sequenceDiagram
  participant L as LiDAR
  participant P as Preprocessor
  participant G as Geofence
  participant M as Map (snapshot)
  participant B as FIWARE bridge
  participant F as Fleet / Operator

  L->>P: LaserScan
  P->>G: lidar_segments (ObstacleArray)
  G->>M: query: vertex within correspondence_tolerance of map?
  M-->>G: matched / unmatched per vertex
  G->>G: segment + classify (matched vs unmatched)
  alt unmatched obstacle in safety circle
    G->>F: obstacle_alert = true
    F->>F: slow / stop AGV
  else clear
    G->>F: obstacle_alert = false
  end
  G->>B: obstacle_report (matched[], unmatched[])
  B->>B: PATCH urn:ngsi-ld:AGV:<id> attrs → Orion-LD → TROE
```

## Operator / AGV-coordination interaction flow

```mermaid
sequenceDiagram
  participant OP as Operator / Fleet supervisor
  participant FM as Fleet manager
  participant G as Geofence (per AGV)
  participant A as AGV motion

  Note over G: continuously evaluates LiDAR vs map
  G-->>FM: obstacle_alert / obstacle_report (per AGV, via NGSI-LD)
  FM->>A: command (proceed / slow / stop) based on alert
  OP-->>FM: monitor AGV + obstacle state (Grafana / NGSI-LD)
  OP->>G: (service) set_safety_circle_radius / set_area_state (lock zone)
  G-->>OP: validate_path result (is a planned path blocked?)
```

## Intent → mission → task → skill decomposition

Shows how an operator *intent* cascades into a task and its constituent skill calls —
instantiated on the "coordinate AGVs in a shared corridor" example from §1.1 (Table 1),
using the geofence service surface.

```mermaid
sequenceDiagram
  participant OP as Operator (intent)
  participant MC as Mission controller
  participant T as Task — "cross shared corridor"
  participant SK as Skills (geofence services)
  participant G as Geofence module

  OP->>MC: intent START_ACTIVITY ("cross shared corridor")
  MC->>T: select & start task (finite skill sequence)
  T->>SK: get_area_state (corridor free?)
  SK->>G: query zone state
  G-->>SK: free / locked
  T->>SK: set_area_state(lock) — reserve corridor
  T->>SK: set_safety_circle_radius(tighten) — narrow passage
  T->>SK: validate_path (planned path blocked?)
  SK->>G: evaluate path vs map + live obstacles
  G-->>SK: clear / blocked
  T->>SK: execute crossing
  T->>SK: set_safety_circle_radius(restore)
  T->>SK: set_area_state(unlock)
  T-->>MC: task complete
  MC-->>OP: mission_status feedback (Grafana / NGSI-LD)
```

These four views (component, per-scan, operator/coordination, intent→skill decomposition)
are the interaction-flow visuals requested at stage 3; reuse them in the D4 written report.

---

# Module internals & system diagrams

The four diagrams above cover the stage-3 interaction flows. The views below document the rest of the system — node state machines, module internals, fleet aggregation, the FIWARE/NGSI-LD data path, and deployment. Each is generated from and cites the source files in its caption; all render on GitHub.

## State machines

### Geofence Node Lifecycle

Managed-node lifecycle state machine for geofence backends (spatial and gridmap), showing the four primary states (UNCONFIGURED, INACTIVE, ACTIVE, FINALIZED) and transitions via on_configure, on_activate, on_deactivate, on_cleanup. Includes auto-activation timer behavior: when enabled, a 100ms wall timer auto-advances UNCONFIGURED→INACTIVE→ACTIVE, then cancels itself. Source: lifecycle_geofence_node_base.hpp/cpp.

```mermaid
stateDiagram-v2
[*] --> UNCONFIGURED

UNCONFIGURED --> INACTIVE: on_configure() (success)
UNCONFIGURED --> UNCONFIGURED: on_configure() (failure)

INACTIVE --> ACTIVE: on_activate() (success)
INACTIVE --> INACTIVE: on_activate() (failure)
INACTIVE --> UNCONFIGURED: on_cleanup()

ACTIVE --> INACTIVE: on_deactivate()
ACTIVE --> FINALIZED: on_shutdown()

FINALIZED --> [*]

note right of UNCONFIGURED
  auto-activate enabled:
  Timer loops→
  trigger_transition(CONFIGURE)
end note

note right of INACTIVE
  auto-activate enabled:
  Timer loops→
  trigger_transition(ACTIVATE)
end note

note right of ACTIVE
  auto-activate enabled:
  Timer detects ACTIVE
  and cancels itself
end note
```

<sub>Source: `geofence/common/include/geofence/common/node/lifecycle_geofence_node_base.hpp`, `geofence/common/src/node/lifecycle_geofence_node_base.cpp`</sub>

### Mission Controller FSM (ARISE Mission Controller)

Finite state machine for the ARISE mission controller, derived from executeMission() in mission_controller_node.cpp and verified by test cases in test_mission_state_machine.cpp. Shows five operational states (Idle, Active, Succeeded, Failed, Cancelled) and transitions triggered by intents and task execution outcomes.

```mermaid
stateDiagram-v2
[*] --> Idle

Idle --> Active: START_ACTIVITY (lock_area/unlock_area/ update_map)
Idle --> Idle: START_ACTIVITY rejected (mission_active)

Active --> Succeeded: All tasks succeed (STATUS_SUCCEEDED)
Active --> Failed: Task fails (STATUS_FAILED)
Active --> Cancelled: STOP_ACTIVITY (mission_active=false)

Succeeded --> Idle: Mission complete
Failed --> Idle: Mission complete
Cancelled --> Idle: Mission complete

Idle --> [*]

note right of Active
  mission_active_ = true
  Publishes STATUS_ACTIVE
  Executes task sequence
end note

note right of Succeeded
  mission_active_ = false
  Publishes STATUS_SUCCEEDED
  All tasks completed
end note

note right of Failed
  mission_active_ = false
  Publishes STATUS_FAILED
  Task execution failed
end note

note right of Cancelled
  mission_active_ = false
  Publishes STATUS_CANCELLED
  Cancelled via STOP_ACTIVITY
end note
```

<sub>Source: `rises_mission_controller/src/mission_controller_node.cpp`, `rises_mission_controller/include/rises_mission_controller/mission_controller_node.hpp`, `rises_mission_controller/test/test_mission_state_machine.cpp`, `rises_interfaces/msg/MissionStatus.msg`</sub>

## Perception & geofence internals

### Multi-LiDAR Synchronization & Segmentation Pipeline

Left-to-right processing pipeline in LaserPreprocessorNode: N synchronized LaserScan inputs flow through TF transformation and PointCloud2 conversion, segmentation strategy selection (DBSCAN/RegionGrow/Distance), spatial indexing with KD-tree, and shape fitting (Point/Line/Circle/Polygon) to produce ObstacleArray. Based on laserscan_preprocessor_node.hpp, laser_synchronizer.hpp, point_cloud_processor.hpp, segmentation_strategy.hpp, spatial_indexer.hpp, and shape_fitter.hpp.

```mermaid
flowchart LR
    A["N LaserScan<br/>Inputs"] -->|topics 0..N-1| B["LaserSynchronizer<br/>ApproximateTime"]
    B -->|onLaserScan| C["PointCloudProcessor<br/>convertToPointCloud2"]
    C -->|TF Transform| D["TF Buffer<br/>target_frame"]
    D -->|cloud_in| E["Point Cloud<br/>Segmentation"]
    E -->|Strategy Select| F["SegmentationStrategy<br/>dbscan/regionGrow/<br/>distanceSegment"]
    F -->|segment_id field| G["SpatialIndexer<br/>KD-tree"]
    G -->|findNeighbors| H["ShapeFitter<br/>Factory"]
    H -->|fitShape| I["Point/Line/Circle<br/>Polygon Fitter"]
    I -->|Obstacle msg| J["ObstacleArray<br/>Publisher"]
    
    style A fill:#e1f5ff
    style B fill:#fff3e0
    style C fill:#fff3e0
    style D fill:#f3e5f5
    style E fill:#f1f8e9
    style F fill:#f1f8e9
    style G fill:#fce4ec
    style H fill:#fff9c4
    style I fill:#fff9c4
    style J fill:#c8e6c9
```

<sub>Source: `laserscan_preprocessor/include/laserscan_preprocessor/laserscan_preprocessor_node.hpp`, `laserscan_preprocessor/include/laserscan_preprocessor/sync/laser_synchronizer.hpp`, `laserscan_preprocessor/include/laserscan_preprocessor/processing/point_cloud_processor.hpp`, `laserscan_preprocessor/include/laserscan_preprocessor/segmentation/segmentation_strategy.hpp`, `laserscan_preprocessor/include/laserscan_preprocessor/spatial/spatial_indexer.hpp`, `laserscan_preprocessor/include/laserscan_preprocessor/shapes/shape_fitter.hpp`</sub>

### Geofence Backend Abstraction: Spatial vs Gridmap

Class diagram showing the common lifecycle base (LifecycleGeofenceNodeBase) with virtual hooks (matchScan, loadObstaclesFromJson, applyMapUpdates) and two concrete backends: spatial (GeofenceNode + GeofenceMap) and gridmap (GeofenceGridmapNode + GridMap). Ground truth from geofence/common/include/geofence/common/node/lifecycle_geofence_node_base.hpp, geofence/spatial_node/include/geofence/spatial/node/geofencing_node.hpp, geofence/gridmap_node/include/geofence/gridmap/node/geofencing_gridmap_node.hpp, geofence/spatial_node/include/geofence/spatial/map/geofence_map.hpp, geofence/gridmap_node/include/geofence/gridmap/map/gridmap.hpp.

```mermaid
classDiagram
    class LifecycleGeofenceNodeBase {
        -map_frame_id_ : string
        -robot_frame_id_ : string
        -tf_buffer_ : TF2Buffer
        -robot_tracking_checker_ : RobotTrackingChecker
        -visualizer_ : GeofenceVisualizer
        -report_builder_ : ObstacleReportBuilder
        +on_configure() State
        +on_activate() State
        +on_deactivate() State
        +on_cleanup() State
        #matchScan(msg, pos, have_pos) ScanMatchResult
        #loadObstaclesFromJson(path) bool
        #loadContoursFromJson(path) bool
        #applyMapUpdates(updates, source) void
        #validatePathService(req, resp) void
        #updateMapService(req, resp) void
        #commonConfig() GeofenceCommonConfig
    }
    
    class GeofenceNode {
        -cfg_ : GeofenceConfig
        -geofence_map_ : GeofenceMap
        -safety_profile_ : RobotSafetyProfile
        -diagnostic_updater_ : DiagnosticUpdater
        +matchScan(msg, pos, have_pos) ScanMatchResult
        +loadObstaclesFromJson(path) bool
        +applyMapUpdates(updates, source) void
        +commonConfig() GeofenceConfig
        -getAreaStateService(req, resp) void
        -getSafetyRadiusService(req, resp) void
        -getMapInfoService(req, resp) void
        -getWarehouseContoursService(req, resp) void
    }
    
    class GeofenceGridmapNode {
        -cfg_ : GridmapConfig
        -gridmap_ : GridMap
        -diagnostic_updater_ : DiagnosticUpdater
        +matchScan(msg, pos, have_pos) ScanMatchResult
        +loadObstaclesFromJson(path) bool
        +applyMapUpdates(updates, source) void
        +commonConfig() GridmapConfig
        -checkCorrespondence(obs, pos) bool
        -buildMatchResult(msg, pos) ObstacleMatchResult
    }
    
    class GeofenceMap {
        -snapshot_ : RCU~Snapshot~
        -dynamic_quadtree_ : SimpleQuadTree
        +segmentIntersectsObstacle(start, end) bool
        +distanceToNearestObstacle(pt) float
        +queryNearby(pt, radius) vector~int64~
        +getObstacle(id) Geometry*
        +getArea(id) Rectangle*
        +isAreaLocked(id) bool
        +forEachObstacle(visitor) void
        +insertObstacle(id, geom) void
        +removeObstacle(id) void
        +insertDynamicObstacle(id, geom, ttl) bool
    }
    
    class GridMap {
        -snapshot_ : RCU~GridMapData~
        -config_ : Config
        +isOccupied(x, y) bool
        +isOccupied(x, y, mask) bool
        +findObstaclesNear(x, y, r) vector~int64~
        +findObstaclesInSafetyCircle(cx, cy, r) vector~int64~
        +isPathBlocked(x1, y1, x2, y2) bool
        +getGridWidth() size_t
        +getGridHeight() size_t
        +insertObstacle(id, obs, inflation) void
        +removeObstacle(id) void
        +updateSnapshot(updater) void
    }
    
    class RobotSafetyProfile {
    }
    
    class GeofenceVisualizer {
    }
    
    LifecycleGeofenceNodeBase <|-- GeofenceNode
    LifecycleGeofenceNodeBase <|-- GeofenceGridmapNode
    GeofenceNode *-- GeofenceMap
    GeofenceNode *-- RobotSafetyProfile
    GeofenceNode *-- GeofenceVisualizer
    GeofenceGridmapNode *-- GridMap
    GeofenceGridmapNode *-- GeofenceVisualizer
```

<sub>Source: `geofence/common/include/geofence/common/node/lifecycle_geofence_node_base.hpp`, `geofence/spatial_node/include/geofence/spatial/node/geofencing_node.hpp`, `geofence/gridmap_node/include/geofence/gridmap/node/geofencing_gridmap_node.hpp`, `geofence/spatial_node/include/geofence/spatial/map/geofence_map.hpp`, `geofence/gridmap_node/include/geofence/gridmap/map/gridmap.hpp`</sub>

## Fleet aggregation & prediction

### Obstacle Heatmap Predictor: Deterministic Pipeline

Data flow from unmatched obstacle reports through track persistence, velocity estimation (least-squares), forward projection (deterministic, non-ML), and Gaussian occupancy grid stamping. Parameters grounded in code: observation_window_sec (default 10.0s), prediction_horizon_sec (30.0s), prediction_step_sec (2.0s), gaussian_sigma (1.0m), grid resolution (0.25m/cell). Source: obstacle_heatmap_predictor/include/obstacle_heatmap_predictor/heatmap_predictor_node.hpp and src/heatmap_predictor_node.cpp.

```mermaid
flowchart LR
    A["obstacle_report<br/>(unmatched_obstacles)"] --> B["obstacleReportCallback"]
    B --> C["updateTrack<br/>per segment ID"]
    C --> D["history buffer<br/>(deque&lt;Observation&gt;)"]
    D --> E["Sliding window trim<br/>(observation_window_sec<br/>default: 10.0s)"]
    E --> F["estimateVelocity<br/>Least-Squares Fit"]
    F --> G["velocity_x, velocity_y<br/>(clamped to 2.0 m/s)"]
    H["evictStaleTracks<br/>(eviction_timeout_sec<br/>default: 5.0s)"] -.-> C
    G --> I["publishHeatmap<br/>(periodic 2 Hz)"]
    I --> J["Take track snapshot<br/>(latest_pos + velocity)"]
    J --> K["For each snapshot:<br/>stamp current position<br/>weight=1.0"]
    K --> L["Project forward<br/>in steps"]
    L --> M["t = 0 to prediction_horizon<br/>(default: 30.0s)<br/>step: prediction_step_sec<br/>(default: 2.0s)"]
    M --> N["Compute predicted_x<br/>predicted_y<br/>via linear extrapolation"]
    N --> O["Confidence decay<br/>1.0 - dt / horizon"]
    O --> P["stampGaussian<br/>on probability_grid"]
    P --> Q["Gaussian kernel<br/>sigma_cells = gaussian_sigma<br/>/ grid_resolution<br/>(default: 1.0m / 0.25m)"]
    Q --> R["3*sigma radius<br/>Gaussian PDF exp"]
    R --> S["Normalize &amp; quantize<br/>to OccupancyGrid<br/>0-100 occupancy"]
    S --> T["publish predicted_occupancy<br/>(nav_msgs/OccupancyGrid)"]
    
    style A fill:#e1f5ff
    style T fill:#c8e6c9
    style F fill:#fff9c4
    style P fill:#ffe0b2
```

<sub>Source: `obstacle_heatmap_predictor/include/obstacle_heatmap_predictor/heatmap_predictor_node.hpp`, `obstacle_heatmap_predictor/src/heatmap_predictor_node.cpp`</sub>

## Safety & validation

### Safety Node: Diagnostics-Driven Health Monitor & Halt Loop

Closed-loop diagnostics monitoring and path validation in the safety node. Subscribes to /diagnostics (DiagnosticArray) from monitored nodes, tracks per-node health level (OK/WARN/ERROR) and freshness, runs periodic health-check timer, and publishes halt commands on critical failures. Also validates incoming paths against heatmap predictions and geofence service. Source: safety/include/safety/safety.hpp, safety/src/safety.cpp

```mermaid
flowchart TB
    subgraph Input["Input Streams"]
        DiagMsg["/diagnostics<br/>(DiagnosticArray)"]
        DetObst["detected_obstacles<br/>(ObstacleArray)"]
        PathMsg["incoming_path<br/>(Path)"]
        HeatMap["predicted_occupancy<br/>(OccupancyGrid)"]
    end
    
    subgraph SafetyNode["Safety Node (safety_node)"]
        DiagCallback["diagnosticsCallback()<br/>Update NodeHealth map<br/>level + last_seen"]
        HealthTimer["healthCheckTimer<br/>(5 sec periodic)"]
        HealthCheck["healthCheckCallback()<br/>Check age > timeout<br/>vs node_timeout_sec"]
        DetCallback["detectedObstaclesCallback()<br/>Publish alert"]
        HeatCallback["heatmapCallback()<br/>Atomic store heatmap"]
        PathCallback["pathCallback()<br/>Check system halted?"]
        PathHeatmap["isPathOverlappingPrediction()<br/>Scan waypoints<br/>vs heatmap cells >= threshold"]
        ValidateService["ValidatePath service call<br/>(geofence validation)"]
        HaltFunc["haltSystem()<br/>Set system_halted = true"]
        ResumeFunc["resumeSystem()<br/>Set system_halted = false"]
    end
    
    subgraph State["Shared State"]
        Monitor["monitored_nodes_<br/>(NodeHealth map)<br/>level: 0=OK, 1=WARN, 2=ERROR"]
        Halted["system_halted_<br/>(atomic bool)"]
        LatestHeat["latest_heatmap_<br/>(shared_ptr, atomic)"]
    end
    
    subgraph Output["Output Streams"]
        AlertPub["alert<br/>(ObstacleArray)"]
        HaltPub["/halt<br/>(Bool: true=halt)"]
        ValidPathPub["validated_path<br/>(Path)"]
        RespPub["response<br/>(String)"]
    end
    
    DiagMsg --> DiagCallback
    DiagCallback --> Monitor
    Monitor --> |age exceeds timeout?| HealthCheck
    HealthTimer --> HealthCheck
    HealthCheck --> |all healthy and<br/>halted| ResumeFunc
    HealthCheck --> |node timeout| HaltFunc
    DiagCallback --> |level = ERROR| HaltFunc
    
    HeatMap --> HeatCallback
    HeatCallback --> LatestHeat
    
    DetObst --> DetCallback
    DetCallback --> AlertPub
    DetCallback --> RespPub
    
    PathMsg --> PathCallback
    PathCallback --> |system_halted<br/>= true| HaltFunc
    LatestHeat --> |atomic_load<br/>snapshot| PathHeatmap
    PathMsg --> PathHeatmap
    PathHeatmap --> |overlap detected| RespPub
    PathCallback --> ValidateService
    ValidateService --> |blocked| RespPub
    ValidateService --> |allowed| ValidPathPub
    ValidateService --> |unavailable| HaltFunc
    
    HaltFunc --> Halted
    HaltFunc --> HaltPub
    HaltFunc --> RespPub
    ResumeFunc --> Halted
    ResumeFunc --> HaltPub
    ResumeFunc --> RespPub
```

<sub>Source: `safety/include/safety/safety.hpp`, `safety/src/safety.cpp`</sub>

### Validation Node KPI Measurement Pipeline

Flowchart showing the validation_node's KPI measurement pipeline: JSON ground-truth spawn events are matched against obstacle_report detections using spatial tolerance (2.5m) and time window (3s), robot safety-circle eligibility is tracked via TF, and KPIs (detection latency, ratio, static-structure match rate) are computed and output as JSON validation_result and CSV log. Source files: geofence/validation_node/include/geofence/validation/validation_node.hpp and geofence/validation_node/src/validation_node.cpp

```mermaid
flowchart LR
    subgraph input["Input Sources"]
        spawn["obstacle_spawn<br/>(JSON ground-truth)"]
        report["obstacle_report<br/>(ObstacleReport)"]
        tf["TF Buffer<br/>(robot pose)"]
    end
    
    subgraph register["Obstacle Registration"]
        spawnCB["spawnCallback<br/>Parse JSON<br/>Create SpawnedObstacle"]
        regSpawn["registerSpawnedObstacle<br/>Store w/ metadata<br/>(x, y, r, t_spawn)"]
    end
    
    subgraph match["Detection Matching"]
        reportCB["reportCallback<br/>Extract unmatched_obstacles<br/>per segment"]
        tryMatch["tryMatchDetection<br/>Spatial match: dist &lt; 2.5m<br/>Time gate: 0 &lt; latency &lt; 3s"]
        pending["Buffer as pending<br/>if no spawn found"]
        retroMatch["retroactive match<br/>when spawn arrives"]
    end
    
    subgraph track["Safety Circle Tracking"]
        tfSample["sampleRobotPose<br/>Sample robot pose<br/>via TF every 100ms"]
        circleEntry["recordCircleEntry<br/>Robot dist &lt; eligibility_radius<br/>(3m)"]
        eligible["markEligible<br/>Increment eligible counter"]
    end
    
    subgraph latency["Latency Computation"]
        reportM["reportMatch<br/>Record match event<br/>t_detection, dist_m"]
        finalize["finalizeLatencyIfReady<br/>t_latency = t_detection - t_circle_entry<br/>(signed: before/after entry)"]
        lats["Accumulate:<br/>min, max, sum<br/>latency_samples++"]
    end
    
    subgraph classify["Static Structure Classification"]
        classify_u["classifyAndTrackUnmatched<br/>Read-only classification"]
        intruder["near_spawn?<br/>Legitimate intruder<br/>-&gt; unmatched_intruder_segments"]
        unclaimed["Unclaimed segment<br/>(static miss or noise)"]
        persist["Track persistent IDs<br/>if seen &gt;= persistent_min_reports<br/>(120 scans)<br/>-&gt; static_miss_segments"]
    end
    
    subgraph agg["KPI Aggregation"]
        stats["Compute stats:<br/>Detection Ratio: detected/spawned<br/>Avg Latency: sum/samples<br/>Static Match Rate"]
    end
    
    subgraph output["Output & Logging"]
        json_out["publishResult<br/>JSON validation_result<br/>(std_msgs/String)"]
        csv_out["CSV per-detection log<br/>(if output_file set)"]
    end
    
    spawn --> spawnCB
    spawnCB --> regSpawn
    regSpawn --> retroMatch
    
    report --> reportCB
    reportCB --> tryMatch
    tryMatch -->|match found| reportM
    tryMatch -->|no match| pending
    pending --> retroMatch
    retroMatch --> reportM
    
    tf --> tfSample
    tfSample --> circleEntry
    circleEntry --> eligible
    
    reportM --> finalize
    finalize --> lats
    
    reportCB --> classify_u
    classify_u -->|dist &lt; 2.5m| intruder
    classify_u -->|dist &gt;= 2.5m| unclaimed
    unclaimed --> persist
    
    intruder --> stats
    persist --> stats
    lats --> stats
    
    stats --> json_out
    stats --> csv_out
    
    json_out --> output
    csv_out --> output
```

<sub>Source: `geofence/validation_node/include/geofence/validation/validation_node.hpp`, `geofence/validation_node/src/validation_node.cpp`, `rises_interfaces/msg/geofencing/ObstacleReport.msg`</sub>

## Middleware & deployment

### FIWARE Data Path: ROS 2 → DDS Enabler → Orion-LD → TimescaleDB → Grafana

End-to-end telemetry pipeline showing ROS 2 core nodes publishing domain-specific messages, the fiware_bridge_node serializing to JSON strings on fiware/* topics, the DDS Enabler bridging to Orion-LD NGSI-LD entities, dual storage (MongoDB for state, TimescaleDB for temporal data), and visualization layers. The architecture uses automatic DDS type discovery (Fast-DDS 3.x) and ignores complex message types in favor of std_msgs/String JSON payloads. Grounded in actual topic names, entity IDs, and service calls from fiware_bridge_node.cpp, dds-enabler.json, docker-compose.yaml, and Dockerfile sources.

```mermaid
flowchart LR
    subgraph ROS2["ROS 2 Core Nodes<br/>(Warehouse AGV Domain)"]
        ObstacleReport["obstacle_report<br/>(ObstacleReport)"]
        ObstacleAlert["obstacle_alert<br/>(Bool)"]
        GeofenceReady["geofence_ready<br/>(Bool)"]
        WarehouseContours["warehouse_contours<br/>(Contours)"]
        Odometry["odometry/filtered<br/>(Odometry)"]
        Diagnostics["/diagnostics<br/>(DiagnosticArray)"]
        PredictedOccupancy["predicted_occupancy<br/>(OccupancyGrid)"]
    end

    Bridge["fiware_bridge_node<br/>(C++ ROS 2 node)<br/>JSON serialization"]

    subgraph FIWARE["FIWARE Topics (DDS)"]
        ObstacleSummary["fiware/obstacle_summary<br/>(String JSON)"]
        AlertJSON["fiware/obstacle_alert<br/>(String JSON)"]
        ReadyJSON["fiware/geofence_ready<br/>(String JSON)"]
        GeometryJSON["fiware/warehouse_geometry<br/>(String JSON)"]
        PositionJSON["fiware/robot_position<br/>(String JSON)"]
        HealthJSON["fiware/system_health<br/>(String JSON)"]
        HeatmapJSON["fiware/heatmap_summary<br/>(String JSON)"]
        MapObstacles["fiware/map_obstacles<br/>(String JSON)"]
        ObstacleSegments["fiware/obstacle_segments<br/>(String JSON)"]
        ValidationResult["fiware/validation_result<br/>(String JSON)"]
    end

    DDSEnabler["DDS Enabler<br/>(eProsima Vulcanexus)<br/>Fast-DDS 3.x type discovery<br/>rt/agv_0/fiware/* → NGSI-LD"]

    subgraph Orion["Orion-LD Context Broker<br/>(NGSI-LD Entity: agv_0)"]
        EntityAttrs["urn:ngsi-ld:AGV:agv_0<br/>attributes:<br/>obstacle_summary<br/>obstacle_alert<br/>geofence_ready<br/>warehouse_geometry<br/>robot_position<br/>system_health<br/>heatmap_summary<br/>map_obstacles<br/>obstacle_segments<br/>validation_result"]
    end

    Mongo["MongoDB<br/>(Current State)<br/>Port 27017"]
    TimeScale["TimescaleDB<br/>(TROE Temporal)<br/>Port 5432"]

    subgraph Viz["Visualization & Queries"]
        Grafana["Grafana<br/>Port 3000<br/>Dashboards"]
        Mintaka["Mintaka<br/>Port 8080<br/>Temporal REST API"]
    end

    ROS2 -->|subscribe and transform| Bridge
    Bridge -->|publish JSON| FIWARE
    FIWARE -->|DDS topic discovery<br/>and NGSI-LD PATCH| DDSEnabler
    DDSEnabler -->|REST PATCH<br/>to entity| Orion
    Orion -->|UPSERT state| Mongo
    Orion -->|TROE append| TimeScale
    Grafana -->|REST query<br/>entity attributes| Orion
    Grafana -->|SQL query<br/>historical data| TimeScale
    Mintaka -->|SQL query<br/>temporal RDF| TimeScale
```

<sub>Source: `fiware_bridge/src/fiware_bridge_node.cpp`, `fiware/docker-compose.yaml`, `fiware/dds-enabler/Dockerfile`, `fiware/config/dds-enabler.json`</sub>

### NGSI-LD AGV Entity Model

NGSI-LD AGV entity (urn:ngsi-ld:AGV:agv_0) and its attributes as mapped from ROS 2 messages by fiware_bridge. Attributes include obstacle summary, position, health, geofence status, geometry, heatmap, and map obstacles, extracted from JSON-formatted fiware/* topics.

```mermaid
erDiagram
AGV {
    string id "urn:ngsi-ld:AGV:agv_0"
    int matched_count
    int unmatched_count
    json unmatched_ids
    json matched_segments
    json unmatched_segments
}

OBSTACLE_ALERT {
    boolean active
}

GEOFENCE_STATUS {
    boolean ready
}

POSITION {
    double x
    double y
}

SYSTEM_HEALTH {
    string node_name
    int level
    string message
    string hardware_id
    json custom_diagnostics
}

WAREHOUSE_GEOMETRY {
    json wall_segments
    json outer_hull
    json inner_polygons
}

HEATMAP_SUMMARY {
    int nonzero_cells
    int max_value
    int grid_width
    int grid_height
    double resolution
    double origin_x
    double origin_y
    json hot_cells
}

MAP_OBSTACLES {
    int count
    json rectangles
}

VALIDATION_RESULT {
    json validation_data
}

AGV ||--o| OBSTACLE_ALERT : has
AGV ||--o| GEOFENCE_STATUS : has
AGV ||--o| POSITION : has
AGV ||--o| SYSTEM_HEALTH : has
AGV ||--o| WAREHOUSE_GEOMETRY : has
AGV ||--o| HEATMAP_SUMMARY : has
AGV ||--o| MAP_OBSTACLES : has
AGV ||--o| VALIDATION_RESULT : has
```

<sub>Source: `fiware_bridge/include/fiware_bridge/fiware_bridge_node.hpp`, `fiware_bridge/src/fiware_bridge_node.cpp`</sub>

### ARISE/RISES Deployment Topology

Multi-container deployment topology showing scenario selection, AGV_DEPLOY_MODE routing, and integration between ROS 2 geofence stack, ARISE middleware, and FIWARE monitoring stack. Grounded in rises.dockerfile (skill bridge, mission controller, heatmap, fiware_bridge), central.dockerfile (translator), docker-compose.yaml (Orion-LD, MongoDB, TimescaleDB, Grafana, DDS Enabler), and entrypoint.sh routing logic.

```mermaid
graph TB
    START["🎯 Entrypoint<br/>entrypoint.sh"]
    SCENARIO{"SCENARIO<br/>Param File?"}
    
    START --> SCENARIO
    SCENARIO -->|default| PARAMS_D["params_default.yaml<br/>(real robot)"]
    SCENARIO -->|rosbag| PARAMS_R["params_rosbag.yaml<br/>(use_sim_time on)"]
    SCENARIO -->|unity| PARAMS_U["params_unity.yaml<br/>(coordinate xforms)"]
    
    PARAMS_D --> DEPLOY_MODE
    PARAMS_R --> DEPLOY_MODE
    PARAMS_U --> DEPLOY_MODE
    
    DEPLOY_MODE{"AGV_DEPLOY_MODE<br/>Routing?"}
    DEPLOY_MODE -->|multi| MULTI["multi_agv_geofence<br/>.launch.py<br/>AGV_COUNT AGVs<br/>in ONE container"]
    DEPLOY_MODE -->|per_agv| PER_AGV["geofence.launch.py<br/>NAMESPACE=agv_N<br/>one AGV per<br/>container"]
    DEPLOY_MODE -->|ARISE_MODE=true| ARISE_L["rises_geofence<br/>.launch.py<br/>+ skill_bridge<br/>+ mission_controller<br/>+ heatmap<br/>+ fiware_bridge"]
    
    subgraph ARISE_CONTAINER ["ARISE Container<br/>(rises:rises)"]
        ARISE_NODES["Core Geofence Nodes:<br/>geofence_node<br/>laserscan_preprocessor<br/>fleet_interface<br/>message_translator<br/>-----<br/>ARISE Nodes:<br/>rises_skill_bridge<br/>rises_mission_controller<br/>obstacle_heatmap_predictor<br/>fiware_bridge"]
    end
    
    subgraph CENTRAL_CONTAINER ["Central Container<br/>(rises:central)"]
        CENTRAL_NODES["centralized_translator<br/>(message_translator_node)<br/>rosbag_player<br/>(optional)"]
        CENTRAL_PARAMS["central_params.yaml<br/>(env vars:<br/>AGV_COUNT,<br/>TRANSLATOR_*,<br/>USE_SIM_TIME)"]
        CENTRAL_PARAMS -.->|env-var substitution| CENTRAL_NODES
    end
    
    subgraph UNITY_CONTAINER ["Unity Container<br/>(rises:unity)"]
        UNITY_NODES["ROS-TCP-Endpoint<br/>(port 10000)<br/>centralized_translator<br/>rosbag_player<br/>(optional)"]
        TCP_IP["ROS_TCP_IP:ROS_TCP_PORT<br/>(0.0.0.0:10000)"]
        TCP_IP -.->|bridge to| UNITY_NODES
    end
    
    subgraph FIWARE_STACK ["FIWARE Stack<br/>(docker-compose.yaml)"]
        ORION["Orion-LD<br/>Context Broker<br/>:1026<br/>DDS Integrated"]
        MONGO["MongoDB<br/>Orion backend<br/>:27017"]
        TIMESCALE["TimescaleDB<br/>TROE temporal<br/>:5432"]
        DDS_EN["DDS Enabler<br/>(ddsenabler)<br/>DDS↔NGSI-LD<br/>bridge"]
        MINTAKA["Mintaka<br/>Temporal query API<br/>:8080"]
        GRAFANA["Grafana<br/>Dashboards<br/>:3000"]
        
        ORION -->|queries/updates| MONGO
        ORION -->|TROE write| TIMESCALE
        DDS_EN -->|NGSI-LD entities| ORION
        TIMESCALE -->|timeseries| MINTAKA
        TIMESCALE -->|datasource| GRAFANA
    end
    
    MULTI -.->|launch| ARISE_CONTAINER
    PER_AGV -.->|launch| ARISE_CONTAINER
    ARISE_L -.->|launch| ARISE_CONTAINER
    
    CENTRAL_NODES -->|subscribes to<br/>geofence topics| ARISE_NODES
    ARISE_NODES -->|ROS 2 DDS| DDS_EN
    DDS_EN -->|maps, obstacles,<br/>diagnostics| FIWARE_STACK
    GRAFANA -->|monitor| ORION
    
    UNITY_NODES -->|receive from<br/>ROS network| ARISE_NODES
    
    style START fill:#4CAF50,color:#fff
    style SCENARIO fill:#2196F3,color:#fff
    style DEPLOY_MODE fill:#2196F3,color:#fff
    style ARISE_CONTAINER fill:#FF9800,color:#fff
    style CENTRAL_CONTAINER fill:#9C27B0,color:#fff
    style UNITY_CONTAINER fill:#9C27B0,color:#fff
    style FIWARE_STACK fill:#F44336,color:#fff
```

<sub>Source: `entrypoint.sh`, `rises.dockerfile`, `central.dockerfile`, `central_entrypoint.sh`, `unity_entrypoint.sh`, `fiware/docker-compose.yaml`</sub>
