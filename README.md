# ARISE · RISES — Geofence Spatial-Safety Module (D4)

> **D4 rendered site (GitHub Pages):** **https://slapstack.github.io/rises/** — overview,
> [documentation portal](https://slapstack.github.io/rises/docs.html), and the
> [validation KPI dashboard](https://slapstack.github.io/rises/kpi-dashboard.html).
>
> **Note:** the `docs/*.html` files show as **raw source** in GitHub's file browser
> (GitHub does not render HTML in-repo) — open the live site above to view them as pages.
> _(Live once a maintainer enables GitHub Pages: **repo Settings → Pages → Deploy from a
> branch → `main` / `/docs`**.)_

A reusable HRI module: a **ROS 2 / Vulcanexus (Jazzy)** spatial-safety geofence for
autonomous warehouse AGVs. It matches 2-D LiDAR detections against a known warehouse
map and flags everything **not** in the map as a potential intruder — raising a safety
alert and a per-segment obstacle report, and exposing its state to the ARISE middleware
(FIWARE / NGSI-LD). It runs end-to-end **without any robot hardware** from a recorded bag.

- **Start reading:** [`docs/00_overview.md`](docs/00_overview.md) (rendered at the
  [docs site](https://slapstack.github.io/rises/docs.html)).
- **Validation evidence:** [`docs/05_role_in_demonstrator.md`](docs/05_role_in_demonstrator.md)
  and the [KPI dashboard](https://slapstack.github.io/rises/kpi-dashboard.html).

---

## Repository structure (D4 §3.2.4)

This is a ROS 2 / colcon workspace — the ROS packages live under `src/` and the Docker
build copies that directory in with a single `COPY src/ src/`. This follows the §3.2.4
recommended layout directly; the table below maps every recommended item to its location
here.

| §3.2.4 item | Where in this repo |
|---|---|
| `README.md` | this file |
| `LICENSE` | [`LICENSE`](LICENSE) (Apache-2.0) + [`NOTICE`](NOTICE) (third-party attributions) |
| `docs/` | [`docs/`](docs/) — `00_overview` + `01_arise_context` … `06_limitations` + `architecture.md` (same files render at the Pages site) |
| `src/` or `ros2_ws/` | [`src/`](src/) — the ROS 2 packages live here directly (see tree below); the Docker build copies them in with a single `COPY src/ src/`. This is the §3.2.4-recommended layout |
| `examples/` | [`examples/`](examples/) — hello-world & basic-demo entry, sample payloads; the recorded bag is a [GitHub Release](https://github.com/SLAPStack/rises/releases) asset |
| `launch/` | [`launch/`](launch/) (pointer) → [`rises_bringup/launch/`](rises_bringup/launch/) + per-package `*/launch/` |
| `config/` | [`config/`](config/) (pointer) → [`rises_bringup/config/`](rises_bringup/config/), [`fiware/config/`](fiware/config/), [`orchestration/configs/`](orchestration/configs/) |
| `media/` | [`media/`](media/) — `architecture_diagram.png`, `diagrams/`, `screenshots/`, `video_link.md` |
| `docker/` | [`docker/`](docker/) (pointer) → `*.dockerfile`, `build_container.sh`, [`fiware/docker-compose.yaml`](fiware/docker-compose.yaml) |

---

## Quick start (hardware-free)

No robot required — a recorded bag drives the pipeline and a dockerised FIWARE stack
stores the results. Full instructions:
[`docs/03_installation_and_hello_world.md`](docs/03_installation_and_hello_world.md).

```bash
# 1. build the core images (ROS 2 Vulcanexus Jazzy)
./build_container.sh                 # -> rises:base, rises:unity
# 2. bring up FIWARE (Orion-LD + Mintaka + TimescaleDB + Grafana)
docker compose -f fiware/docker-compose.yaml -p rises-fiware up -d
# 3. replay the 300 s warehouse bag through geofence + validation + bridge
python3 -m orchestration.cli -c orchestration/configs/andros_moving_obstacles_smoke.yaml
# 4. KPIs are exported to logs/kpis_*.csv
```

> The recorded bag is ~1 GB and is distributed as a **GitHub Release** asset (too large
> for git) — see [`examples/`](examples/) for the download + placement steps. To drive
> the **full ARISE stack** (skill bridge + mission controller) via intents, see
> [`docs/INTERACT_FULL_ARISE.md`](docs/INTERACT_FULL_ARISE.md).

---

## Directory structure

```
.
├── README.md  LICENSE  NOTICE
├── docs/                            # 00–06 + architecture.md  (+ Pages site: index/docs/kpi .html)
├── examples/                        # hello-world & basic-demo entry, sample payloads
├── media/                           # architecture_diagram.png, diagrams/, screenshots/, video_link.md
├── launch/  config/  docker/        # pointer READMEs -> real locations (see each)
│
├── src/                             # ROS 2 / colcon workspace packages
│   ├── geofence/                    # core module: spatial + gridmap + validation nodes
│   ├── laserscan_preprocessor/      # LiDAR preprocessing / segmentation
│   ├── safety/                      # safety + diagnostics health monitor
│   ├── obstacle_heatmap_predictor/  # predictive obstacle heatmap
│   ├── rises_leg_filter/            # ROS4HRI human-leg detection
│   ├── message_translator/          # message bridging / translation
│   ├── fleet_interface/             # fleet-level interface
│   ├── fiware_bridge/               # NGSI-LD bridge (ROS -> Orion-LD)
│   ├── rises_skill_bridge/          # ARISE skill action servers
│   ├── rises_mission_controller/    # Intent -> Mission -> Task -> Skill orchestration
│   ├── rises_task_examples/         # reference tasks composing geofence skills (run on demand)
│   ├── obstacle_aggregator/         # multi-AGV obstacle-report fan-in (optional)
│   ├── rises_interfaces/            # custom messages / services
│   ├── rises_bringup/               # launch/ + config/ for the whole stack
│   └── test_support/                # shared test utilities
│
├── orchestration/                   # hardware-free run harness + scenario configs
├── fiware/                          # docker-compose + DDS-Enabler + NGSI-LD config
├── resources/                       # rviz layouts, params, demo map fixtures
├── scripts/                         # KPI export, intent helper, bag utilities
├── rises.dockerfile  central.dockerfile
└── build_container.sh  entrypoint.sh  central_entrypoint.sh  unity_entrypoint.sh
```

---

## Building the images

All Dockerfiles build from the **repo root** as the context (they `COPY src/ src/` to
pull in the colcon workspace). See [`docker/`](docker/) for the full image list.

```bash
# convenience script: rises:base + rises:central + rises:unity (from central.dockerfile)
./build_container.sh

# or the full ARISE-stack AGV image directly
docker build -f rises.dockerfile --target base -t rises:base .
```

Base image: `eprosima/vulcanexus:jazzy-base` (ROS 2 Jazzy + colcon + rosdep). Compiler
and build options are `ARG`-configurable via `--build-arg`:

| Argument | Default | Description |
|---|---|---|
| `CC` / `CXX` | `clang` / `clang++` | C / C++ compiler |
| `LINKER` | `lld` | Linker |
| `CXX_STD` | `17` | C++ standard |
| `BUILD_TYPE` | `Release` | CMake build type |
| `CMAKE_GENERATOR` | `Ninja` | CMake generator |
| `OPT_LEVEL` | `-O3` | Optimisation flags |
| `LTO` / `LTO_FLAG` | `ON` / `-flto=thin` | Link-time optimisation |
| `TARGET_ARCH` | `-march=native` | Target architecture |
| `BUILD_JOBS` | `0` (auto) | Parallel build jobs (`0` = auto-detect) |

```bash
docker build -f central.dockerfile --target base -t rises:base --build-arg BUILD_JOBS=8 .
```

> The hardware-free demos build the images automatically through the `orchestration`
> CLI, so a manual build is only needed for development.

---

## License & attribution

- **Module code:** Apache-2.0 — see [`LICENSE`](LICENSE).
- **Third-party components:** see [`NOTICE`](NOTICE). The FIWARE services used by the
  demo (Orion-LD, Mintaka) are **AGPL-3.0** and run as **separate, unmodified
  containers** via `fiware/docker-compose.yaml`; they are not linked into the module
  code, so the module remains Apache-2.0.
- **Maintainer:** SLAPStack GmbH — repository <https://github.com/SLAPStack/rises>.

Co-funded by the European Union (ARISE Open Call 1, experiment **RISES**).
