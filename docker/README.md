# docker/

Container build files live at the repository root because the Docker build context is
the repo root and the Dockerfiles `COPY src/ src/` to pull in the colcon workspace
(the ROS 2 packages live under `src/`); the `orchestration/` harness also references
them by name. This pointer page maps the §3.2.4 `docker/` item to their real locations.

| File | Purpose |
|---|---|
| [`../central.dockerfile`](../central.dockerfile) | Multi-stage **base / central / unity** image — core geofence + translator. `rises:base` is the build base for the AGV image |
| [`../rises.dockerfile`](../rises.dockerfile) | **Full ARISE-stack** AGV image — geofence + skill bridge + mission controller + heatmap + obstacle aggregator + FIWARE bridge |
| [`../fiware/docker-compose.yaml`](../fiware/docker-compose.yaml) | FIWARE stack — Orion-LD, Mintaka, MongoDB, TimescaleDB, Grafana |
| [`../fiware/dds-enabler/Dockerfile`](../fiware/dds-enabler/Dockerfile) | eProsima DDS-Enabler bridge image |
| [`../build_container.sh`](../build_container.sh) | Convenience build script (`rises:base`, `rises:central`, `rises:unity`) |

## Build (from the repo root)

```bash
# core base + central/unity images
./build_container.sh
# or build the full ARISE AGV image directly
docker build -f rises.dockerfile --target base -t rises:base .
```

The hardware-free demos build these images automatically via the orchestration CLI —
see [`docs/03_installation_and_hello_world.md`](../docs/03_installation_and_hello_world.md).
