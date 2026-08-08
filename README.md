# livo-slam

A LiDAR-visual-inertial fusion localization and mapping system.

## Requirements

- Docker
- VS Code with the Dev Containers extension

Nothing else is installed on the host. All builds run inside the container.

## Getting started

Open the repository in VS Code and run **Dev Containers: Reopen in Container**.
This might take a few minutes to set up the first time.

Then, in the container terminal:

```bash
cmake --preset relwithdebinfo
cmake --build --preset relwithdebinfo
ctest --preset relwithdebinfo
```

Replace the preset with `release` or `debug` as needed. Alternatively, you can
build using the VS Code task with `F7`.

The first configure clones and builds the dependencies and takes a few minutes.

Build output goes to `build/<preset>/`. Tests are controlled by
`LIVO_SLAM_BUILD_TESTS` (default `ON`).

## Run

```bash
./build/relwithdebinfo/src/livo_slam # Or debug/release
```

In VS Code, F5 builds and launches under the debugger.

Topics come from `config/livo_slam.yaml`:

```bash
./build/relwithdebinfo/src/livo_slam --ros-args --params-file config/livo_slam.yaml
```

## Visualization

Rerun runs entirely inside the container. Start the viewer, then the node:

```bash
rerun --serve-web --port 9876 --web-viewer-port 9090 &
./build/relwithdebinfo/src/livo_slam --ros-args --params-file config/livo_slam.yaml
```

Open `http://127.0.0.1:9090` in a browser on the host. The devcontainer uses host
networking, so no port mapping is needed.

Logged entities: `world/imu` (pose and axes), `world/trajectory`, `world/cloud`.
Timelines: `group` and `sensor_time`.

To turn it off, set `viz.enabled: false` in the config, or build with
`-DLIVO_SLAM_ENABLE_RERUN=OFF`, which drops the dependency entirely and compiles the
visualizer as no-ops.

## Packaged image

```bash
docker build --target runtime -t livo-slam:runtime .
docker run --rm livo-slam:runtime livo_slam
```
