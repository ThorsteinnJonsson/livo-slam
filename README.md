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

In VS Code, F5 builds and launches under the debugger, asking which dataset to
run first.

`config/` holds one file per dataset. Topics and calibration come from the one
you pass:

```bash
./build/relwithdebinfo/src/livo_slam --ros-args --params-file config/ntu_viral.yaml
```

`scripts/select_dataset.sh ntu_viral/eee_03_mcap` points `config/active.yaml`
and `datasets/active` at a dataset and sequence. This is what F5 uses, and what
`scripts/play_bag.sh` plays when given no argument.

## Visualization

Rerun runs entirely inside the container. Start the viewer, then the node:

```bash
rerun --serve-web --port 9877 --web-viewer-port 9091 &
./build/relwithdebinfo/src/livo_slam --ros-args --params-file config/default.yaml
```

Open the URL that `rerun --serve-web` prints on startup. It carries the gRPC
endpoint as a query parameter:

```
http://127.0.0.1:9091?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9877%2Fproxy
```

Opening `http://127.0.0.1:9091` without the `url` parameter shows the welcome
page, because the viewer has no data source selected.

The devcontainer uses host networking, so no port mapping is needed. For the
same reason port 9877 can collide with something already running on the host (VS
Code binds it in some setups). To move it, pass `--port <n>` to `rerun` and set
`viz.grpc_url` in the config to match.

Logged entities: `world/imu` (pose and axes), `world/trajectory`, `world/cloud`.
Timelines: `group` and `sensor_time`.

To turn it off, set `viz.enabled: false` in the config, or build with
`-DLIVO_SLAM_ENABLE_RERUN=OFF`, which drops the dependency entirely and compiles
the visualizer as no-ops.

## Packaged image

```bash
docker build --target runtime -t livo-slam:runtime .
docker run --rm --network=host --ipc=host livo-slam:runtime
```

The image entrypoint starts the Rerun viewer, waits for it, prints the viewer
URL, and then runs the node. Host networking is required for the browser to
reach the viewer, and shared IPC for DDS to communicate with other containers.

Override ports and config with `RERUN_GRPC_PORT`, `RERUN_WEB_PORT`, or
`LIVO_SLAM_CONFIG`. To pass extra arguments to the node, name the launcher
explicitly, since arguments after the image replace the default command rather
than appending to it:

```bash
docker run --rm --network=host --ipc=host -e RERUN_WEB_PORT=9092 \
  livo-slam:runtime livo-slam-launch -p viz.enabled:=false
```

The image includes the `ros2 bag` CLI. Set `LIVO_SLAM_BAG` and the launcher
replays a recording against the node itself:

```bash
docker run --rm --network=host --ipc=host -v $PWD/datasets:/data \
  -e LIVO_SLAM_BAG=/data/ntu_viral/eee_03_mcap livo-slam:runtime
```

The launcher waits for the node to appear in `ros2 node list` before starting
playback. When playback finishes the container keeps running so the recording
stays viewable in the browser; stop it with Ctrl-C or `docker stop`.
