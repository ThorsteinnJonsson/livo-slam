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

## Packaged image

```bash
docker build --target runtime -t livo-slam:runtime .
docker run --rm livo-slam:runtime livo_slam
```
