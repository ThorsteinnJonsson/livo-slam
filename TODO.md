## Datasets to check:

- Check https://huggingface.co/datasets/DapengFeng/MCAP. mcap versions of
  datasets used by FAST-LIVO2
- Oxford Spires
- Oxford Osprey
- Cavers dataset

## Goals

- (Done) Fully implement livo-odometry inspired by FAST-LIVO2
- Support multi-lidar, multi-camera

## TODO

- Add real color to pointcloud
- Limit patches to 30 per visual point
- Implement slide-function in local map
- Test more datasets
- Refactoring
  - Helper for transforming points (instead of a for-loop everywhere)
  - Less verbose parameter parsing from yaml
