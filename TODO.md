## Datasets to check:

- Oxford Spires (https://dynamic.robots.ox.ac.uk/datasets/oxford-spires/)
- Check https://huggingface.co/datasets/DapengFeng/MCAP. mcap versions of
  datasets used by FAST-LIVO (No ground truth, not Pointcloud2 msg but Hesai
  msg)
- Oxford Osprey
- Cavers dataset

## Goals

- (Done) Fully implement livo-odometry inspired by FAST-LIVO2
- Support multi-lidar, multi-camera

## TODO

- Test more datasets
  - Oxford Spires
    - (Done) Parsing the messages (Hesai points, CompressedImage)
    - Support for fish-eye camera model
- Validate camera update
- Add real color to pointcloud
- Refactoring
  - Helper for transforming points (instead of a for-loop everywhere)
  - Less verbose parameter parsing from yaml
- Limit patches to 30 per visual point
- Implement slide-function in local map
- Multi-camera, multi-lidar support
