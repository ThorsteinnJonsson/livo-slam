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
      - Change pinhole to undistort in projection (+ Jacobian changes)
      - Implement fisheye
      - Check results
- Visualize visual points + camera stream
- Validate camera update
- Add real color to pointcloud
- Refactoring
  - Helper for transforming points (instead of a for-loop everywhere)
  - Less verbose parameter parsing from yaml
- Limit patches to 30 per visual point
- Implement slide-function in local map
- Multi-camera, multi-lidar support

Phase 1 — radtan into the projection, on NTU

1. Project: take x/z, y/z, apply the radtan polynomial (k1, k2 radial, p1, p2
   tangential), then scale by fx, fy and add cx, cy. Keep the z <= 0 rejection.
2. ProjectionJacobian: derivative of the above through to ∂u/∂p_cam. This is the
   part that feeds the iEKF.
3. Tests, before touching anything else. Two kinds: check Project against
   cv::projectPoints on random points, which pins the model against a reference
   implementation; and extend the existing finite-difference check to the new
   Jacobian. A wrong derivative degrades the estimate silently, so this gate
   matters more than the ATE run.
4. Pipeline: drop the image*undistorter*.Undistort call, feed the raw image to
   cv::buildPyramid.
5. Delete src/core/camera/undistort.{h,cpp}, the image*undistorter* member, and
   the CMake entry.
6. Run NTU eee_03, compare against 0.051 m with the visual update enabled. Small
   movement expected, large jump means a bug.

Phase 2 — equidistant, on Spires

7. CameraParams: add the model discriminator and k3, k4 — this is the point
   where two models exist, so this is where the enum earns its place.
8. ReadConfig and the three yamls: camera.distortion_model, plus the two new
   coefficients. oxford_spires.yaml already has the real values sitting in a
   comment.
9. Project: branch on model. Equidistant is r = √(x²+y²)/z, θ = atan(r), θd =
   θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸), scale by θd/r, then intrinsics.
10. ProjectionJacobian: matching branch.
11. Tests: same two kinds, against cv::fisheye::projectPoints and finite
    differences.
12. Run Spires, visual update enabled, against the 0.432 m lidar-inertial
    baseline.
