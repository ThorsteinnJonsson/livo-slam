#!/usr/bin/env python3
"""Convert a livo-slam trajectory from the IMU frame to the base frame.

    ./apply_base_transform.py ../trajectory.tum <sequence>/estimate_gt.tum

The ground truth gives the pose of "base", an artificial frame at the bottom of
the device, while we output the IMU pose. Umeyama alignment cannot absorb the
difference because it is a body-frame offset and the device rotates.

Extrinsics are the published ones from configs/sensor.yaml in
ori-drs/oxford_spires_dataset. Quaternions there are xyzw.
"""

import argparse
import sys

import numpy as np

# cam_front, T_cam_lidar and T_cam_imu; and the base frame.
T_CAM_LIDAR = ([0.00035060884033447846, -0.079574053851818, -0.053826061545697405],
               [0.5034725628949727, 0.5003337192107279, -0.4947539264483171,
                0.5013981452864594])
T_CAM_IMU = ([-0.0008544600980918986, 0.005115741648037186, -0.06898747851173177],
             [0.0042909719518877815, -0.7085293318962699, 0.705668106144218,
              0.0005453193252705428])
T_BASE_LIDAR = ([0.0, 0.0, 0.124], [0.0, 0.0, 1.0, 0.0])


def matrix(translation, quaternion_xyzw):
    x, y, z, w = quaternion_xyzw
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    out = np.eye(4)
    out[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    out[:3, 3] = translation
    return out


def quaternion_xyzw(rotation):
    trace = rotation.trace()
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation[2, 1] - rotation[1, 2]) * s
        y = (rotation[0, 2] - rotation[2, 0]) * s
        z = (rotation[1, 0] - rotation[0, 1]) * s
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k])
        q = np.zeros(3)
        q[i] = 0.25 * s
        q[j] = (rotation[j, i] + rotation[i, j]) / s
        q[k] = (rotation[k, i] + rotation[i, k]) / s
        w = (rotation[k, j] - rotation[j, k]) / s
        x, y, z = q
    return np.array([x, y, z, w]) / np.linalg.norm([x, y, z, w])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source")
    parser.add_argument("destination")
    args = parser.parse_args()

    T_imu_base = (np.linalg.inv(matrix(*T_CAM_IMU)) @ matrix(*T_CAM_LIDAR)
                  @ np.linalg.inv(matrix(*T_BASE_LIDAR)))

    count = 0
    with open(args.source) as source, open(args.destination, "w") as destination:
        for line in source:
            if not line.strip() or line.startswith("#"):
                continue
            fields = line.split()
            T_world_imu = matrix([float(v) for v in fields[1:4]],
                                 [float(v) for v in fields[4:8]])
            T_world_base = T_world_imu @ T_imu_base
            t = T_world_base[:3, 3]
            q = quaternion_xyzw(T_world_base[:3, :3])
            destination.write(
                f"{fields[0]} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} "
                f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
            count += 1

    print(f"{count} poses -> {args.destination}", file=sys.stderr)


if __name__ == "__main__":
    main()
