#!/usr/bin/env python3
"""Move an estimated IMU trajectory onto the Leica prism for comparison.

The prism sits 0.40 m from the IMU, so comparing IMU positions against the
prism ground truth adds a rotation-dependent error of that size.

    ./apply_prism_offset.py trajectory.tum eee_03/estimate_gt.tum
"""

import sys
from pathlib import Path

# T_Body_Prism from the sequence's leica_prism.yaml. T_Body_Imu is identity for
# NTU VIRAL, so this is also the IMU-to-prism translation.
T_IMU_PRISM = (-0.293656, -0.012288, -0.273095)


def rotate(q, v):
    """Rotate v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)

    source = Path(sys.argv[1])
    target = Path(sys.argv[2])
    target.parent.mkdir(parents=True, exist_ok=True)

    count = 0
    with source.open() as reader, target.open("w") as writer:
        for line in reader:
            if not line.strip() or line.startswith("#"):
                continue
            timestamp, tx, ty, tz, qx, qy, qz, qw = (
                float(field) for field in line.split()
            )
            offset = rotate((qx, qy, qz, qw), T_IMU_PRISM)
            writer.write(
                f"{timestamp:.9f} "
                f"{tx + offset[0]:.9f} {ty + offset[1]:.9f} "
                f"{tz + offset[2]:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )
            count += 1

    print(f"{count} poses -> {target}")


if __name__ == "__main__":
    main()
