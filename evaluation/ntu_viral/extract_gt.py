#!/usr/bin/env python3
"""Write the NTU VIRAL Leica prism ground truth to a TUM trajectory file.

Reads /leica/pose/relative straight out of the original ROS1 bag, so it needs
no ROS installation. Run once per sequence and commit the result.

    ./extract_gt.py <sequence>.bag eee_03/gt.tum
"""

import struct
import sys
from pathlib import Path

GT_TOPIC = "/leica/pose/relative"

OP_MESSAGE_DATA = 0x02
OP_CHUNK = 0x05
OP_CONNECTION = 0x07


def read_fields(buf):
    fields = {}
    offset = 0
    while offset < len(buf):
        (length,) = struct.unpack_from("<I", buf, offset)
        offset += 4
        key, _, value = buf[offset : offset + length].partition(b"=")
        offset += length
        fields[key.decode()] = value
    return fields


def read_records(buf):
    offset = 0
    while offset < len(buf):
        (header_len,) = struct.unpack_from("<I", buf, offset)
        offset += 4
        header = read_fields(buf[offset : offset + header_len])
        offset += header_len
        (data_len,) = struct.unpack_from("<I", buf, offset)
        offset += 4
        data = buf[offset : offset + data_len]
        offset += data_len
        yield header, data


def parse_pose_stamped(data):
    """seq, stamp, frame_id, then position (3) and orientation (4) as float64."""
    sec, nsec = struct.unpack_from("<II", data, 4)
    (frame_len,) = struct.unpack_from("<I", data, 12)
    offset = 16 + frame_len
    values = struct.unpack_from("<7d", data, offset)
    return sec + nsec * 1e-9, values


def extract(bag_path):
    raw = Path(bag_path).read_bytes()
    if not raw.startswith(b"#ROSBAG V2.0\n"):
        raise SystemExit(f"{bag_path} is not a ROS1 bag")

    connections = set()
    poses = []
    for header, data in read_records(raw[13:]):
        op = header.get("op", b"")[:1]
        if op == bytes([OP_CONNECTION]):
            if header.get("topic", b"").decode() == GT_TOPIC:
                connections.add(header["conn"])
        elif op == bytes([OP_CHUNK]):
            if header.get("compression", b"none") != b"none":
                raise SystemExit("compressed chunks are not supported")
            for inner_header, inner_data in read_records(data):
                inner_op = inner_header.get("op", b"")[:1]
                if inner_op == bytes([OP_CONNECTION]):
                    if inner_header.get("topic", b"").decode() == GT_TOPIC:
                        connections.add(inner_header["conn"])
                elif inner_op == bytes([OP_MESSAGE_DATA]):
                    if inner_header.get("conn") in connections:
                        poses.append(parse_pose_stamped(inner_data))
    return poses


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)

    poses = extract(sys.argv[1])
    if not poses:
        raise SystemExit(f"no messages on {GT_TOPIC}")
    poses.sort(key=lambda entry: entry[0])

    out = Path(sys.argv[2])
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w") as handle:
        for timestamp, values in poses:
            handle.write(
                f"{timestamp:.9f} "
                + " ".join(f"{v:.9f}" for v in values)
                + "\n"
            )

    print(f"{len(poses)} poses -> {out}")
    print(f"span {poses[-1][0] - poses[0][0]:.2f} s")


if __name__ == "__main__":
    main()
