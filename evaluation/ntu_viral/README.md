# NTU VIRAL evaluation

Ground truth is the in-bag Leica prism, which gives position only, so every
orientation in `gt.tum` is identity. Only ATE is meaningful; RPE would compare
world-frame motion against body-frame motion.

| Script                  | Does                                                                                           |
| ----------------------- | ---------------------------------------------------------------------------------------------- |
| `extract_gt.py`         | ROS1 bag -> `<sequence>/gt.tum`. Run once, result is committed.                                |
| `apply_prism_offset.py` | `trajectory.tum` (output from our algorithm) -> `<sequence>/estimate_gt.tum`, prism lever arm. |
| `evaluate.py`           | Runs `evo_ape` over every sequence folder, prints a table.                                     |

Produce a trajectory, then evaluate:

```bash
./build/relwithdebinfo/src/livo_slam --ros-args --params-file config/livo_slam.yaml \
    -p ros.output_trajectory:=true &
ros2 bag play /data/ntu_viral/eee_03_mcap

cd evaluation/ntu_viral
./apply_prism_offset.py ../trajectory.tum eee_03/estimate_gt.tum
./evaluate.py
```
