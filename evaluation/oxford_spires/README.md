# Oxford Spires evaluation

Ground truth comes from registering the Hesai scans against a Leica RTC360
terrestrial scan, so it is centimetre-accurate and carries real orientations.
Both ATE and RPE are meaningful, unlike NTU VIRAL.

Ground truth is the pose of `base`, an artificial frame at the bottom of the
device, while we output the IMU pose. Umeyama alignment cannot absorb the
difference because it is a body-frame offset and the device rotates.

| Script                     | Does                                                                             |
| -------------------------- | -------------------------------------------------------------------------------- |
| `apply_base_transform.py`  | `trajectory.tum` (IMU frame) -> `<sequence>/estimate_gt.tum` (base frame).       |
| `evaluate.py`              | Runs `evo_ape` and `evo_rpe` over every sequence folder here, prints a table.    |

`gt.tum` per sequence is `processed/trajectory/gt-tum.txt` from the dataset,
already in TUM format, so there is no extraction step.

Produce a trajectory, then evaluate:

```bash
cd evaluation/oxford_spires
./apply_base_transform.py ../trajectory.tum 2024-05-20-bodleian-library-02/estimate_gt.tum
./evaluate.py
```
