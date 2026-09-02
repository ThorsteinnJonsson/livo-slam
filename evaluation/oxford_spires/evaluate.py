#!/usr/bin/env python3
"""Run evo over every sequence folder here and print a summary table.

A sequence folder qualifies when it holds both gt.tum and estimate_gt.tum.

The ground truth carries real orientations, registered against a terrestrial
laser scan, so unlike NTU VIRAL both ATE and RPE are meaningful here.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

RMSE = re.compile(r"^\s*rmse\s+([0-9.eE+-]+)\s*$", re.M)
PAIRS = re.compile(r"Compared (\d+) absolute pose pairs")


def run(command):
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        tail = result.stderr.strip().splitlines()
        return None, (tail[-1] if tail else " ".join(command) + " failed")
    return result.stdout, None


def evaluate(folder, t_max_diff, delta):
    gt = str(folder / "gt.tum")
    estimate = str(folder / "estimate_gt.tum")
    common = ["--align", "--t_max_diff", str(t_max_diff)]

    out, error = run(["evo_ape", "tum", gt, estimate, "-r", "trans_part",
                      *common, "-v"])
    if error:
        return {"error": error}
    ate = RMSE.search(out)
    pairs = PAIRS.search(out)

    result = {"ate": float(ate.group(1)) if ate else None,
              "pairs": int(pairs.group(1)) if pairs else None}

    for label, relation in (("rpe_trans", "trans_part"), ("rpe_rot", "angle_deg")):
        out, error = run(["evo_rpe", "tum", gt, estimate, "-r", relation,
                          "--delta", str(delta), "--delta_unit", "m", *common])
        rmse = RMSE.search(out) if out else None
        result[label] = float(rmse.group(1)) if rmse else None

    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--t_max_diff", type=float, default=0.05)
    parser.add_argument("--delta", type=float, default=1.0,
                        help="RPE delta in metres")
    args = parser.parse_args()

    folders = sorted(
        path
        for path in HERE.iterdir()
        if path.is_dir()
        and (path / "gt.tum").is_file()
        and (path / "estimate_gt.tum").is_file()
    )
    if not folders:
        raise SystemExit(
            f"no sequence folders with gt.tum and estimate_gt.tum under {HERE}")

    name_width = max(len(f.name) for f in folders)
    header = (f"{'sequence':<{name_width}}  {'pairs':>6}  {'ATE RMSE [m]':>12}"
              f"  {'RPE trans [m]':>13}  {'RPE rot [deg]':>13}")
    print(header)
    print("-" * len(header))
    for folder in folders:
        result = evaluate(folder, args.t_max_diff, args.delta)
        if "error" in result:
            print(f"{folder.name:<{name_width}}  {result['error']}")
            continue
        print(f"{folder.name:<{name_width}}  {result['pairs']:>6}  "
              f"{result['ate']:>12.3f}  {result['rpe_trans']:>13.3f}  "
              f"{result['rpe_rot']:>13.3f}")


if __name__ == "__main__":
    main()
