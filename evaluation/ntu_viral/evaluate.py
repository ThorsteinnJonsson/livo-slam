#!/usr/bin/env python3
"""Run evo over every sequence folder here and print a summary table.

A sequence folder qualifies when it holds both gt.tum and estimate_gt.tum.

Only ATE is reported. The Leica gives position only, so every ground-truth
orientation is identity and RPE would compare world-frame motion against
body-frame motion. ATE with --pose_relation trans_part never forms relative
poses and is unaffected.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

RMSE = re.compile(r"^\s*rmse\s+([0-9.eE+-]+)\s*$", re.M)
PAIRS = re.compile(r"Compared (\d+) absolute pose pairs")


def evaluate(folder, t_max_diff):
    result = subprocess.run(
        [
            "evo_ape", "tum",
            str(folder / "gt.tum"), str(folder / "estimate_gt.tum"),
            "--align", "-r", "trans_part", "--t_max_diff", str(t_max_diff),
            "-v",
        ],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        tail = result.stderr.strip().splitlines()
        return {"error": tail[-1] if tail else "evo_ape failed"}

    rmse = RMSE.search(result.stdout)
    pairs = PAIRS.search(result.stdout)
    return {
        "ate": float(rmse.group(1)) if rmse else None,
        "pairs": int(pairs.group(1)) if pairs else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--t_max_diff", type=float, default=0.05)
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
            f"no sequence folders with gt.tum and estimate_gt.tum under {HERE}"
        )

    print(f"{'sequence':<12} {'pairs':>7} {'ATE RMSE [m]':>13}")
    print("-" * 34)

    failed = False
    for folder in folders:
        result = evaluate(folder, args.t_max_diff)
        if "error" in result:
            failed = True
            print(f"{folder.name:<12} {result['error']}")
            continue
        pairs = result["pairs"] if result["pairs"] is not None else "-"
        ate = f"{result['ate']:.3f}" if result["ate"] is not None else "-"
        print(f"{folder.name:<12} {pairs:>7} {ate:>13}")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
