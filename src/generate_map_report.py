"""Class Work 8 - SLAM: Explore the Unknown World.

Builds the 3 deliverables that depend on the exploration log:
  - Map             : occupancy-grid PNG (visited cells + sensed walls)
  - Robot Trajectory: PNG of the real (x, y) path with start/end marked
  - Accuracy report : Map Accuracy % and Coverage % (per the formulas in the
                      assignment) + a printed Start/End position report

Usage:
    python analysis/generate_map_report.py
    python analysis/generate_map_report.py --log path/to/log_..._exploration_map_data.csv
"""

import argparse
import glob
import json
import os
import sys

import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
try:
    from config_loader import load_config
except ImportError:
    def load_config():
        return {}


# ---------------------------------------------------------------------------
# EDIT THIS after the real maze is revealed on test day, to get true
# Map Accuracy instead of the "assume everything is free" placeholder.
# ---------------------------------------------------------------------------
GROUND_TRUTH_FREE_CELLS = None


def get_latest_file(data_dir, pattern):
    files = glob.glob(os.path.join(data_dir, pattern))
    if not files:
        return None
    return max(files, key=os.path.getctime)


def classify_wall(distance, open_above):
    """True == wall present, False == open, None == invalid."""
    if distance is None or pd.isna(distance):
        return None
    # กรองกรณีอ่านค่าหลุดหรือ ToF return 0
    if distance <= 0:
        return None
    return not (distance > open_above)


def build_wall_votes(df, wall_threshold_cm=38.0):
    """direction vote counting per cell, from every row that visited it.

    direction index: 0=N(+y) 1=E(+x) 2=S(-y) 3=W(-x), matches chassis.py.
    - front sensor covers `heading`
    - right sensor covers `(heading + 1) % 4`
    - back sensor covers `(heading + 2) % 4`  (เพิ่มใหม่)
    - left sensor covers `(heading + 3) % 4`
    """
    votes = {}  # (gx,gy) -> {dir: [wall_votes, open_votes]}

    def add_vote(cell, direction, is_wall):
        if is_wall is None:
            return
        d = votes.setdefault(cell, {0: [0, 0], 1: [0, 0], 2: [0, 0], 3: [0, 0]})
        d[direction][0 if is_wall else 1] += 1

    for _, row in df.iterrows():
        # ข้ามแถว RETRACE เพราะค่าเซนเซอร์ไม่ได้มาจากการสแกนจริงรอบตัว
        if str(row.get("action", "")).strip().upper() == "RETRACE":
            continue

        cell = (int(row["grid_x"]), int(row["grid_y"]))
        h = int(row["heading"])

        # 1. Front (mm -> cm หรือเทียบ mm ด้วยเกณฑ์ 380 mm)
        front_val_mm = row.get("front_tof_mm")
        front_wall = classify_wall(front_val_mm, wall_threshold_cm * 10)

        # 2. Right (cm)
        right_val_cm = row.get("right_ir_cm")
        right_wall = classify_wall(right_val_cm, wall_threshold_cm)

        # 3. Back (cm) - ตรวจสอบทิศด้านหลัง (heading + 2) % 4
        back_val_cm = row.get("back_ir_cm")
        if pd.isna(back_val_cm) and "back_tof_mm" in row:
            back_val_cm = row["back_tof_mm"] / 10.0
        back_wall = classify_wall(back_val_cm, wall_threshold_cm)

        # 4. Left (cm)
        left_val_cm = row.get("left_ir_cm")
        left_wall = classify_wall(left_val_cm, wall_threshold_cm)

        add_vote(cell, h, front_wall)
        add_vote(cell, (h + 1) % 4, right_wall)
        add_vote(cell, (h + 2) % 4, back_wall)
        add_vote(cell, (h + 3) % 4, left_wall)

    walls = {}
    for cell, dirs in votes.items():
        walls[cell] = {}
        for d, (wall_v, open_v) in dirs.items():
            if wall_v == 0 and open_v == 0:
                continue
            walls[cell][d] = wall_v >= open_v  # tie -> wall (conservative)
    return walls


def plot_map(df, walls, max_x, max_y, start_cell, end_cell, out_path):
    fig, ax = plt.subplots(figsize=(max(6, max_x + 2), max(6, max_y + 2)))

    visited = {(int(r["grid_x"]), int(r["grid_y"])) for _, r in df.iterrows()}

    for gx in range(max_x + 1):
        for gy in range(max_y + 1):
            color = "#E5EFF7" if (gx, gy) in visited else "#FFFFFF"
            ax.add_patch(plt.Rectangle((gx - 0.5, gy - 0.5), 1, 1, color=color, zorder=1))

    # deg_map dir -> boundary segment offset (N,E,S,W relative to +x east, +y north)
    seg = {
        0: lambda gx, gy: ((gx - 0.5, gy + 0.5), (gx + 0.5, gy + 0.5)),  # N edge
        1: lambda gx, gy: ((gx + 0.5, gy - 0.5), (gx + 0.5, gy + 0.5)),  # E edge
        2: lambda gx, gy: ((gx - 0.5, gy - 0.5), (gx + 0.5, gy - 0.5)),  # S edge
        3: lambda gx, gy: ((gx - 0.5, gy - 0.5), (gx - 0.5, gy + 0.5)),  # W edge
    }
    for (gx, gy), dirs in walls.items():
        for d, is_wall in dirs.items():
            if not is_wall:
                continue
            (x0, y0), (x1, y1) = seg[d](gx, gy)
            ax.plot([x0, x1], [y0, y1], color="black", linewidth=4, zorder=4, solid_capstyle="projecting")

    ax.plot(*start_cell, marker="s", color="#2CA02C", markersize=14, label="start", zorder=6)
    ax.plot(*end_cell, marker="o", color="#D62728", markersize=12, label="end", zorder=6)

    ax.set_xticks(range(max_x + 1))
    ax.set_yticks(range(max_y + 1))
    ax.set_xlim(-1, max_x + 1)
    ax.set_ylim(-1, max_y + 1)
    ax.set_aspect("equal")
    ax.set_xlabel("grid_x (E ->)")
    ax.set_ylabel("grid_y (N ^)")
    ax.set_title(f"SLAM Occupancy Map | visited={len(visited)}/{(max_x + 1) * (max_y + 1)} cells")
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1))
    plt.tight_layout()
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_trajectory(df, start_cell, end_cell, cell_size, out_path):
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(df["real_x_m"], df["real_y_m"], color="#1F77B4", linewidth=1.8,
            marker="o", markersize=3, zorder=2, label="trajectory (incl. retrace)")

    start_xy = (start_cell[0] * cell_size, start_cell[1] * cell_size)
    end_xy = (end_cell[0] * cell_size, end_cell[1] * cell_size)
    ax.plot(*start_xy, marker="s", color="#2CA02C", markersize=14, label="start", zorder=3)
    ax.plot(*end_xy, marker="o", color="#D62728", markersize=13, label="end", zorder=3)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.set_title("Robot Trajectory (self-localized via odometry)")
    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def compute_accuracy(visited, max_x, max_y, ground_truth_free_cells):
    total_cells = (max_x + 1) * (max_y + 1)
    if ground_truth_free_cells is None:
        ground_truth_free_cells = {(gx, gy) for gx in range(max_x + 1) for gy in range(max_y + 1)}

    correct = 0
    for gx in range(max_x + 1):
        for gy in range(max_y + 1):
            predicted_free = (gx, gy) in visited
            actual_free = (gx, gy) in ground_truth_free_cells
            if predicted_free == actual_free:
                correct += 1

    coverage_pct = round(100.0 * len(visited) / total_cells, 2) if total_cells else 0.0
    accuracy_pct = round(100.0 * correct / total_cells, 2) if total_cells else 0.0
    return coverage_pct, accuracy_pct, total_cells


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", default=None, help="Path to exploration_map_data csv (default: latest in data_dir)")
    parser.add_argument("--ground-truth-json", default=None,
                        help='JSON file with a list of [x,y] free cells, e.g. [[0,0],[0,1],...]')
    parser.add_argument("--threshold", type=float, default=38.0,
                        help="Threshold distance in cm to classify as open vs wall (default: 38.0)")
    args = parser.parse_args()

    config = load_config()
    grid_cfg = config.get("grid_map", {})
    max_x = grid_cfg.get("max_x", 3)
    max_y = grid_cfg.get("max_y", 3)
    cell_size = config.get("movement", {}).get("distance", 0.6)
    data_dir = config.get("data_collection", {}).get("data_dir", "data/raw/run1")

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir_abs = os.path.join(base_dir, data_dir)

    log_path = args.log or get_latest_file(data_dir_abs, "*exploration_map_data*.csv")
    if not log_path:
        print(f"[-] No exploration log found in {data_dir_abs}")
        return 1

    print(f"[+] Reading log: {log_path}")
    df = pd.read_csv(log_path)
    if df.empty:
        print("[-] Log file is empty")
        return 1

    visited = {(int(r["grid_x"]), int(r["grid_y"])) for _, r in df.iterrows()}
    start_cell = (int(df.iloc[0]["grid_x"]), int(df.iloc[0]["grid_y"]))

    non_retrace = df[df["action"] != "RETRACE"]
    last_real_row = non_retrace.iloc[-1] if not non_retrace.empty else df.iloc[-1]
    end_cell = (int(last_real_row["grid_x"]), int(last_real_row["grid_y"]))

    # สร้างข้อมูลกำแพงโดยใช้ threshold ที่รองรับกริด 0.6m
    walls = build_wall_votes(df, wall_threshold_cm=args.threshold)

    ground_truth_free_cells = GROUND_TRUTH_FREE_CELLS
    if args.ground_truth_json:
        with open(args.ground_truth_json, "r", encoding="utf-8") as f:
            ground_truth_free_cells = {tuple(c) for c in json.load(f)}

    coverage_pct, accuracy_pct, total_cells = compute_accuracy(
        visited, max_x, max_y, ground_truth_free_cells
    )

    map_png = os.path.join(data_dir_abs, "slam_map.png")
    traj_png = os.path.join(data_dir_abs, "slam_trajectory.png")
    plot_map(df, walls, max_x, max_y, start_cell, end_cell, map_png)
    plot_trajectory(df, start_cell, end_cell, cell_size, traj_png)

    report = {
        "log_file": log_path,
        "start_grid": list(start_cell),
        "start_real_m": [round(start_cell[0] * cell_size, 3), round(start_cell[1] * cell_size, 3)],
        "end_grid": list(end_cell),
        "end_real_m": [round(end_cell[0] * cell_size, 3), round(end_cell[1] * cell_size, 3)],
        "visited_cells": len(visited),
        "total_cells": total_cells,
        "coverage_percent": coverage_pct,
        "map_accuracy_percent": accuracy_pct,
        "ground_truth_used": ground_truth_free_cells is not None,
        "map_image": map_png,
        "trajectory_image": traj_png,
    }
    report_path = os.path.join(data_dir_abs, "slam_report.json")
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print("\n" + "=" * 60)
    print("SLAM Mission Report")
    print("=" * 60)
    print(f"Start position : grid={report['start_grid']}  real={report['start_real_m']} m")
    print(f"End position   : grid={report['end_grid']}  real={report['end_real_m']} m")
    print(f"Coverage       : {report['visited_cells']}/{total_cells} cells = {coverage_pct}%")
    print(f"Map Accuracy   : {accuracy_pct}%"
          + ("" if report["ground_truth_used"] else "  (placeholder ground truth - edit GROUND_TRUTH_FREE_CELLS!)"))
    print(f"Map image      : {map_png}")
    print(f"Trajectory img : {traj_png}")
    print(f"Report json    : {report_path}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())