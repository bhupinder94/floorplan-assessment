"""
pipeline.py
One command, one capture: parses a raw LiDAR capture folder, fits a room,
writes the JSON output contract + a rendered top-down plan PNG.

Usage:
    python pipeline.py --capture /path/to/capture_folder --room-id room_0
"""
import argparse
import os
from dataclasses import asdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from lidar_parser import build_point_cloud
from fit_room import estimate_room, apply_horizontal_calibration
from schema import PropertyPlan


def render_plan(room, out_path):
    fig, ax = plt.subplots(figsize=(6, 6))
    xs = [w.start_xy[0] for w in room.walls] + [room.walls[0].start_xy[0]]
    ys = [w.start_xy[1] for w in room.walls] + [room.walls[0].start_xy[1]]
    ax.plot(xs, ys, "k-", linewidth=2)
    for w in room.walls:
        mid = ((w.start_xy[0] + w.end_xy[0]) / 2, (w.start_xy[1] + w.end_xy[1]) / 2)
        ax.annotate(f"{w.length.value_m:.2f}m", mid, fontsize=9, ha="center")
    ax.set_title(f"{room.room_id} | ceiling {room.ceiling_height.value_m:.2f}m "
                 f"+/- {room.ceiling_height.confidence_m:.2f} | "
                 f"area {room.floor_area_m2.value_m:.2f}m2")
    ax.axis("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")
    plt.tight_layout()
    plt.savefig(out_path, dpi=120)
    plt.close(fig)


def run(capture_dir, room_id, out_dir, frame_stride=10, pixel_stride=4, min_confidence=2,
        horizontal_calibration=None):
    os.makedirs(out_dir, exist_ok=True)
    points = build_point_cloud(capture_dir, frame_stride=frame_stride,
                                pixel_stride=pixel_stride, min_confidence=min_confidence)
    room = estimate_room(points, room_id=room_id, tier="lidar")
    if horizontal_calibration is not None:
        room = apply_horizontal_calibration(
            room, horizontal_calibration,
            evidence_note="derived from tape-measured rooms, iPhone 12 Pro + Stray Scanner")
    plan = PropertyPlan.from_single_room(room, capture_id=os.path.basename(capture_dir.rstrip("/")))

    json_path = os.path.join(out_dir, f"{room_id}.json")
    plan.to_json(json_path)
    png_path = os.path.join(out_dir, f"{room_id}_plan.png")
    render_plan(room, png_path)

    print(f"wrote {json_path}")
    print(f"wrote {png_path}")
    return plan


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture", required=True)
    ap.add_argument("--room-id", default="room_0")
    ap.add_argument("--out", default="./output")
    ap.add_argument("--frame-stride", type=int, default=10)
    ap.add_argument("--pixel-stride", type=int, default=4)
    ap.add_argument("--min-confidence", type=int, default=2)
    ap.add_argument("--horizontal-calibration", type=float, default=None)
    args = ap.parse_args()

    run(args.capture, args.room_id, args.out,
        frame_stride=args.frame_stride, pixel_stride=args.pixel_stride,
        min_confidence=args.min_confidence,
        horizontal_calibration=args.horizontal_calibration)