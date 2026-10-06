"""
lidar_parser.py
Parses a raw LiDAR capture folder into a fused, confidence-filtered 3D
point cloud in world coordinates.

Expected folder layout (point this at the hash-named folder):
    depth/NNNNNN.png        16-bit, millimeters, 256x192
    confidence/NNNNNN.png   0=low, 1=medium, 2=high
    odometry.csv            timestamp,frame,x,y,z,qx,qy,qz,qw,fx,fy,cx,cy,...
    camera_matrix.csv       3x3 intrinsics at RGB resolution (fallback)

Dependencies: numpy, scipy, Pillow.
"""
import csv
import os
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation


def load_camera_matrix(capture_dir):
    path = os.path.join(capture_dir, "camera_matrix.csv")
    return np.loadtxt(path, delimiter=",")


def load_odometry(capture_dir):
    path = os.path.join(capture_dir, "odometry.csv")
    frames = []
    with open(path) as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        for row in reader:
            if not row:
                continue
            row = [v.strip() for v in row]
            rec = dict(zip(header, row))

            def _f(key, default=None):
                v = rec.get(key, "")
                try:
                    return float(v)
                except (ValueError, TypeError):
                    return default

            frames.append({
                "frame": int(rec["frame"]),
                "timestamp": _f("timestamp"),
                "position": np.array([_f("x"), _f("y"), _f("z")]),
                "quat_xyzw": np.array([_f("qx"), _f("qy"), _f("qz"), _f("qw")]),
                "fx": _f("fx"), "fy": _f("fy"), "cx": _f("cx"), "cy": _f("cy"),
            })
    frames.sort(key=lambda r: r["frame"])
    return frames


def _depth_frame_path(capture_dir, frame_idx):
    return os.path.join(capture_dir, "depth", f"{frame_idx:06d}.png")


def _confidence_frame_path(capture_dir, frame_idx):
    return os.path.join(capture_dir, "confidence", f"{frame_idx:06d}.png")


def depth_frame_to_camera_points(depth_mm, confidence, fx, fy, cx, cy,
                                  min_confidence=2, pixel_stride=4,
                                  min_depth_m=0.15, max_depth_m=6.0):
    h, w = depth_mm.shape
    ys, xs = np.mgrid[0:h:pixel_stride, 0:w:pixel_stride]
    depth = depth_mm[ys, xs].astype(np.float64) / 1000.0
    conf = confidence[ys, xs]

    valid = (depth > min_depth_m) & (depth < max_depth_m) & (conf >= min_confidence)
    xs_v, ys_v, depth_v = xs[valid], ys[valid], depth[valid]
    if len(depth_v) == 0:
        return np.zeros((0, 3))

    x_cam = (xs_v - cx) / fx * depth_v
    y_cam = (ys_v - cy) / fy * depth_v
    z_cam = depth_v
    return np.stack([x_cam, y_cam, z_cam], axis=1)


def camera_to_world(points_cam, position, quat_xyzw, axis_fix=None):
    if axis_fix is not None:
        points_cam = points_cam @ axis_fix.T
    rot = Rotation.from_quat(quat_xyzw)
    return rot.apply(points_cam) + position


def build_point_cloud(capture_dir, frame_stride=15, pixel_stride=4,
                       min_confidence=2, axis_fix=None):
    odometry = load_odometry(capture_dir)
    cam_matrix = load_camera_matrix(capture_dir)
    fx_rgb, fy_rgb = cam_matrix[0, 0], cam_matrix[1, 1]
    cx_rgb, cy_rgb = cam_matrix[0, 2], cam_matrix[1, 2]

    depth_dir = os.path.join(capture_dir, "depth")
    sample_name = sorted(os.listdir(depth_dir))[0]
    depth_h, depth_w = np.array(Image.open(os.path.join(depth_dir, sample_name))).shape

    all_points = []
    frames_used = 0
    for rec in odometry[::frame_stride]:
        frame_idx = rec["frame"]
        d_path, c_path = _depth_frame_path(capture_dir, frame_idx), _confidence_frame_path(capture_dir, frame_idx)
        if not (os.path.exists(d_path) and os.path.exists(c_path)):
            continue
        depth_mm = np.array(Image.open(d_path))
        confidence = np.array(Image.open(c_path))

        fx, fy, cx, cy = rec["fx"], rec["fy"], rec["cx"], rec["cy"]
        if fx is None or np.isnan(fx):
            fx, fy, cx, cy = fx_rgb, fy_rgb, cx_rgb, cy_rgb

        scale_x = depth_w / (2 * cx)
        scale_y = depth_h / (2 * cy)
        fx_d, fy_d, cx_d, cy_d = fx * scale_x, fy * scale_y, cx * scale_x, cy * scale_y

        pts_cam = depth_frame_to_camera_points(
            depth_mm, confidence, fx_d, fy_d, cx_d, cy_d,
            min_confidence=min_confidence, pixel_stride=pixel_stride,
        )
        if len(pts_cam) == 0:
            continue
        pts_world = camera_to_world(pts_cam, rec["position"], rec["quat_xyzw"], axis_fix=axis_fix)
        all_points.append(pts_world)
        frames_used += 1

    if not all_points:
        raise RuntimeError(f"No valid points extracted from {capture_dir}")
    print(f"[lidar_parser] fused {frames_used} frames -> {sum(len(p) for p in all_points)} points")
    return np.concatenate(all_points, axis=0)