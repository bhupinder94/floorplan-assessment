"""
fit_room.py
Fits floor height, ceiling height, and a rectangular room footprint from
a fused point cloud (Manhattan-world assumption -- known limitation).
"""
import numpy as np
from scipy import ndimage
from schema import Wall, Opening, RoomPlan, Measurement


def keep_largest_cluster(points, cell_size_m=0.1, min_points_per_cell=3):
    xz = points[:, [0, 2]]
    mins = xz.min(axis=0)
    cell_idx = np.floor((xz - mins) / cell_size_m).astype(int)
    grid_shape = cell_idx.max(axis=0) + 1

    flat_idx = cell_idx[:, 0] * grid_shape[1] + cell_idx[:, 1]
    counts = np.bincount(flat_idx, minlength=grid_shape[0] * grid_shape[1])
    counts_grid = counts.reshape(grid_shape)
    occupancy = counts_grid >= min_points_per_cell

    labeled, n_labels = ndimage.label(occupancy, structure=np.ones((3, 3)))
    if n_labels == 0:
        return points

    label_sizes = ndimage.sum(occupancy, labeled, index=range(1, n_labels + 1))
    largest_label = 1 + int(np.argmax(label_sizes))

    point_labels = labeled[cell_idx[:, 0], cell_idx[:, 1]]
    keep = point_labels == largest_label
    return points[keep]


def split_floor_ceiling(points, floor_pctile=2, ceiling_pctile=98,
                         ceiling_density_min_frac=0.01):
    y = points[:, 1]
    floor_y = np.percentile(y, floor_pctile)
    ceiling_y_candidate = np.percentile(y, ceiling_pctile)
    near_top = np.sum(np.abs(y - ceiling_y_candidate) < 0.05)
    ceiling_observed = (near_top / len(y)) > ceiling_density_min_frac
    return floor_y, (ceiling_y_candidate if ceiling_observed else None)


def minimum_area_rectangle(points_xz, angle_step_deg=1.0):
    best = None
    for angle_deg in np.arange(0, 90, angle_step_deg):
        theta = np.radians(angle_deg)
        c, s = np.cos(theta), np.sin(theta)
        rot = np.array([[c, -s], [s, c]])
        rotated = points_xz @ rot.T
        min_xy, max_xy = rotated.min(axis=0), rotated.max(axis=0)
        extent = max_xy - min_xy
        area = extent[0] * extent[1]
        if best is None or area < best[0]:
            best = (area, theta, min_xy, max_xy)

    _, theta, min_xy, max_xy = best
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    rot_inv = rot.T

    corners_rot = np.array([
        [min_xy[0], min_xy[1]], [max_xy[0], min_xy[1]],
        [max_xy[0], max_xy[1]], [min_xy[0], max_xy[1]],
    ])
    corners = corners_rot @ rot_inv.T
    width = max_xy[0] - min_xy[0]
    depth = max_xy[1] - min_xy[1]
    return corners, width, depth, theta


def apply_horizontal_calibration(room: RoomPlan, factor: float, evidence_note: str) -> RoomPlan:
    import copy
    corrected = copy.deepcopy(room)
    for w in corrected.walls:
        w.length.value_m = round(w.length.value_m * factor, 3)
        w.length.confidence_m = 0.05
    w0, w1 = corrected.walls[0].length.value_m, corrected.walls[1].length.value_m
    corrected.floor_area_m2.value_m = round(w0 * w1, 3)
    corrected.floor_area_m2.confidence_m = 0.15
    corrected.notes.append(
        f"horizontal_calibration applied: factor={factor} ({evidence_note}). "
        f"Ceiling height NOT corrected -- unvalidated against tape ground truth."
    )
    return corrected


def estimate_room(points, room_id="room_0", tier="lidar", wall_margin_m=0.08):
    points = keep_largest_cluster(points)
    floor_y, ceiling_y = split_floor_ceiling(points)

    above_floor = points[:, 1] > (floor_y + wall_margin_m)
    if ceiling_y is not None:
        below_ceiling = points[:, 1] < (ceiling_y - 0.02)
        wall_mask = above_floor & below_ceiling
    else:
        wall_mask = above_floor
    wall_points_xz = points[wall_mask][:, [0, 2]]

    if len(wall_points_xz) < 50:
        raise RuntimeError("Not enough wall points after floor/ceiling split.")

    corners, width, depth, theta = minimum_area_rectangle(wall_points_xz)

    walls = []
    for i in range(4):
        p0, p1 = corners[i], corners[(i + 1) % 4]
        length = float(np.linalg.norm(p1 - p0))
        walls.append(Wall(
            wall_id=f"wall_{i}",
            start_xy=(float(p0[0]), float(p0[1])),
            end_xy=(float(p1[0]), float(p1[1])),
            length=Measurement(value_m=round(length, 3), confidence_m=0.03, source=tier),
        ))

    if ceiling_y is not None:
        ceiling_height = Measurement(value_m=round(float(ceiling_y - floor_y), 3),
                                      confidence_m=0.015, source=tier)
        notes = []
    else:
        rough = float(points[:, 1].max() - floor_y)
        ceiling_height = Measurement(value_m=round(rough, 3), confidence_m=0.5, source=tier)
        notes = ["ceiling not confidently observed in this capture"]

    floor_area = Measurement(value_m=round(float(width * depth), 3), confidence_m=0.1, source=tier)

    return RoomPlan(
        room_id=room_id, tier=tier, walls=walls,
        ceiling_height=ceiling_height, floor_area_m2=floor_area,
        openings=[], notes=notes,
    )