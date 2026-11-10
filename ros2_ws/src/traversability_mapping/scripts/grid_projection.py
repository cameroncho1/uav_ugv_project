"""Flat-ground camera projection without ROS callbacks or simulator truth."""

import cv2
import numpy as np


def robot_projection_mask(x, y, camera, center, radius, height):
    """Ground rays intersecting the known robot's vertical bounding cylinder."""
    if not 0 < height < camera[2] or radius <= 0:
        raise ValueError("Robot bounds must lie below the camera")
    ground = np.stack((x, y), axis=-1)
    top = ground + (camera[:2] - ground) * height / camera[2]
    delta = top - ground
    offset = np.asarray(center[:2]) - ground
    length2 = np.sum(delta * delta, axis=-1)
    alpha = np.clip(np.sum(offset * delta, axis=-1) / np.maximum(length2, 1e-15), 0., 1.)
    nearest = ground + alpha[:, :, None] * delta
    return np.sum((nearest - center[:2])**2, axis=-1) <= radius**2


def project_grid(gray, intrinsic, distortion, camera_position, map_from_optical,
                 resolution, threshold):
    height, width = gray.shape
    k = np.asarray(intrinsic, dtype=float).reshape(3, 3)
    d = np.asarray(distortion, dtype=float)
    if (not np.isfinite(k).all() or not np.isfinite(d).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0 or camera_position[2] <= .5):
        raise ValueError("Invalid camera geometry")
    gray = cv2.undistort(gray, k, d) if np.any(d) else gray
    corners = np.array([[0., 0., 1.], [width - 1., 0., 1.],
                        [width - 1., height - 1., 1.], [0., height - 1., 1.]])
    rays = (map_from_optical @ np.linalg.inv(k) @ corners.T).T
    if np.any(rays[:, 2] >= -.01):
        raise ValueError("Camera footprint crosses the ground horizon")
    points = camera_position - camera_position[2] / rays[:, 2, None] * rays
    origin = np.floor(points[:, :2].min(axis=0) / resolution) * resolution
    extent = points[:, :2].max(axis=0) - origin
    dimensions = np.ceil(extent / resolution).astype(int)
    nx, ny = dimensions
    if nx <= 0 or ny <= 0 or nx * ny > 100000:
        raise ValueError("Camera grid dimensions exceed the processing limit")
    x, y = np.meshgrid(origin[0] + (np.arange(nx) + .5) * resolution,
                       origin[1] + (np.arange(ny) + .5) * resolution)
    luminance = np.full(x.shape, 255.)
    visible = np.ones(x.shape, bool)
    for ox, oy in ((0., 0.), (-.25, -.25), (-.25, .25), (.25, -.25), (.25, .25)):
        world = np.stack((x + ox * resolution, y + oy * resolution, np.zeros(x.shape)), axis=-1)
        optical = (world - camera_position) @ map_from_optical
        projected = optical @ k.T
        positive = optical[:, :, 2] > .1
        denominator = np.where(positive, projected[:, :, 2], 1.)
        px = np.rint(projected[:, :, 0] / denominator).astype(int)
        py = np.rint(projected[:, :, 1] / denominator).astype(int)
        good = positive & (px >= 0) & (px < width) & (py >= 0) & (py < height)
        visible &= good
        samples = gray[np.clip(py, 0, height - 1), np.clip(px, 0, width - 1)]
        luminance = np.minimum(luminance, samples)
    grid = np.where(visible, np.where(luminance < threshold, 100, 0), -1).astype(np.int8)
    return origin, grid, x, y


def fuse_into_map(log_odds, seen, map_origin, resolution, frame_origin, frame,
                  hit=.6, miss=.6, low=-1.5, high=2.5):
    """Add one camera grid (-1 unseen, 0 free, 100 occupied) to a persistent map in place.

    Each observation moves a cell's log-odds toward occupied or free, clamped so that a
    long-seen obstacle clears after a few free views and one dark view marks a new cell.
    Both origins must lie on the same cell lattice; cells outside the map are ignored.
    """
    shift = (np.asarray(frame_origin[:2], float) - np.asarray(map_origin[:2], float)) / resolution
    offset = np.rint(shift).astype(int)
    if np.any(np.abs(shift - offset) > 1e-6):
        raise ValueError("Camera grid is not aligned with the map lattice")
    rows, cols = frame.shape
    map_rows, map_cols = log_odds.shape
    x0, y0 = int(offset[0]), int(offset[1])
    fx0, fy0 = max(0, -x0), max(0, -y0)
    fx1, fy1 = min(cols, map_cols - x0), min(rows, map_rows - y0)
    if fx1 <= fx0 or fy1 <= fy0:
        return
    part = frame[fy0:fy1, fx0:fx1]
    target = (slice(y0 + fy0, y0 + fy1), slice(x0 + fx0, x0 + fx1))
    observed = part >= 0
    view = log_odds[target]
    view[observed] = np.clip(view[observed] + np.where(part[observed] >= 50, hit, -miss),
                             low, high)
    seen[target] |= observed


def map_values(log_odds, seen):
    """Occupancy values for publication: -1 never seen, 100 occupied, 0 free."""
    return np.where(seen, np.where(log_odds > 0, 100, 0), -1).astype(np.int8)
