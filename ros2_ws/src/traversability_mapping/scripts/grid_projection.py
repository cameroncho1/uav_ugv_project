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
