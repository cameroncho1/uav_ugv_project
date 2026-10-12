"""WGS84/map frame operations without ROS or transport dependencies."""

import math

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(latitude_deg, longitude_deg, altitude):
    if (not all(math.isfinite(v) for v in (latitude_deg, longitude_deg, altitude))
            or abs(latitude_deg) > 90 or abs(longitude_deg) > 180):
        raise ValueError("Invalid WGS84 coordinates")
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude = math.sin(latitude)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude**2)
    return (
        (radius + altitude) * math.cos(latitude) * math.cos(longitude),
        (radius + altitude) * math.cos(latitude) * math.sin(longitude),
        (radius * (1.0 - WGS84_E2) + altitude) * sin_latitude,
    )


def geodetic_to_enu(latitude, longitude, altitude, origin):
    x, y, z = geodetic_to_ecef(latitude, longitude, altitude)
    east_x, east_y, east_z = origin[3]
    north_x, north_y, north_z = origin[4]
    up_x, up_y, up_z = origin[5]
    dx, dy, dz = x - origin[0], y - origin[1], z - origin[2]
    return (
        east_x * dx + east_y * dy + east_z * dz,
        north_x * dx + north_y * dy + north_z * dz,
        up_x * dx + up_y * dy + up_z * dz,
    )


def make_enu_origin(latitude_deg, longitude_deg, altitude):
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_latitude, cos_latitude = math.sin(latitude), math.cos(latitude)
    sin_longitude, cos_longitude = math.sin(longitude), math.cos(longitude)
    return (*geodetic_to_ecef(latitude_deg, longitude_deg, altitude),
            (-sin_longitude, cos_longitude, 0.0),
            (-sin_latitude * cos_longitude, -sin_latitude * sin_longitude, cos_latitude),
            (cos_latitude * cos_longitude, cos_latitude * sin_longitude, sin_latitude))


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def diagonal_covariance(values):
    """Return a flattened covariance with the given diagonal."""
    size = len(values)
    result = [0.0] * (size * size)
    for index, value in enumerate(values):
        result[index * (size + 1)] = value
    return result


def enu_to_geodetic(east, north, up, origin):
    """Invert the local tangent-plane projection; used by sensor synthesis."""
    x, y, z = (origin[i] + east * origin[3][i] + north * origin[4][i]
               + up * origin[5][i] for i in range(3))
    p = math.hypot(x, y)
    longitude = math.atan2(y, x)
    latitude = math.atan2(z, p * (1 - WGS84_E2))
    for _ in range(12):
        radius = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(latitude)**2)
        latitude = math.atan2(z + WGS84_E2 * radius * math.sin(latitude), p)
    radius = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(latitude)**2)
    altitude = (p / math.cos(latitude) - radius if abs(math.cos(latitude)) > 1e-8
                else abs(z) - radius * (1 - WGS84_E2))
    return math.degrees(latitude), math.degrees(longitude), altitude
