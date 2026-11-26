import numpy as np
from scipy.spatial import cKDTree

LIDAR_OFFSET = np.array([0.22, 0.0, 0.11])


def scan_points(ranges, angle_min, increment, range_min, range_max, rotation):
    ranges = np.asarray(ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges < range_max - 0.02)
    indices = np.flatnonzero(valid)
    angles = angle_min + indices * increment
    distance = ranges[indices]
    rays = np.column_stack((distance * np.cos(angles), distance * np.sin(angles),
                            np.zeros(len(indices))))
    # Векторы от trunk в осях odom. Учитываем вынос лидара и наклон корпуса.
    return ((rays + LIDAR_OFFSET) @ rotation.T)[:, :2], indices


class ScanOdometry:
    def __init__(self):
        self.position = np.zeros(2)
        self.reference = None

    def set_reference(self, points, indices, yaw):
        if len(points) < 35:
            return False
        tangent = points[2:] - points[:-2]
        length = np.linalg.norm(tangent, axis=1)
        left = np.linalg.norm(points[1:-1] - points[:-2], axis=1)
        right = np.linalg.norm(points[2:] - points[1:-1], axis=1)
        valid = ((indices[2:] - indices[:-2] == 2) & (length > 0.005)
                 & (left < 0.22) & (right < 0.22))
        if np.count_nonzero(valid) < 30:
            return False
        tangent = tangent[valid] / length[valid, None]
        self.normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        self.reference = cKDTree(points[1:-1][valid] + self.position)
        self.reference_position = self.position.copy()
        self.reference_yaw = yaw
        return True

    def update(self, points, indices, yaw):
        if self.reference is None:
            self.set_reference(points, indices, yaw)
            return False
        if len(points) < 35:
            return False

        estimate = self.position.copy()
        for _ in range(6):
            distance, nearest = self.reference.query(points + estimate)
            matched = distance < 0.25
            if np.count_nonzero(matched) < max(30, len(points) // 5):
                return False
            nearest = nearest[matched]
            normals = self.normals[nearest]
            error = self.reference.data[nearest] - (points[matched] + estimate)
            residual = np.sum(normals * error, axis=1)
            weights = np.sqrt(np.minimum(1.0, 0.035 / np.maximum(np.abs(residual), 1e-9)))
            # Почти ненаблюдаемое направление (например, вдоль одной стены)
            # не восстанавливаем из шума.
            step = np.linalg.lstsq(normals * weights[:, None],
                                   residual * weights, rcond=0.16)[0]
            estimate += step
            if np.linalg.norm(step) < 0.0005:
                break

        if np.linalg.norm(estimate - self.position) > 0.18:
            return False
        self.position = estimate
        yaw_change = np.arctan2(np.sin(yaw - self.reference_yaw),
                                np.cos(yaw - self.reference_yaw))
        if np.linalg.norm(estimate - self.reference_position) > 0.16 or abs(yaw_change) > 0.25:
            self.set_reference(points, indices, yaw)
        return True
