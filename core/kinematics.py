import numpy as np


class DifferentialDriveKinematics:
    def __init__(self, wheel_radius=0.05, track_width=0.30):
        self.r = wheel_radius
        self.L = track_width

    def inverse_kinematics(self, linear_vel, angular_vel):
        """Maps linear (v) and angular (w) velocities to left/right wheel speeds (rad/s)."""
        w_left = (linear_vel - (angular_vel * self.L / 2.0)) / self.r
        w_right = (linear_vel + (angular_vel * self.L / 2.0)) / self.r
        return float(w_left), float(w_right)

    @staticmethod
    def simulate_lidar_scan(robot_pos, obstacles, max_range=8.0, num_rays=32):
        """Simulates 2D LiDAR ray sweeps detecting circular obstacle boundaries."""
        angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False)
        ranges = np.full(num_rays, max_range)

        for i, angle in enumerate(angles):
            ray_dir = np.array([np.cos(angle), np.sin(angle)])
            for ox, oy, radius in obstacles:
                to_obs = np.array([ox, oy]) - robot_pos
                proj = np.dot(to_obs, ray_dir)
                if proj > 0:
                    perp_sq = np.dot(to_obs, to_obs) - proj**2
                    if perp_sq < radius**2:
                        hit_dist = proj - np.sqrt(radius**2 - perp_sq)
                        if 0 < hit_dist < ranges[i]:
                            ranges[i] = hit_dist
        return angles, ranges
