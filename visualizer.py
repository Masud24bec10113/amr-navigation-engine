import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec
from core.kinematics import DifferentialDriveKinematics


def launch_animated_dashboard(raw_path, obstacles, start, goal):
    interp_steps = 120
    t_raw = np.linspace(0, 1, len(raw_path))
    t_fine = np.linspace(0, 1, interp_steps)
    path = np.zeros((interp_steps, 2))
    path[:, 0] = np.interp(t_fine, t_raw, raw_path[:, 0])
    path[:, 1] = np.interp(t_fine, t_raw, raw_path[:, 1])

    kin = DifferentialDriveKinematics()
    w_left_arr, w_right_arr, heading_arr = [], [], []

    for k in range(len(path) - 1):
        disp = path[k+1] - path[k]
        h = np.arctan2(disp[1], disp[0])
        w_l, w_r = kin.inverse_kinematics(linear_vel=1.0, angular_vel=h * 0.15)
        w_left_arr.append(w_l)
        w_right_arr.append(w_r)
        heading_arr.append(np.degrees(h))
    w_left_arr.append(w_left_arr[-1])
    w_right_arr.append(w_right_arr[-1])
    heading_arr.append(heading_arr[-1])

    plt.style.use('dark_background')
    fig = plt.figure(figsize=(15, 8))
    gs = GridSpec(2, 2, figure=fig, width_ratios=[1.2, 1.0])

    ax_map = fig.add_subplot(gs[:, 0])
    for ox, oy, r in obstacles:
        ax_map.add_patch(plt.Circle((ox, oy), r, color='#e74c3c', alpha=0.85))
        ax_map.add_patch(plt.Circle((ox, oy), r + 1.2, color='#c0392b', fill=False, linestyle='--', alpha=0.4))

    ax_map.plot(path[:, 0], path[:, 1], color='#27ae60', linestyle=':', linewidth=2, label='A* Trajectory')
    ax_map.scatter(*start, color='#f1c40f', s=100, label='Start')
    ax_map.scatter(*goal, color='#3498db', s=120, marker='*', label='Goal')

    robot_marker, = ax_map.plot([], [], 'o', color='#00ffff', markersize=12, zorder=6, label='AMR Pose')
    lidar_lines = [ax_map.plot([], [], color='#00ffff', alpha=0.3, linewidth=0.8)[0] for _ in range(32)]

    ax_map.set_xlim(-1, 28)
    ax_map.set_ylim(-1, 28)
    ax_map.set_aspect('equal')
    ax_map.grid(True, linestyle=':', alpha=0.3)
    ax_map.set_title("LIVE AUTONOMOUS MISSION NAVIGATION (MASUD)", fontsize=10, fontweight='bold')
    ax_map.legend(loc='upper left', fontsize=8)

    ax_w = fig.add_subplot(gs[0, 1])
    line_wl, = ax_w.plot([], [], label='Left ($ω_L$)', color='#3498db', linewidth=2)
    line_wr, = ax_w.plot([], [], label='Right ($ω_R$)', color='#e67e22', linewidth=2)
    ax_w.set_xlim(0, interp_steps)
    ax_w.set_ylim(min(w_left_arr + w_right_arr) - 0.5, max(w_left_arr + w_right_arr) + 0.5)
    ax_w.set_ylabel("Speed (rad/s)", fontsize=8)
    ax_w.grid(True, linestyle=':', alpha=0.4)
    ax_w.legend(loc='upper right', fontsize=8)
    ax_w.set_title("MOTOR VELOCITY STREAM", fontsize=10, fontweight='bold')

    ax_h = fig.add_subplot(gs[1, 1])
    line_h, = ax_h.plot([], [], color='#2ecc71', linewidth=2)
    ax_h.set_xlim(0, interp_steps)
    ax_h.set_ylim(-10, 100)
    ax_h.set_xlabel("Time Frame Step", fontsize=8)
    ax_h.set_ylabel("Steering Heading (deg)", fontsize=8)
    ax_h.grid(True, linestyle=':', alpha=0.4)

    def update(frame):
        curr_x, curr_y = path[frame]
        robot_marker.set_data([curr_x], [curr_y])
        angles, ranges = kin.simulate_lidar_scan(np.array([curr_x, curr_y]), obstacles)
        for i, (ang, rng) in enumerate(zip(angles, ranges)):
            hit_x = curr_x + rng * np.cos(ang)
            hit_y = curr_y + rng * np.sin(ang)
            lidar_lines[i].set_data([curr_x, hit_x], [curr_y, hit_y])

        idx_range = np.arange(frame + 1)
        line_wl.set_data(idx_range, w_left_arr[:frame + 1])
        line_wr.set_data(idx_range, w_right_arr[:frame + 1])
        line_h.set_data(idx_range, heading_arr[:frame + 1])
        return [robot_marker, line_wl, line_wr, line_h] + lidar_lines

    anim = FuncAnimation(fig, update, frames=interp_steps, interval=40, blit=False, repeat=True)
    plt.tight_layout()
    plt.show()
