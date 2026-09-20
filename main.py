import sys
import argparse
import numpy as np
from core.kinematics import DifferentialDriveKinematics
from planning.astar import AStarPlanner
from telemetry.motor_pwm import MotorControllerTelemetry
from visualizer import launch_animated_dashboard


def main():
    parser = argparse.ArgumentParser(description="Autonomous Mobile Robot (AMR) Navigation Engine")
    parser.add_argument("--start", type=float, nargs=2, default=[2.0, 2.0], help="Start coords: X Y")
    parser.add_argument("--goal", type=float, nargs=2, default=[24.0, 24.0], help="Goal coords: X Y")
    args = parser.parse_args()

    obstacles = [(8.0, 8.0, 2.5), (16.0, 10.0, 3.0), (10.0, 19.0, 2.2), (19.0, 18.0, 2.4)]

    print("=" * 60)
    print(" AUTONOMOUS MOBILE ROBOT (AMR) CONTROLLER - OPERATOR: MASUD")
    print("=" * 60)
    print(f"[*] Origin: {args.start} | Target: {args.goal}")

    planner = AStarPlanner(grid_size=28)
    try:
        path = planner.plan(args.start, args.goal, obstacles)
    except Exception as e:
        print(f"[!] Planning failed: {e}")
        sys.exit(1)

    print(f"[SUCCESS] Collision-free path computed: {len(path)} waypoints")

    kin = DifferentialDriveKinematics()
    telemetry = MotorControllerTelemetry()
    w_l, w_r = kin.inverse_kinematics(linear_vel=1.0, angular_vel=0.2)
    print(f"[*] Initial Telemetry Frame: {telemetry.serialize(w_l, w_r, packet_id=1)}")

    # Automatically launch Figure 1 animation dashboard
    launch_animated_dashboard(np.array(path), obstacles, args.start, args.goal)


if __name__ == "__main__":
    main()
