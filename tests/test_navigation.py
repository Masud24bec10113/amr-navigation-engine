import pytest
import numpy as np
from core.kinematics import DifferentialDriveKinematics
from planning.astar import AStarPlanner
from telemetry.motor_pwm import MotorControllerTelemetry


def test_straight_line_kinematics():
    kin = DifferentialDriveKinematics(wheel_radius=0.05, track_width=0.30)
    w_l, w_r = kin.inverse_kinematics(linear_vel=1.0, angular_vel=0.0)
    assert w_l == w_r
    assert np.isclose(w_l, 20.0)


def test_astar_finds_path():
    planner = AStarPlanner(grid_size=20)
    path = planner.plan(start=(0, 0), goal=(10, 10), obstacles=[(5, 5, 2)])
    assert len(path) > 0
    assert np.allclose(path[0], [0, 0])
    assert np.allclose(path[-1], [10, 10])


def test_astar_collision_error():
    planner = AStarPlanner(grid_size=20)
    with pytest.raises(ValueError):
        planner.plan(start=(5, 5), goal=(10, 10), obstacles=[(5, 5, 2)])


def test_telemetry_serialization():
    telemetry = MotorControllerTelemetry()
    pkt = telemetry.serialize(w_left=10.0, w_right=-5.0, packet_id=1)
    assert "$AMR,ID=0001" in pkt
    assert "L_DIR=FWD" in pkt
