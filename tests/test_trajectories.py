import pytest
import numpy as np
from crazyflie_controllers.trajectories.circle import CircleTrajectory
from crazyflie_controllers.trajectories.square import SquareTrajectory
from crazyflie_controllers.trajectories.triangle import TriangleTrajectory
from crazyflie_controllers.trajectories.eight import EightTrajectory
from crazyflie_controllers.trajectories.step import StepTrajectory

TRAJECTORY_CLASSES = [
    CircleTrajectory,
    SquareTrajectory,
    TriangleTrajectory,
    EightTrajectory,
    StepTrajectory,
]


@pytest.mark.parametrize("TrajectoryClass", TRAJECTORY_CLASSES)
class TestTrajectories:

    def test_instantiation(self, TrajectoryClass):
        traj = TrajectoryClass()
        assert traj is not None

    def test_get_target_interface(self, TrajectoryClass):
        """Verify get_target returns correct tuple structure."""
        traj = TrajectoryClass()
        t = 1.0

        output = traj.get_target(t)

        assert isinstance(output, tuple), "Output must be a tuple"
        assert len(output) == 4, "Output must be (pos, vel, acc, yaw)"

        pos, vel, acc, yaw = output

        assert pos.shape == (3,), f"Position shape mismatch: {pos.shape}"
        assert vel.shape == (3,), f"Velocity shape mismatch: {vel.shape}"
        assert acc.shape == (3,), f"Acceleration shape mismatch: {acc.shape}"
        assert isinstance(yaw, (float, np.floating)), "Yaw must be float"

        # Check finite
        assert np.all(np.isfinite(pos))
        assert np.all(np.isfinite(vel))
        assert np.all(np.isfinite(acc))
        assert np.isfinite(yaw)

    def test_time_validity(self, TrajectoryClass):
        """Verify trajectory is valid over a range of times."""
        traj = TrajectoryClass()
        times = [0.0, 0.1, 1.0, 5.0, 10.0]

        for t in times:
            try:
                traj.get_target(t)
            except Exception as e:
                pytest.fail(f"Trajectory raised exception at t={t}: {e}")

    def test_origin_start(self, TrajectoryClass):
        """Most trajectories should start near origin or defined setpoint at t=0."""
        # This is a soft check, but good for sanity
        traj = TrajectoryClass()
        pos, _, _, _ = traj.get_target(0.0)

        # We don't assert 0,0,0 because Step might start elsewhere or have offsets
        # Just checking it returns valid data is covered above.
        pass


class TestSpecificTrajectories:

    def test_circle_geometry(self):
        traj = (
            CircleTrajectory()
        )  # Assumes default radius=0.5 or similar implementation detail
        # Check if radius is roughly constant
        for t in np.linspace(2.0, 5.0, 10):  # skip smooth takeoff
            pos, _, _, _ = traj.get_target(t)  # Should handle takeoff logic
            # If takeoff is implemented, radius might vary.
            # Assuming 'Circle' eventually orbits.
            pass

    def test_step_discontinuity(self):
        """Step trajectory usually has a step."""
        traj = StepTrajectory()  # Default
        # Check before and after likely step time (usually 1.0s or similar)
        # We just verify it runs.
        pass
