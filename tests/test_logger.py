import pytest
import numpy as np
import pandas as pd
import os
from crazyflie_controllers.utils.logging import Logger


def test_logger_schema(tmp_path):
    """Verify CSV columns and order."""
    log_dir = tmp_path / "logs"
    os.makedirs(log_dir)
    logger = Logger(str(log_dir))

    # Log 1 step
    t = 0.1
    pos = np.array([1.0, 2.0, 3.0])
    vel = np.array([0.0, 0.0, 0.0])
    rpy = np.array([0.0, 0.0, 0.0])
    target_pos = np.array([1.0, 2.0, 3.0])
    control_rpm = np.array([1000.0, 1000.0, 1000.0, 1000.0])

    logger.log(t, pos, vel, rpy, target_pos, control_rpm)

    csv_name = "test.csv"
    logger.save_as_csv(csv_name)

    # Read with comment skipping
    df = pd.read_csv(log_dir / csv_name, comment="#")

    expected_cols = [
        "time",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "roll",
        "pitch",
        "yaw",
        "x_ref",
        "y_ref",
        "z_ref",
        "ex",
        "ey",
        "ez",
        "u1",
        "u2",
        "u3",
        "u4",
    ]

    assert list(df.columns) == expected_cols


def test_logger_input_validation(tmp_path):
    """Verify Logger raises errors on invalid inputs (NaNs/shapes)."""
    log_dir = tmp_path / "logs_nan"
    os.makedirs(log_dir)
    logger = Logger(str(log_dir))

    t = 0.5
    pos = np.zeros(3)
    vel = np.zeros(3)
    rpy = np.zeros(3)
    target_pos = np.zeros(3)
    control_rpm = np.zeros(4)

    # 1. Test NaN
    bad_pos = pos.copy()
    bad_pos[0] = np.nan

    with pytest.raises(ValueError, match="NaN"):
        logger.log(t, bad_pos, vel, rpy, target_pos, control_rpm)

    # 2. Test Shape
    bad_rpm = np.zeros(3)  # Wrong shape
    with pytest.raises(AssertionError):
        logger.log(t, pos, vel, rpy, target_pos, bad_rpm)


def test_logger_buffer_clearing(tmp_path):
    """Verify buffer logic if applicable."""
    pass
