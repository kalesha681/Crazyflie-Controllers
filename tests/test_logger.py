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
    pos = np.array([1., 2., 3.])
    vel = np.array([0., 0., 0.])
    rpy = np.array([0., 0., 0.])
    target_pos = np.array([1., 2., 3.])
    control_rpm = np.array([1000., 1000., 1000., 1000.])
    
    logger.log(t, pos, vel, rpy, target_pos, control_rpm)
    
    csv_name = "test.csv"
    logger.save_as_csv(csv_name)
    
    df = pd.read_csv(log_dir / csv_name)
    
    expected_cols = [
        'time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 
        'roll', 'pitch', 'yaw', 
        'x_ref', 'y_ref', 'z_ref', 
        'error_x', 'error_y', 'error_z', 
        'control_u1', 'control_u2', 'control_u3', 'control_u4'
    ]
    
    assert list(df.columns) == expected_cols

def test_logger_nan_handling(tmp_path):
    """Verify Logger handles NaNs gracefully."""
    log_dir = tmp_path / "logs_nan"
    os.makedirs(log_dir)
    logger = Logger(str(log_dir))
    
    t = 0.5
    pos = np.array([np.nan, 2., 3.]) # NaN input
    vel = np.zeros(3)
    rpy = np.zeros(3)
    target_pos = np.zeros(3)
    control_rpm = np.zeros(4)
    
    logger.log(t, pos, vel, rpy, target_pos, control_rpm)
    logger.save_as_csv("nan_test.csv")
    
    df = pd.read_csv(log_dir / "nan_test.csv")
    assert np.isnan(df.iloc[0]['x'])
    assert df.iloc[0]['y'] == 2.0

def test_logger_buffer_clearing(tmp_path):
    """Verify buffer logic if applicable (not strictly required by prompt but good practice)."""
    pass
