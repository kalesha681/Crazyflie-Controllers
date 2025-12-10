import numpy as np
import os
import pandas as pd
from typing import List, Dict

class Logger:
    """
    Standardized logger for Crazyflie simulation experiments.
    Enforces a strict CSV schema.
    """
    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.data_buffer: List[Dict] = []
        
    def log(self, t: float, 
            pos: np.ndarray, 
            vel: np.ndarray, 
            rpy: np.ndarray, 
            target_pos: np.ndarray, 
            control_rpm: np.ndarray):
        """
        Logs a single time step.
        """
        
        # Calculate Errors
        error = target_pos - pos
        
        entry = {
            'time': t,
            'x': pos[0], 'y': pos[1], 'z': pos[2],
            'vx': vel[0], 'vy': vel[1], 'vz': vel[2],
            'roll': rpy[0], 'pitch': rpy[1], 'yaw': rpy[2],
            'x_ref': target_pos[0], 'y_ref': target_pos[1], 'z_ref': target_pos[2],
            'error_x': error[0], 'error_y': error[1], 'error_z': error[2],
            'control_u1': control_rpm[0],
            'control_u2': control_rpm[1],
            'control_u3': control_rpm[2],
            'control_u4': control_rpm[3]
        }
        self.data_buffer.append(entry)
        
    def save_as_csv(self, filename: str):
        """
        Saves the logged data to a CSV file.
        """
        if not self.data_buffer:
            print("Warning: No data to save.")
            return

        df = pd.DataFrame(self.data_buffer)
        
        # Ensure column order
        cols = [
            'time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw',
            'x_ref', 'y_ref', 'z_ref',
            'error_x', 'error_y', 'error_z',
            'control_u1', 'control_u2', 'control_u3', 'control_u4'
        ]
        
        # Add missing columns as NaN if needed (though our log structure guarantees them)
        for c in cols:
            if c not in df.columns:
                df[c] = np.nan
        
        filepath = os.path.join(self.output_dir, filename)
        df.to_csv(filepath, index=False, columns=cols)
        print(f"[Logger] Saved data to {filepath}")
