import numpy as np
import os
import pandas as pd
from typing import List, Dict, Optional
import time
from dataclasses import dataclass

@dataclass
class LogEntry:
    time: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    roll: float
    pitch: float
    yaw: float
    x_ref: float
    y_ref: float
    z_ref: float
    ex: float
    ey: float
    ez: float
    u1: float
    u2: float
    u3: float
    u4: float

class Logger:
    """
    Standardized logger for Crazyflie simulation experiments.
    Enforces a strict CSV schema, metadata injection, and validation.
    """
    
    # Strict Column Order
    COLUMNS = [
        'time', 
        'x', 'y', 'z', 
        'vx', 'vy', 'vz', 
        'roll', 'pitch', 'yaw',
        'x_ref', 'y_ref', 'z_ref',
        'ex', 'ey', 'ez',
        'u1', 'u2', 'u3', 'u4'
    ]

    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.data_buffer: List[Dict[str, float]] = []
        
    def log(self, t: float, 
            pos: np.ndarray, 
            vel: np.ndarray, 
            rpy: np.ndarray, 
            target_pos: np.ndarray, 
            control_rpm: np.ndarray) -> None:
        """
        Logs a single time step with strict shape and NaN checking.
        """
        # 1. Runtime Shape Assertions
        assert pos.shape == (3,), f"pos shape mismatch: {pos.shape}"
        assert vel.shape == (3,), f"vel shape mismatch: {vel.shape}"
        assert rpy.shape == (3,), f"rpy shape mismatch: {rpy.shape}"
        assert target_pos.shape == (3,), f"target_pos shape mismatch: {target_pos.shape}"
        assert control_rpm.shape == (4,), f"control_rpm shape mismatch: {control_rpm.shape}"
        
        # 2. NaN Checks (Fail Fast)
        if not np.all(np.isfinite(pos)): raise ValueError("pos contains NaN/Inf")
        if not np.all(np.isfinite(vel)): raise ValueError("vel contains NaN/Inf")
        if not np.all(np.isfinite(rpy)): raise ValueError("rpy contains NaN/Inf")
        if not np.all(np.isfinite(target_pos)): raise ValueError("target_pos contains NaN/Inf")
        if not np.all(np.isfinite(control_rpm)): raise ValueError("control_rpm contains NaN/Inf")

        # Calculate Errors
        error = target_pos - pos
        
        # 3. Strict Dictionary Creation
        entry = {
            'time': float(t),
            'x': float(pos[0]), 'y': float(pos[1]), 'z': float(pos[2]),
            'vx': float(vel[0]), 'vy': float(vel[1]), 'vz': float(vel[2]),
            'roll': float(rpy[0]), 'pitch': float(rpy[1]), 'yaw': float(rpy[2]),
            'x_ref': float(target_pos[0]), 'y_ref': float(target_pos[1]), 'z_ref': float(target_pos[2]),
            'ex': float(error[0]), 'ey': float(error[1]), 'ez': float(error[2]),
            'u1': float(control_rpm[0]),
            'u2': float(control_rpm[1]),
            'u3': float(control_rpm[2]),
            'u4': float(control_rpm[3])
        }
        
        # 4. Fail Fast on Missing Keys (Defense in depth)
        if len(entry) != len(self.COLUMNS):
             raise ValueError(f"Log entry length mismatch. Expected {len(self.COLUMNS)}, got {len(entry)}")

        self.data_buffer.append(entry)
        
    def save_as_csv(self, filename: str, metadata: Optional[Dict[str, str]] = None) -> None:
        """
        Saves the logged data to a CSV file with metadata header.
        Enforces strict formatting and schema.
        """
        if not self.data_buffer:
            print("[Logger] Warning: No data to save.")
            return

        df = pd.DataFrame(self.data_buffer)
        
        # 5. Schema Validation
        if not all(col in df.columns for col in self.COLUMNS):
            missing = set(self.COLUMNS) - set(df.columns)
            raise ValueError(f"Missing columns in dataframe: {missing}")
            
        # 6. Enforce Column Order
        df = df[self.COLUMNS]
        
        # 7. Explicit NaN Filling (Should not happen due to strict log, but safe)
        if df.isnull().values.any():
            raise ValueError("NaN values detected in log buffer. Logs must be complete.")

        filepath = os.path.join(self.output_dir, filename)
        
        # 8. Write with Metadata and Deterministic Formatting
        with open(filepath, 'w') as f:
            if metadata:
                for key, value in metadata.items():
                    f.write(f"# {key}: {value}\n")
            f.write(f"# timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            
            # Use float_format to ensure deterministic output (e.g. %.6f)
            df.to_csv(f, index=False, float_format='%.6f')
            
        print(f"[Logger] Saved data to {filepath}")
