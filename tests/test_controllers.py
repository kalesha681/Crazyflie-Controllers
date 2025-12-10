import pytest
import numpy as np
from crazyflie_controllers.controllers.pid_controller import PIDController
from crazyflie_controllers.controllers.smc_controller import SMCController
from crazyflie_controllers.controllers.mpc_controller import MPCController

CONTROLLER_CLASSES = [PIDController, SMCController, MPCController]

@pytest.fixture
def sample_inputs():
    """
    Returns standard inputs for controller testing.
    """
    control_timestep = 0.01
    cur_pos = np.array([0., 0., 1.])
    cur_quat = np.array([0., 0., 0., 1.])
    cur_vel = np.array([0., 0., 0.])
    cur_ang_vel = np.array([0., 0., 0.])
    target_pos = np.array([0., 0., 1.5])
    target_vel = np.array([0.1, 0., 0.])
    target_acc = np.array([0., 0., 0.])
    target_yaw = 0.5
    return {
        'control_timestep': control_timestep,
        'cur_pos': cur_pos,
        'cur_quat': cur_quat,
        'cur_vel': cur_vel,
        'cur_ang_vel': cur_ang_vel,
        'target_pos': target_pos,
        'target_vel': target_vel,
        'target_acc': target_acc,
        'target_yaw': target_yaw
    }

@pytest.mark.parametrize("ControllerClass", CONTROLLER_CLASSES)
class TestControllers:

    def test_instantiation_and_reset(self, ControllerClass):
        """Verify controller can be instantiated and reset without error."""
        ctrl = ControllerClass()
        ctrl.reset()
        assert True

    def test_compute_control_interface(self, ControllerClass, sample_inputs):
        """Verify compute_control returns correct shape and type."""
        ctrl = ControllerClass()
        ctrl.reset()
        
        output = ctrl.compute_control(**sample_inputs)
        
        assert isinstance(output, np.ndarray), "Output must be a numpy array"
        assert output.shape == (4,), f"Output shape mismatch: expected (4,), got {output.shape}"
        assert np.issubdtype(output.dtype, np.number), "Output must be numeric"
        assert np.all(np.isfinite(output)), "Output contains NaNs or Infs"

    def test_determinism(self, ControllerClass, sample_inputs):
        """Verify identical inputs produce identical outputs."""
        ctrl = ControllerClass()
        
        # Run 1
        ctrl.reset()
        out1 = ctrl.compute_control(**sample_inputs)
        
        # Run 2 (Reset to ensure internal state is cleared if relevant)
        ctrl.reset()
        out2 = ctrl.compute_control(**sample_inputs)
        
        np.testing.assert_array_equal(out1, out2, err_msg="Controller is non-deterministic")

    def test_invalid_dt(self, ControllerClass, sample_inputs):
        """Verify ValueError on non-positive dt."""
        ctrl = ControllerClass()
        inputs = sample_inputs.copy()
        
        for invalid_dt in [0, -0.01]:
            inputs['control_timestep'] = invalid_dt
            with pytest.raises(ValueError, match="positive"):
                ctrl.compute_control(**inputs)

    def test_invalid_inputs(self, ControllerClass, sample_inputs):
        """Verify ValueError on NaN/Inf inputs."""
        ctrl = ControllerClass()
        
        keys_to_test = ['cur_pos', 'cur_vel', 'target_pos']
        
        for key in keys_to_test:
            inputs = sample_inputs.copy()
            # Inject NaN
            bad_val = inputs[key].copy()
            bad_val[0] = np.nan
            inputs[key] = bad_val
            
            with pytest.raises(ValueError, match="NaN or Inf"):
                ctrl.compute_control(**inputs)
                
            # Inject Inf
            inputs = sample_inputs.copy()
            bad_val = inputs[key].copy()
            bad_val[0] = np.inf
            inputs[key] = bad_val
            
            with pytest.raises(ValueError, match="NaN or Inf"):
                ctrl.compute_control(**inputs)

    def test_stability_bounded_output(self, ControllerClass, sample_inputs):
        """Verify control outputs are within physical sanity limits for small errors."""
        ctrl = ControllerClass()
        
        # Max RPM for Crazyflie is around 22k-25k approx, but let's just check it doesn't explode to 1e9
        HARD_LIMIT = 1e6 
        
        output = ctrl.compute_control(**sample_inputs)
        assert np.all(np.abs(output) < HARD_LIMIT), f"Control output exceeded hard limit {HARD_LIMIT}"
