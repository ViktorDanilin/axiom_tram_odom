import math

import pytest

from tram_model import tram_model as model


@pytest.mark.parametrize("notch, requested_torque", [
    (2.0, 1400.0), (-2.0, -1800.0), (-15.0, -5000.0)])
def test_drive_response_uses_input_calibration(notch, requested_torque):

    state = model.State(velocity=5.0, omega=5.0 / model.R, motor_torque=0.0)
    inputs = model.Inputs(
        notch=notch, traction_torque_per_notch_nm=700.0,
        brake_torque_per_notch_nm=900.0, max_brake_torque_nm=5000.0)
    dt = 0.0005
    for _ in range(1000):
        state, _ = model.step(state, inputs, dt)
    expected = requested_torque * (1 - math.exp(-model.MOTOR_TAU_INV * 0.5))
    assert state.motor_torque == pytest.approx(expected, rel=1e-8)
    assert (state.velocity - 5.0) * notch > 0


def test_total_drive_torque_accelerates_all_driven_wheels():
    state = model.State(velocity=5.0, omega=5.0 / model.R,
                        motor_torque=800.0)
    output = model.outputs(state, model.Inputs(notch=0.0))
    expected_inertia = (model.NUM_TRACTION_WHEELS * 0.5
                        * model.WHEEL_MASS * model.R**2)
    assert output["adhesion_force_n"] == pytest.approx(0.0, abs=1e-8)
    assert output["_wheel_acceleration"] == pytest.approx(800.0 / expected_inertia)
