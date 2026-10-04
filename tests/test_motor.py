"""
Unit tests for EV Motor Digital Twin physical model, thermal equations,
threshold alerts, and simulation engine.
"""

import math
import unittest
import numpy as np

from models.motor_model import (
    EVMotorTwin,
    MotorParameters,
    ThermalParameters,
    VehicleParameters,
)
from models.drive_cycles import (
    generate_accel_cruise_brake,
    generate_urban_stop_and_go,
    generate_highway_stress,
)
from simulation.simulator import MotorSimulator


class TestEVMotorTwin(unittest.TestCase):

    def test_torque_current_relationship(self):
        """Verify BLDC/DC fundamental equation: Torque = Kt * Current."""
        motor_p = MotorParameters(Kt=0.8, R0=0.05)
        twin = EVMotorTwin(motor_params=motor_p)

        I = 50.0
        expected_torque = motor_p.Kt * I
        self.assertAlmostEqual(expected_torque, 40.0, places=4)

        # Test step execution produces torque proportional to current
        res = twin.step(v_target=20.0, a_target=1.5, dt=0.1)
        self.assertGreater(res["torque_nm"], 0)
        self.assertAlmostEqual(res["torque_nm"], motor_p.Kt * res["current_a"], delta=0.01)

    def test_copper_resistance_temp_coefficient(self):
        """Verify resistance increases with temperature according to alpha_cu."""
        motor_p = MotorParameters(R0=0.05, alpha_cu=0.00393, T_ref=20.0)
        twin = EVMotorTwin(motor_params=motor_p)

        R_20 = twin.get_resistance(20.0)
        R_70 = twin.get_resistance(70.0)

        self.assertAlmostEqual(R_20, 0.05, places=5)
        expected_R_70 = 0.05 * (1.0 + 0.00393 * 50.0)
        self.assertAlmostEqual(R_70, expected_R_70, places=4)
        self.assertGreater(R_70, R_20)

    def test_joule_heating_temperature_rise(self):
        """Verify temperature rises over time driven by I²R losses."""
        thermal_p = ThermalParameters(T_ambient=25.0, C_th=1000.0, R_th_base=1.0)
        twin = EVMotorTwin(thermal_params=thermal_p, initial_temp=25.0, enable_thermal_derating=False)

        initial_temp = twin.T_motor
        self.assertEqual(initial_temp, 25.0)

        # Execute high acceleration for 5 seconds
        for _ in range(50):
            res = twin.step(v_target=30.0, a_target=2.0, dt=0.1)

        self.assertGreater(res["t_motor_c"], initial_temp)
        self.assertGreater(res["p_cu_kw"], 0.0)

    def test_cooling_to_ambient(self):
        """Verify motor cools back down toward ambient when stationary and idle."""
        thermal_p = ThermalParameters(T_ambient=25.0, C_th=1000.0, R_th_base=0.5)
        # Start at high temperature (90°C) with no load (v=0, a=0)
        twin = EVMotorTwin(thermal_params=thermal_p, initial_temp=90.0)

        for _ in range(100):  # 10 seconds of idle
            res = twin.step(v_target=0.0, a_target=0.0, dt=0.1)

        self.assertLess(res["t_motor_c"], 90.0)
        self.assertGreaterEqual(res["t_motor_c"], 25.0)
        self.assertGreater(res["p_cool_kw"], 0.0)

    def test_thermal_threshold_alerts(self):
        """Verify alert states: NORMAL -> WARNING -> CRITICAL as temperature crosses thresholds."""
        thermal_p = ThermalParameters(T_warning=80.0, T_critical=100.0)
        twin = EVMotorTwin(thermal_params=thermal_p)

        self.assertEqual(twin.evaluate_alert(25.0), "NORMAL")
        self.assertEqual(twin.evaluate_alert(79.9), "NORMAL")
        self.assertEqual(twin.evaluate_alert(80.0), "WARNING")
        self.assertEqual(twin.evaluate_alert(95.0), "WARNING")
        self.assertEqual(twin.evaluate_alert(100.0), "CRITICAL")
        self.assertEqual(twin.evaluate_alert(115.0), "CRITICAL")

    def test_thermal_derating_protection(self):
        """Verify derate factor throttles maximum allowable current at elevated temperature."""
        thermal_p = ThermalParameters(T_warning=80.0, T_critical=100.0)
        twin = EVMotorTwin(thermal_params=thermal_p, enable_thermal_derating=True)

        # Below warning: 100% factor
        self.assertEqual(twin.calculate_derate_factor(70.0), 1.0)

        # At midpoint between warning and critical: derated
        mid_factor = twin.calculate_derate_factor(90.0)
        self.assertTrue(0.15 < mid_factor < 1.0)

        # At or above critical: clamped to safety floor (0.15)
        self.assertEqual(twin.calculate_derate_factor(100.0), 0.15)
        self.assertEqual(twin.calculate_derate_factor(110.0), 0.15)

    def test_full_simulation_run(self):
        """Verify full drive cycle simulator produces complete telemetry dataset."""
        twin = EVMotorTwin()
        sim = MotorSimulator(twin)
        cycle_df = generate_accel_cruise_brake(total_time=15.0, dt=0.2)

        df_out, summary = sim.run_full(cycle_df)

        self.assertEqual(len(df_out), len(cycle_df))
        self.assertIn("t_motor_c", df_out.columns)
        self.assertIn("current_a", df_out.columns)
        self.assertIn("torque_nm", df_out.columns)
        self.assertIn("v_kmh", df_out.columns)
        self.assertGreaterEqual(summary.peak_temp_c, 25.0)
        self.assertGreater(summary.max_current_a, 0.0)


if __name__ == "__main__":
    unittest.main()
