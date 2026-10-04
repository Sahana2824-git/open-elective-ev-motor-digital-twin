"""
Simulation engine for the EV Motor Digital Twin.
Executes drive cycles, computes electrical/mechanical/thermal metrics,
and aggregates performance & thermal safety statistics.
"""

from dataclasses import dataclass
from typing import Dict, Any, Generator, Tuple
import numpy as np
import pandas as pd

from models.motor_model import EVMotorTwin


@dataclass
class SimulationSummary:
    """Aggregated key performance indicators of a simulation run."""
    peak_temp_c: float
    final_temp_c: float
    time_in_warning_s: float
    time_in_critical_s: float
    max_current_a: float
    max_torque_nm: float
    max_power_kw: float
    total_energy_consumed_kwh: float
    total_energy_regen_kwh: float
    net_energy_kwh: float
    total_copper_loss_kwh: float
    avg_efficiency_pct: float
    thermal_event_triggered: bool


class MotorSimulator:
    """
    Coordinates execution of drive cycles through the EV Motor Digital Twin.
    """

    def __init__(self, twin: EVMotorTwin):
        self.twin = twin

    def run_full(self, drive_cycle: pd.DataFrame, initial_temp: float = None) -> Tuple[pd.DataFrame, SimulationSummary]:
        """
        Runs the full drive cycle deterministically from start to end.
        Returns the detailed telemetry DataFrame and high-level summary KPIs.
        """
        self.twin.reset_state(initial_temp)

        records = []
        times = drive_cycle["time"].values
        speeds = drive_cycle["speed_ms"].values
        accels = drive_cycle["accel_ms2"].values
        grades = drive_cycle["grade_pct"].values if "grade_pct" in drive_cycle.columns else np.zeros_like(times)

        n_steps = len(times)
        dt = (times[1] - times[0]) if n_steps > 1 else 0.1

        for i in range(n_steps):
            t = times[i]
            v = speeds[i]
            a = accels[i]
            # Convert grade % to radians: grade_pct / 100 = tan(theta) approx theta
            grade_rad = np.arctan(grades[i] / 100.0)

            telemetry = self.twin.step(v_target=v, a_target=a, dt=dt, grade_rad=grade_rad)
            telemetry["time"] = float(t)
            records.append(telemetry)

        df_telemetry = pd.DataFrame(records)
        summary = self._compute_summary(df_telemetry, dt)
        return df_telemetry, summary

    def stream_step(
        self, drive_cycle: pd.DataFrame, initial_temp: float = None
    ) -> Generator[Dict[str, Any], None, None]:
        """
        Yields telemetry step-by-step for real-time live streaming visualization.
        """
        self.twin.reset_state(initial_temp)
        times = drive_cycle["time"].values
        speeds = drive_cycle["speed_ms"].values
        accels = drive_cycle["accel_ms2"].values
        grades = drive_cycle["grade_pct"].values if "grade_pct" in drive_cycle.columns else np.zeros_like(times)

        n_steps = len(times)
        dt = (times[1] - times[0]) if n_steps > 1 else 0.1

        for i in range(n_steps):
            t = times[i]
            v = speeds[i]
            a = accels[i]
            grade_rad = np.arctan(grades[i] / 100.0)

            telemetry = self.twin.step(v_target=v, a_target=a, dt=dt, grade_rad=grade_rad)
            telemetry["time"] = float(t)
            telemetry["step_index"] = i
            telemetry["total_steps"] = n_steps
            yield telemetry

    def _compute_summary(self, df: pd.DataFrame, dt: float) -> SimulationSummary:
        """Calculates energy totals, peak temperatures, and threshold violations."""
        peak_temp = float(df["t_motor_c"].max())
        final_temp = float(df["t_motor_c"].iloc[-1])

        # Thermal duration counters
        warning_steps = (df["alert_level"] == "WARNING").sum()
        critical_steps = (df["alert_level"] == "CRITICAL").sum()
        time_warning = float(warning_steps * dt)
        time_critical = float(critical_steps * dt)

        # Electrical energy integration: E = sum(V * I * dt) / 3600 / 1000 [kWh]
        P_elec_kw = (df["v_terminal"] * df["current_a"]) / 1000.0
        # Positive power: Motoring consumed
        motoring_kw = np.maximum(0.0, P_elec_kw)
        # Negative power: Regenerative recovery
        regen_kw = np.maximum(0.0, -P_elec_kw)

        energy_consumed = float(np.sum(motoring_kw) * dt / 3600.0)
        energy_regen = float(np.sum(regen_kw) * dt / 3600.0)
        net_energy = energy_consumed - energy_regen

        # Copper heating loss energy
        cu_loss_energy = float(np.sum(df["p_cu_kw"]) * dt / 3600.0)

        # Mechanical peaks
        max_cur = float(df["current_a"].abs().max())
        max_trq = float(df["torque_nm"].abs().max())
        max_pwr = float(df["p_mech_kw"].abs().max())

        # Average efficiency during active motoring
        motoring_mask = df["p_mech_kw"] > 0.5
        if motoring_mask.any():
            avg_eff = float(df.loc[motoring_mask, "efficiency"].mean() * 100.0)
        else:
            avg_eff = 0.0

        return SimulationSummary(
            peak_temp_c=peak_temp,
            final_temp_c=final_temp,
            time_in_warning_s=time_warning,
            time_in_critical_s=time_critical,
            max_current_a=max_cur,
            max_torque_nm=max_trq,
            max_power_kw=max_pwr,
            total_energy_consumed_kwh=energy_consumed,
            total_energy_regen_kwh=energy_regen,
            net_energy_kwh=net_energy,
            total_copper_loss_kwh=cu_loss_energy,
            avg_efficiency_pct=avg_eff,
            thermal_event_triggered=(time_warning > 0 or time_critical > 0),
        )
