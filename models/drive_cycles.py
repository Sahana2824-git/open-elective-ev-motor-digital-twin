"""
Drive Cycle Profiles for EV Digital Twin Simulation.
Includes synthetic acceleration/cruise/braking, urban stop-and-go,
high-load highway endurance, and customizable speed profiles.
"""

from typing import Dict, List, Tuple
import numpy as np
import pandas as pd


def generate_accel_cruise_brake(
    total_time: float = 120.0,
    dt: float = 0.1,
    accel_time_1: float = 10.0,
    cruise_speed_1: float = 60.0,  # km/h
    cruise_time_1: float = 25.0,
    accel_time_2: float = 8.0,
    cruise_speed_2: float = 110.0, # km/h
    cruise_time_2: float = 30.0,
    brake_time: float = 15.0,
    idle_time: float = 10.0,
) -> pd.DataFrame:
    """
    Constructs a multi-stage synthetic drive cycle:
    Rest -> Accel 1 -> Cruise 1 -> Accel 2 (Highway) -> Cruise 2 -> Regen Brake -> Idle.
    """
    times = np.arange(0.0, total_time + dt, dt)
    speeds_kmh = np.zeros_like(times)

    # Segment boundaries
    t0 = 2.0  # Initial rest
    t1 = t0 + accel_time_1
    t2 = t1 + cruise_time_1
    t3 = t2 + accel_time_2
    t4 = t3 + cruise_time_2
    t5 = t4 + brake_time
    t6 = t5 + idle_time

    for i, t in enumerate(times):
        if t <= t0:
            speeds_kmh[i] = 0.0
        elif t <= t1:
            # Linear acceleration to cruise_speed_1
            alpha = (t - t0) / accel_time_1
            speeds_kmh[i] = cruise_speed_1 * alpha
        elif t <= t2:
            # Steady cruise 1
            speeds_kmh[i] = cruise_speed_1
        elif t <= t3:
            # Acceleration from cruise_speed_1 to cruise_speed_2
            alpha = (t - t2) / accel_time_2
            speeds_kmh[i] = cruise_speed_1 + (cruise_speed_2 - cruise_speed_1) * alpha
        elif t <= t4:
            # Steady high-speed cruise 2
            speeds_kmh[i] = cruise_speed_2
        elif t <= t5:
            # Regenerative braking down to 0
            alpha = (t - t4) / brake_time
            speeds_kmh[i] = max(0.0, cruise_speed_2 * (1.0 - alpha))
        else:
            speeds_kmh[i] = 0.0

    # Convert km/h to m/s
    speeds_ms = speeds_kmh / 3.6
    # Compute acceleration dv/dt using central differences with boundary edge handling
    accel_ms2 = np.gradient(speeds_ms, dt)

    return pd.DataFrame({
        "time": np.round(times, 2),
        "speed_kmh": np.round(speeds_kmh, 2),
        "speed_ms": np.round(speeds_ms, 3),
        "accel_ms2": np.round(accel_ms2, 3),
        "grade_pct": np.zeros_like(times),
    })


def generate_urban_stop_and_go(total_time: float = 100.0, dt: float = 0.1) -> pd.DataFrame:
    """
    Simulates high-demand urban city driving with aggressive accelerations,
    frequent stoplights, and heavy regenerative braking.
    Generates high current peaks ($I^2 R$ heat).
    """
    times = np.arange(0.0, total_time + dt, dt)
    speeds_kmh = np.zeros_like(times)

    # 4 distinct pulses of stop & go
    pattern_steps = [
        # (duration, peak_kmh, accel_ratio, brake_ratio)
        (20.0, 45.0, 0.35, 0.35),
        (25.0, 55.0, 0.25, 0.35),
        (30.0, 65.0, 0.30, 0.30),
        (25.0, 40.0, 0.40, 0.35),
    ]

    t_cursor = 0.0
    for dur, peak_v, a_rat, b_rat in pattern_steps:
        t_end = t_cursor + dur
        mask = (times >= t_cursor) & (times < t_end)
        t_local = times[mask] - t_cursor
        t_acc = dur * a_rat
        t_brk = dur * (1.0 - b_rat)

        v_seg = np.zeros_like(t_local)
        for j, tl in enumerate(t_local):
            if tl < t_acc:
                v_seg[j] = peak_v * (tl / t_acc)
            elif tl < t_brk:
                # Cruise with small variation
                v_seg[j] = peak_v + 3.0 * np.sin(2 * np.pi * (tl - t_acc) / (t_brk - t_acc))
            else:
                alpha = (tl - t_brk) / (dur - t_brk)
                v_seg[j] = max(0.0, peak_v * (1.0 - alpha))

        speeds_kmh[mask] = v_seg
        t_cursor = t_end

    speeds_ms = speeds_kmh / 3.6
    accel_ms2 = np.gradient(speeds_ms, dt)

    return pd.DataFrame({
        "time": np.round(times, 2),
        "speed_kmh": np.round(speeds_kmh, 2),
        "speed_ms": np.round(speeds_ms, 3),
        "accel_ms2": np.round(accel_ms2, 3),
        "grade_pct": np.zeros_like(times),
    })


def generate_highway_stress(total_time: float = 150.0, dt: float = 0.1) -> pd.DataFrame:
    """
    Simulates aggressive high-speed highway cruising with steep incline grade.
    Designed specifically to push motor thermal limit past WARNING and CRITICAL thresholds.
    """
    times = np.arange(0.0, total_time + dt, dt)
    speeds_kmh = np.zeros_like(times)
    grades_pct = np.zeros_like(times)

    for i, t in enumerate(times):
        if t < 5.0:
            speeds_kmh[i] = (t / 5.0) * 30.0
        elif t < 18.0:
            alpha = (t - 5.0) / 13.0
            speeds_kmh[i] = 30.0 + alpha * 90.0  # Reach 120 km/h
        elif t < 90.0:
            # Sustained high speed cruise with uphill climb (4.5% grade)
            speeds_kmh[i] = 125.0 + 3.0 * np.sin(0.1 * t)
            if t > 30.0 and t < 85.0:
                grades_pct[i] = 4.5  # 4.5% uphill grade adds major tractive load
        elif t < 120.0:
            # Full throttle push to 140 km/h
            alpha = (t - 90.0) / 30.0
            speeds_kmh[i] = 125.0 + alpha * 20.0
            grades_pct[i] = 3.0
        else:
            # Decel to zero
            alpha = (t - 120.0) / 30.0
            speeds_kmh[i] = max(0.0, 145.0 * (1.0 - alpha))

    speeds_ms = speeds_kmh / 3.6
    accel_ms2 = np.gradient(speeds_ms, dt)

    return pd.DataFrame({
        "time": np.round(times, 2),
        "speed_kmh": np.round(speeds_kmh, 2),
        "speed_ms": np.round(speeds_ms, 3),
        "accel_ms2": np.round(accel_ms2, 3),
        "grade_pct": np.round(grades_pct, 1),
    })


def generate_wltp_simplified(dt: float = 0.1) -> pd.DataFrame:
    """
    Synthesizes a representative 180-second multi-phase WLTP style cycle:
    Phase 1: Low speed urban (0-45 km/h)
    Phase 2: Medium speed suburban (0-75 km/h)
    Phase 3: High speed rural (0-95 km/h)
    Phase 4: Extra-high speed motorway (0-130 km/h)
    """
    total_time = 180.0
    times = np.arange(0.0, total_time + dt, dt)
    speeds_kmh = np.zeros_like(times)

    waypoints = [
        # (time_sec, speed_kmh)
        (0.0, 0.0),
        (10.0, 20.0),
        (20.0, 35.0),
        (30.0, 0.0),
        (40.0, 0.0),
        (55.0, 50.0),
        (70.0, 70.0),
        (85.0, 25.0),
        (95.0, 0.0),
        (110.0, 65.0),
        (130.0, 95.0),
        (145.0, 50.0),
        (160.0, 130.0),
        (172.0, 80.0),
        (180.0, 0.0),
    ]

    wp_t = [w[0] for w in waypoints]
    wp_v = [w[1] for w in waypoints]

    speeds_kmh = np.interp(times, wp_t, wp_v)
    speeds_ms = speeds_kmh / 3.6
    accel_ms2 = np.gradient(speeds_ms, dt)

    return pd.DataFrame({
        "time": np.round(times, 2),
        "speed_kmh": np.round(speeds_kmh, 2),
        "speed_ms": np.round(speeds_ms, 3),
        "accel_ms2": np.round(accel_ms2, 3),
        "grade_pct": np.zeros_like(times),
    })


AVAILABLE_CYCLES = {
    "Synthetic (Accel / Cruise / Brake)": generate_accel_cruise_brake,
    "Urban Stop-and-Go": generate_urban_stop_and_go,
    "Highway Thermal Stress Test (Overheat Scenario)": generate_highway_stress,
    "WLTP Simplified (Class 3 Representative)": generate_wltp_simplified,
}


def list_available_drive_cycles() -> List[str]:
    return list(AVAILABLE_CYCLES.keys())


def get_drive_cycle(name: str, **kwargs) -> pd.DataFrame:
    """Retrieve drive cycle DataFrame by name."""
    if name not in AVAILABLE_CYCLES:
        name = "Synthetic (Accel / Cruise / Brake)"
    generator = AVAILABLE_CYCLES[name]
    return generator(**kwargs)
