"""
EV Motor Digital Twin - Physical & Thermal Models
Implements simplified BLDC / Permanent Magnet DC motor dynamics,
tractive resistance, Joule heating (I²R), and thermal dissipation.
"""

from dataclasses import dataclass
from typing import Dict, Any, Tuple
import numpy as np


@dataclass
class MotorParameters:
    """Electrical and electromechanical specifications."""
    Kt: float = 0.65            # Torque constant [N·m / A]
    Ke: float = 0.65            # Back-EMF constant [V / (rad/s)]
    R0: float = 0.045           # Armature resistance at 20°C [Ohms]
    alpha_cu: float = 0.00393   # Copper temperature coefficient [1/°C]
    T_ref: float = 20.0         # Reference temperature [°C]
    I_max: float = 250.0        # Peak allowable current [A]
    I_rated: float = 140.0      # Continuous rated current [A]
    max_rpm: float = 9000.0     # Maximum motor mechanical speed [RPM]
    V_dc: float = 400.0         # DC bus / Battery pack voltage [V]
    core_loss_coeff: float = 0.0008  # Core/iron loss coefficient


@dataclass
class ThermalParameters:
    """Thermal lumped-capacitance parameters."""
    C_th: float = 2800.0        # Thermal capacitance [J / °C] (W·s/°C)
    R_th_base: float = 0.42     # Base thermal resistance to ambient [°C / W]
    T_ambient: float = 25.0     # Ambient temperature [°C]
    T_warning: float = 85.0     # Warning temperature threshold [°C]
    T_critical: float = 110.0   # Critical shutdown/derate threshold [°C]
    cooling_factor: float = 0.035  # Speed-dependent forced cooling enhancement [s/m]


@dataclass
class VehicleParameters:
    """Vehicle physical properties for tractive load calculation."""
    mass: float = 1650.0        # Total vehicle curb + payload mass [kg]
    Cd: float = 0.26            # Aerodynamic drag coefficient
    frontal_area: float = 2.25  # Frontal area [m²]
    crr: float = 0.012          # Rolling resistance coefficient
    wheel_radius: float = 0.315 # Tire rolling radius [m]
    gear_ratio: float = 8.5     # Single-speed reduction transmission ratio
    gear_efficiency: float = 0.96 # Transmission efficiency
    regen_factor: float = 0.75  # Regen braking kinetic capture factor
    air_density: float = 1.225  # Air density at sea level [kg/m³]
    gravity: float = 9.81       # Gravitational acceleration [m/s²]


class EVMotorTwin:
    """
    Digital Twin of an EV Motor.
    Tracks live state including:
    - Speed, acceleration, required wheel and shaft torque
    - Armature current (Torque = Kt * Current)
    - Terminal and Back-EMF voltage
    - Joule loss (I²R) with temperature-dependent copper resistance
    - Motor winding temperature evolution via heat transfer ODE
    - Real-time thermal protection alerts (NORMAL, WARNING, CRITICAL)
    """

    def __init__(
        self,
        motor_params: MotorParameters = None,
        thermal_params: ThermalParameters = None,
        vehicle_params: VehicleParameters = None,
        initial_temp: float = None,
        enable_thermal_derating: bool = True,
    ):
        self.motor = motor_params or MotorParameters()
        self.thermal = thermal_params or ThermalParameters()
        self.vehicle = vehicle_params or VehicleParameters()
        self.enable_thermal_derating = enable_thermal_derating

        # State variables
        self.T_motor = initial_temp if initial_temp is not None else self.thermal.T_ambient
        self.reset_state(self.T_motor)

    def reset_state(self, initial_temp: float = None):
        """Reset the digital twin internal state."""
        self.T_motor = initial_temp if initial_temp is not None else self.thermal.T_ambient
        self.current = 0.0
        self.torque = 0.0
        self.rpm = 0.0
        self.v = 0.0
        self.a = 0.0
        self.P_cu = 0.0
        self.P_core = 0.0
        self.P_heat = 0.0
        self.P_cool = 0.0
        self.P_mech = 0.0
        self.V_terminal = 0.0
        self.V_bemf = 0.0
        self.derate_factor = 1.0
        self.alert_level = "NORMAL"

    def get_resistance(self, T: float) -> float:
        """Calculate temperature-dependent copper resistance: R(T) = R0 * (1 + alpha * (T - T_ref))."""
        return max(0.001, self.motor.R0 * (1.0 + self.motor.alpha_cu * (T - self.motor.T_ref)))

    def calculate_tractive_force(self, v: float, a: float, grade_rad: float = 0.0) -> float:
        """
        Calculate total road load force required at the vehicle wheels:
        F_tractive = F_rolling_resistance + F_aero_drag + F_acceleration + F_grade
        """
        v_abs = abs(v)

        # Rolling resistance (opposes motion)
        if v_abs > 0.05:
            F_rr = np.sign(v) * self.vehicle.crr * self.vehicle.mass * self.vehicle.gravity * np.cos(grade_rad)
        else:
            F_rr = 0.0

        # Aerodynamic drag: 0.5 * rho * Cd * A * v²
        F_aero = 0.5 * self.vehicle.air_density * self.vehicle.Cd * self.vehicle.frontal_area * (v ** 2) * np.sign(v)

        # Inertial acceleration force: m * a
        F_acc = self.vehicle.mass * a

        # Gravitational grade resistance: m * g * sin(grade)
        F_grade = self.vehicle.mass * self.vehicle.gravity * np.sin(grade_rad)

        return F_rr + F_aero + F_acc + F_grade

    def calculate_derate_factor(self, T: float) -> float:
        """
        Calculates thermal derating factor (1.0 = 100% capacity, 0.0 = completely throttled)
        Protects the motor from catastrophic winding burnout when T exceeds T_warning.
        """
        if not self.enable_thermal_derating:
            return 1.0

        if T < self.thermal.T_warning:
            return 1.0
        elif T >= self.thermal.T_critical:
            return 0.15  # 15% crawl limp-home mode
        else:
            # Linear ramp-down from warning to critical
            span = self.thermal.T_critical - self.thermal.T_warning
            delta = T - self.thermal.T_warning
            return max(0.15, 1.0 - 0.85 * (delta / span))

    def evaluate_alert(self, T: float) -> str:
        """Determine alert status based on motor temperature thresholds."""
        if T >= self.thermal.T_critical:
            return "CRITICAL"
        elif T >= self.thermal.T_warning:
            return "WARNING"
        return "NORMAL"

    def step(self, v_target: float, a_target: float, dt: float = 0.1, grade_rad: float = 0.0) -> Dict[str, Any]:
        """
        Advance the digital twin simulation by dt seconds.
        
        Args:
            v_target: Target vehicle speed [m/s]
            a_target: Target vehicle acceleration [m/s²]
            dt: Timestep [s]
            grade_rad: Road gradient angle [radians]
            
        Returns:
            Dictionary containing instantaneous motor telemetry.
        """
        self.v = max(0.0, v_target)
        self.a = a_target

        # 1. Vehicle road load and wheel mechanics
        F_tract = self.calculate_tractive_force(self.v, self.a, grade_rad)
        wheel_torque = F_tract * self.vehicle.wheel_radius
        wheel_omega = self.v / self.vehicle.wheel_radius  # rad/s

        # 2. Transmission to motor shaft
        motor_omega = wheel_omega * self.vehicle.gear_ratio  # rad/s
        motor_rpm = min(self.motor.max_rpm, (motor_omega * 60.0) / (2.0 * np.pi))
        # Re-derive motor_omega from capped RPM
        motor_omega = (motor_rpm * 2.0 * np.pi) / 60.0

        # Required motor shaft torque
        if wheel_torque >= 0:
            # Motoring
            tau_required = wheel_torque / (self.vehicle.gear_ratio * self.vehicle.gear_efficiency)
        else:
            # Regenerative braking
            tau_required = (wheel_torque * self.vehicle.gear_efficiency * self.vehicle.regen_factor) / self.vehicle.gear_ratio

        # 3. Thermal Derating & Motor Electrical Current
        derate = self.calculate_derate_factor(self.T_motor)
        effective_I_max = self.motor.I_max * derate

        # Torque = Kt * Current  =>  Current = Torque / Kt
        I_ideal = tau_required / self.motor.Kt

        # Clamp current by thermal derating & physical bounds
        if I_ideal >= 0:
            I_actual = min(I_ideal, effective_I_max)
        else:
            # Regen limit
            I_actual = max(I_ideal, -effective_I_max * 0.7)

        # Actual motor torque produced with clamped current
        tau_actual = self.motor.Kt * I_actual
        self.torque = tau_actual
        self.current = I_actual
        self.rpm = motor_rpm
        self.derate_factor = derate

        # 4. Electrical Voltages & Mechanical Power
        R_actual = self.get_resistance(self.T_motor)
        V_bemf = self.motor.Ke * motor_omega
        V_armature = V_bemf + (I_actual * R_actual)
        self.V_bemf = V_bemf
        self.V_terminal = min(self.motor.V_dc, max(-self.motor.V_dc, V_armature))

        P_mech = tau_actual * motor_omega  # [Watts]
        self.P_mech = P_mech

        # 5. Losses (Joule Heating + Core/Iron Parasitic)
        # P_cu = I² * R
        P_cu = (I_actual ** 2) * R_actual
        # Core/Eddy losses scale with motor speed
        P_core = self.motor.core_loss_coeff * (abs(motor_omega) ** 1.5)
        P_heat = P_cu + P_core

        self.P_cu = P_cu
        self.P_core = P_core
        self.P_heat = P_heat

        # 6. Thermal Dissipation (Convective Cooling to Ambient)
        # Higher vehicle speed increases forced airflow over casing / radiator
        R_th_effective = self.thermal.R_th_base / (1.0 + self.thermal.cooling_factor * self.v)
        P_cool = (self.T_motor - self.thermal.T_ambient) / R_th_effective
        self.P_cool = P_cool

        # Thermal ODE: dT/dt = (P_heat - P_cool) / C_th
        dT_dt = (P_heat - P_cool) / self.thermal.C_th
        self.T_motor += dT_dt * dt

        # 7. Alert Status Assessment
        self.alert_level = self.evaluate_alert(self.T_motor)

        # Efficiency calculation
        P_elec_input = max(0.0, (V_armature * I_actual))
        if P_elec_input > 10.0 and P_mech > 0:
            efficiency = min(0.98, max(0.10, P_mech / (P_elec_input + 1e-3)))
        else:
            efficiency = 0.0

        return {
            "v_ms": float(self.v),
            "v_kmh": float(self.v * 3.6),
            "a_ms2": float(self.a),
            "rpm": float(motor_rpm),
            "omega_rads": float(motor_omega),
            "torque_nm": float(tau_actual),
            "current_a": float(I_actual),
            "v_bemf": float(V_bemf),
            "v_terminal": float(self.V_terminal),
            "p_mech_kw": float(P_mech / 1000.0),
            "p_cu_kw": float(P_cu / 1000.0),
            "p_core_kw": float(P_core / 1000.0),
            "p_heat_kw": float(P_heat / 1000.0),
            "p_cool_kw": float(P_cool / 1000.0),
            "t_motor_c": float(self.T_motor),
            "r_winding_ohm": float(R_actual),
            "derate_factor": float(derate),
            "alert_level": self.alert_level,
            "efficiency": float(efficiency),
        }
