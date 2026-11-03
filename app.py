"""
Digital Twin of an EV Motor - Interactive Streamlit Dashboard
Real-time simulation and engineering analytics of BLDC/DC motor dynamics:
- Torque = Kt × Current
- Joule heating: Temperature rise driven by I²R losses over drive-cycle speed profiles
- Thermal dissipation & forced convection cooling
- Live threshold alerts (Normal / Warning / Critical Overheat) & Thermal Derating
"""

import time
import io
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import streamlit as st

from models.motor_model import (
    EVMotorTwin,
    MotorParameters,
    ThermalParameters,
    VehicleParameters,
)
from models.drive_cycles import (
    get_drive_cycle,
    list_available_drive_cycles,
    generate_accel_cruise_brake,
)
from simulation.simulator import MotorSimulator, SimulationSummary

# Page Configuration
st.set_page_config(
    page_title="EV Motor Digital Twin",
    page_icon="⚡",
    layout="wide",
    initial_sidebar_state="expanded",
)

# Custom CSS for modern styling
st.markdown("""
<style>
    .main-title {
        font-size: 2.2rem;
        font-weight: 800;
        background: linear-gradient(90deg, #1E88E5 0%, #00E676 100%);
        -webkit-background-clip: text;
        -webkit-text-fill-color: transparent;
        margin-bottom: 0.1rem;
    }
    .sub-title {
        color: #78909C;
        font-size: 1.05rem;
        margin-bottom: 1.2rem;
    }
    .metric-card {
        background: rgba(30, 41, 59, 0.6);
        border: 1px solid rgba(255, 255, 255, 0.08);
        border-radius: 12px;
        padding: 16px;
        text-align: center;
        box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
    }
    .metric-title {
        color: #94A3B8;
        font-size: 0.85rem;
        text-transform: uppercase;
        letter-spacing: 0.05em;
        margin-bottom: 4px;
    }
    .metric-value {
        font-size: 1.7rem;
        font-weight: 700;
        color: #F8FAFC;
    }
    .alert-normal {
        background-color: rgba(34, 197, 94, 0.15);
        border-left: 5px solid #22C55E;
        padding: 12px 16px;
        border-radius: 8px;
        color: #86EFAC;
        font-weight: 600;
        margin-bottom: 15px;
    }
    .alert-warning {
        background-color: rgba(245, 158, 11, 0.18);
        border-left: 5px solid #F59E0B;
        padding: 12px 16px;
        border-radius: 8px;
        color: #FCD34D;
        font-weight: 600;
        margin-bottom: 15px;
    }
    .alert-critical {
        background-color: rgba(239, 68, 68, 0.22);
        border-left: 5px solid #EF4444;
        padding: 14px 18px;
        border-radius: 8px;
        color: #FCA5A5;
        font-weight: 700;
        font-size: 1.05rem;
        box-shadow: 0 0 12px rgba(239, 68, 68, 0.35);
        margin-bottom: 15px;
    }
    [data-testid="stMetric"] {
        background: rgba(30, 41, 59, 0.6);
        border: 1px solid rgba(255, 255, 255, 0.08);
        border-radius: 10px;
        padding: 10px 14px;
        min-height: 95px;
    }
</style>
""", unsafe_allow_html=True)


def build_sidebar():
    """Build interactive sidebar for parameter inputs."""
    st.sidebar.header("⚙️ Digital Twin Configuration")

    # 1. Drive Cycle Selection
    st.sidebar.subheader("🚗 1. Drive Cycle Profile")
    cycle_options = list_available_drive_cycles() + ["Custom Accel-Cruise-Brake"]
    selected_cycle_name = st.sidebar.selectbox(
        "Select Drive Cycle",
        cycle_options,
        index=0,
        help="Select a standard or stress drive cycle to feed the digital twin.",
    )

    if selected_cycle_name == "Custom Accel-Cruise-Brake":
        c_col1, c_col2 = st.sidebar.columns(2)
        custom_v1 = c_col1.slider("Cruise 1 (km/h)", 20.0, 100.0, 60.0, 5.0)
        custom_t1 = c_col2.slider("Accel 1 Time (s)", 2.0, 20.0, 8.0, 1.0)
        custom_v2 = c_col1.slider("Cruise 2 (km/h)", 60.0, 160.0, 120.0, 5.0)
        custom_t2 = c_col2.slider("Cruise 2 Time (s)", 5.0, 60.0, 25.0, 5.0)
        custom_tb = st.sidebar.slider("Braking Time (s)", 3.0, 25.0, 12.0, 1.0)

        drive_cycle_df = generate_accel_cruise_brake(
            total_time=100.0,
            cruise_speed_1=custom_v1,
            accel_time_1=custom_t1,
            cruise_speed_2=custom_v2,
            cruise_time_2=custom_t2,
            brake_time=custom_tb,
        )
    else:
        drive_cycle_df = get_drive_cycle(selected_cycle_name)

    # 2. Motor Parameters
    st.sidebar.subheader("⚡ 2. BLDC / DC Motor Specs")
    col_m1, col_m2 = st.sidebar.columns(2)
    kt = col_m1.number_input(
        "Torque Const Kt (N·m/A)",
        min_value=0.10,
        max_value=2.50,
        value=0.65,
        step=0.05,
        help="Motor torque constant: Torque = Kt × Current",
    )
    r0 = col_m2.number_input(
        "Winding R₀ (Ω)",
        min_value=0.005,
        max_value=0.500,
        value=0.045,
        step=0.005,
        format="%.3f",
        help="Base copper armature resistance at 20°C (drives I²R heat).",
    )
    i_max = col_m1.number_input("Max Current (A)", 50.0, 600.0, 260.0, 10.0)
    v_dc = col_m2.number_input("DC Bus (V)", 100.0, 800.0, 400.0, 50.0)

    motor_params = MotorParameters(
        Kt=kt,
        Ke=kt,  # SI consistent
        R0=r0,
        I_max=i_max,
        V_dc=v_dc,
    )

    # 3. Thermal Parameters & Thresholds
    st.sidebar.subheader("🔥 3. Thermal Model & Alerts")
    col_t1, col_t2 = st.sidebar.columns(2)
    t_ambient = col_t1.number_input("Ambient Temp (°C)", -10.0, 50.0, 25.0, 1.0)
    c_th = col_t2.number_input(
        "Thermal Mass Cth (J/°C)",
        min_value=500.0,
        max_value=15000.0,
        value=2800.0,
        step=200.0,
        help="Heat capacity of motor stator/windings.",
    )
    r_th = col_t1.number_input(
        "Thermal Res Rth (°C/W)",
        min_value=0.05,
        max_value=2.00,
        value=0.45,
        step=0.05,
        help="Resistance to heat dissipation to ambient.",
    )
    cooling_factor = col_t2.slider(
        "Speed Cooling Coeff",
        0.0, 0.10, 0.035, 0.005,
        help="Enhancement of convective cooling from airflow at higher speeds.",
    )

    st.sidebar.markdown("**Alert Thresholds**")
    col_th1, col_th2 = st.sidebar.columns(2)
    t_warning = col_th1.slider("Warning Limit (°C)", 50.0, 110.0, 85.0, 1.0)
    t_critical = col_th2.slider("Critical Limit (°C)", 80.0, 150.0, 110.0, 1.0)

    enable_derate = st.sidebar.checkbox(
        "Enable Thermal Derating (BMS/MCU Guard)",
        value=True,
        help="Automatically throttles motor current when winding exceeds threshold to prevent catastrophic thermal burnout.",
    )

    thermal_params = ThermalParameters(
        C_th=c_th,
        R_th_base=r_th,
        T_ambient=t_ambient,
        T_warning=t_warning,
        T_critical=t_critical,
        cooling_factor=cooling_factor,
    )

    # 4. Vehicle Dynamics
    with st.sidebar.expander("🚗 Vehicle Dynamics Parameters", expanded=False):
        col_v1, col_v2 = st.sidebar.columns(2)
        mass = col_v1.number_input("Mass (kg)", 500.0, 3500.0, 1650.0, 50.0)
        cd = col_v2.number_input("Drag Coeff Cd", 0.15, 0.50, 0.26, 0.01)
        area = col_v1.number_input("Frontal Area (m²)", 1.0, 4.0, 2.25, 0.1)
        gear_ratio = col_v2.number_input("Gear Ratio", 4.0, 15.0, 8.5, 0.5)
        wheel_r = col_v1.number_input("Wheel Radius (m)", 0.20, 0.50, 0.315, 0.01)
        regen_eff = col_v2.slider("Regen Capture", 0.0, 1.0, 0.75, 0.05)

        vehicle_params = VehicleParameters(
            mass=mass,
            Cd=cd,
            frontal_area=area,
            gear_ratio=gear_ratio,
            wheel_radius=wheel_r,
            regen_factor=regen_eff,
        )

    return (
        selected_cycle_name,
        drive_cycle_df,
        motor_params,
        thermal_params,
        vehicle_params,
        enable_derate,
    )


def create_plotly_dashboard(
    df: pd.DataFrame,
    thermal_params: ThermalParameters,
    motor_params: MotorParameters,
    fixed_x_max: float = None,
    fixed_y_ranges: bool = False,
):
    """Generates comprehensive synchronized 4-panel engineering plot."""
    fig = make_subplots(
        rows=4,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.06,
        subplot_titles=(
            "Vehicle Speed Profile [km/h]",
            "Motor Shaft Torque: τ = Kt · I [N·m]",
            "Armature Current [A]",
            "Motor Temperature & I²R Thermal Evolution [°C]",
        ),
    )

    t = df["time"]

    # 1. Speed Plot
    fig.add_trace(
        go.Scatter(
            x=t,
            y=df["v_kmh"],
            name="Vehicle Speed",
            line=dict(color="#00E5FF", width=2.5),
            fill="tozeroy",
            fillcolor="rgba(0, 229, 255, 0.08)",
        ),
        row=1, col=1,
    )

    # 2. Torque Plot
    fig.add_trace(
        go.Scatter(
            x=t,
            y=df["torque_nm"],
            name="Motor Torque (N·m)",
            line=dict(color="#FFD600", width=2.2),
        ),
        row=2, col=1,
    )
    # Zero line for torque
    fig.add_hline(y=0, line_dash="dash", line_color="gray", opacity=0.5, row=2, col=1)

    # 3. Current Plot
    fig.add_trace(
        go.Scatter(
            x=t,
            y=df["current_a"],
            name="Armature Current (A)",
            line=dict(color="#FF9100", width=2.2),
        ),
        row=3, col=1,
    )
    # Motor max current limit line
    fig.add_hline(
        y=motor_params.I_max,
        line_dash="dot",
        line_color="#EF4444",
        annotation_text="+I_max limit",
        annotation_position="top right",
        row=3, col=1,
    )
    fig.add_hline(
        y=-motor_params.I_max * 0.7,
        line_dash="dot",
        line_color="#22C55E",
        annotation_text="-Regen limit",
        annotation_position="bottom right",
        row=3, col=1,
    )

    # 4. Temperature Plot & Thresholds
    fig.add_trace(
        go.Scatter(
            x=t,
            y=df["t_motor_c"],
            name="Winding Temp (°C)",
            line=dict(color="#FF1744", width=3.0),
        ),
        row=4, col=1,
    )
    # Ambient Temp line
    fig.add_hline(
        y=thermal_params.T_ambient,
        line_dash="dash",
        line_color="#64748B",
        annotation_text=f"Ambient ({thermal_params.T_ambient}°C)",
        annotation_position="bottom left",
        row=4, col=1,
    )
    # Warning Threshold Line
    fig.add_hline(
        y=thermal_params.T_warning,
        line_dash="dash",
        line_color="#F59E0B",
        line_width=2,
        annotation_text=f"Warning ({thermal_params.T_warning}°C)",
        annotation_position="top left",
        row=4, col=1,
    )
    # Critical Threshold Line
    fig.add_hline(
        y=thermal_params.T_critical,
        line_dash="dash",
        line_color="#EF4444",
        line_width=2.5,
        annotation_text=f"CRITICAL OVERHEAT ({thermal_params.T_critical}°C)",
        annotation_position="top left",
        row=4, col=1,
    )

    # Shaded warning and critical regions
    fig.add_hrect(
        y0=thermal_params.T_warning,
        y1=thermal_params.T_critical,
        fillcolor="rgba(245, 158, 11, 0.12)",
        line_width=0,
        row=4, col=1,
    )
    fig.add_hrect(
        y0=thermal_params.T_critical,
        y1=max(thermal_params.T_critical + 30.0, df["t_motor_c"].max() + 5.0),
        fillcolor="rgba(239, 68, 68, 0.20)",
        line_width=0,
        row=4, col=1,
    )

    fig.update_xaxes(title_text="Time [s]", row=4, col=1)

    if fixed_x_max is not None and fixed_x_max > 0:
        fig.update_xaxes(range=[0, fixed_x_max], autorange=False)

    if fixed_y_ranges:
        max_v = max(100.0, float(df["v_kmh"].max()) * 1.15) if len(df) > 0 else 120.0
        fig.update_yaxes(range=[0, max_v], autorange=False, row=1, col=1)
        max_tau = float(motor_params.I_max * motor_params.Kt * 1.05)
        min_tau = -float(motor_params.I_max * 0.7 * motor_params.Kt * 1.05)
        fig.update_yaxes(range=[min_tau, max_tau], autorange=False, row=2, col=1)
        fig.update_yaxes(range=[-motor_params.I_max * 0.8, motor_params.I_max * 1.1], autorange=False, row=3, col=1)
        fig.update_yaxes(range=[thermal_params.T_ambient - 5.0, max(thermal_params.T_critical + 25.0, 110.0)], autorange=False, row=4, col=1)

    fig.update_layout(
        height=850,
        margin=dict(l=50, r=30, t=40, b=40),
        hovermode="x unified",
        template="plotly_dark",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        uirevision="twin_stream_fixed",
        transition={"duration": 0},
    )
    return fig


def plot_chart(container_or_st, fig, key=None):
    """Render plotly chart without deprecation warning and with responsive full-width."""
    try:
        if key:
            return container_or_st.plotly_chart(fig, width="stretch", key=key)
        return container_or_st.plotly_chart(fig, width="stretch")
    except TypeError:
        if key:
            return container_or_st.plotly_chart(fig, use_container_width=True, key=key)
        return container_or_st.plotly_chart(fig, use_container_width=True)


def get_alert_banner_html(current_temp: float, alert_level: str, t_warn: float, t_crit: float, derate: float) -> str:
    """Returns contextual alert banner HTML based on real-time temperature."""
    if alert_level == "CRITICAL":
        return f"""
        <div class="alert-critical">
            🚨 <b>CRITICAL OVERHEAT ALARM!</b> Motor winding temperature reached <b>{current_temp:.1f}°C</b>, 
            exceeding the critical safe threshold of <b>{t_crit:.1f}°C</b>! 
            Thermal Derating Active: Motor current restricted to <b>{derate*100:.0f}%</b> to prevent insulation breakdown.
        </div>
        """
    elif alert_level == "WARNING":
        return f"""
        <div class="alert-warning">
            ⚠️ <b>TEMPERATURE WARNING:</b> Motor winding temperature is at <b>{current_temp:.1f}°C</b> 
            (exceeded warning limit of <b>{t_warn:.1f}°C</b>). 
            Continuous I²R Joule heating is outpacing thermal dissipation. Derating factor: <b>{derate*100:.0f}%</b>.
        </div>
        """
    else:
        return f"""
        <div class="alert-normal">
            ✅ <b>THERMAL STATE NORMAL:</b> Motor temperature is safe at <b>{current_temp:.1f}°C</b> 
            (Safe Operating Zone &lt; {t_warn:.1f}°C). Full 100% torque available.
        </div>
        """


def render_alert_banner(current_temp: float, alert_level: str, t_warn: float, t_crit: float, derate: float, target=None):
    """Renders contextual alert banner based on real-time temperature."""
    html = get_alert_banner_html(current_temp, alert_level, t_warn, t_crit, derate)
    if target is not None:
        target.markdown(html, unsafe_allow_html=True)
    else:
        st.markdown(html, unsafe_allow_html=True)


def main():
    # Title & Header
    st.markdown('<div class="main-title">⚡ Digital Twin of an EV Motor</div>', unsafe_allow_html=True)
    st.markdown(
        '<div class="sub-title">'
        'Simplified BLDC / DC Motor Physics (<b>Torque = Kt × Current</b>, <b>dT/dt = (I²R - P_cool)/C_th</b>) '
        'driven by dynamic drive cycles with live threshold alerting and thermal derating protection.'
        '</div>',
        unsafe_allow_html=True,
    )

    # Sidebar parameters
    (
        cycle_name,
        cycle_df,
        motor_params,
        thermal_params,
        vehicle_params,
        enable_derate,
    ) = build_sidebar()

    # Instantiate Digital Twin and Simulator
    twin = EVMotorTwin(
        motor_params=motor_params,
        thermal_params=thermal_params,
        vehicle_params=vehicle_params,
        initial_temp=thermal_params.T_ambient,
        enable_thermal_derating=enable_derate,
    )
    simulator = MotorSimulator(twin)

    # Tabs for different operational modes
    tab1, tab2, tab3 = st.tabs([
        "📊 Full Drive-Cycle Analytics",
        "🔴 Live Real-Time Twin Stream",
        "🔬 Thermal & Loss Model Breakdown",
    ])

    # ----------------------------------------------------
    # TAB 1: Full Drive-Cycle Analytics
    # ----------------------------------------------------
    with tab1:
        st.markdown(f"### Mission Telemetry & Thermal Assessment: `{cycle_name}`")

        # Run complete simulation
        df_full, summary = simulator.run_full(cycle_df, initial_temp=thermal_params.T_ambient)

        # High-level Alert Banner based on peak temperature in cycle
        peak_alert = twin.evaluate_alert(summary.peak_temp_c)
        render_alert_banner(
            summary.peak_temp_c,
            peak_alert,
            thermal_params.T_warning,
            thermal_params.T_critical,
            twin.calculate_derate_factor(summary.peak_temp_c),
        )

        # Key Metrics Row
        mcol1, mcol2, mcol3, mcol4, mcol5, mcol6 = st.columns(6)
        mcol1.metric("Peak Temperature", f"{summary.peak_temp_c:.1f} °C", delta=f"{summary.peak_temp_c - thermal_params.T_ambient:+.1f} °C", delta_color="inverse")
        mcol2.metric("Max Current", f"{summary.max_current_a:.1f} A")
        mcol3.metric("Peak Torque", f"{summary.max_torque_nm:.1f} N·m")
        mcol4.metric("Net Energy", f"{summary.net_energy_kwh:.3f} kWh")
        mcol5.metric("Regen Energy", f"{summary.total_energy_regen_kwh:.3f} kWh")
        mcol6.metric("Copper I²R Loss", f"{summary.total_copper_loss_kwh:.3f} kWh")

        # Plotly synchronized dashboard
        fig = create_plotly_dashboard(df_full, thermal_params, motor_params)
        plot_chart(st, fig, key="tab1_plotly_dashboard")

        # Export telemetry data
        st.markdown("#### 📥 Export Simulation Data")
        csv_buffer = io.StringIO()
        df_full.to_csv(csv_buffer, index=False)
        st.download_button(
            label="Download Drive Cycle Telemetry CSV",
            data=csv_buffer.getvalue(),
            file_name=f"ev_motor_twin_{cycle_name.replace(' ', '_').lower()}.csv",
            mime="text/csv",
        )

    # ----------------------------------------------------
    # TAB 2: Live Real-Time Twin Stream
    # ----------------------------------------------------
    with tab2:
        st.markdown("### 🔴 Real-Time Motor Telemetry Live-Stream")
        st.caption("Streams drive-cycle steps interactively. Observe real-time current spikes and thermal rise.")

        col_ctl1, col_ctl2, col_ctl3 = st.columns([1, 1, 2])
        start_btn = col_ctl1.button("▶️ Start Live Stream", key="live_start_btn", type="primary")
        sim_speed = col_ctl2.select_slider("Playback Speed", options=[1, 2, 5, 10, 20], value=5)

        total_duration = float(cycle_df["time"].iloc[-1])
        total_steps = len(cycle_df)

        # Progress bar placeholder
        progress_placeholder = st.empty()

        # Alert banner placeholder
        alert_placeholder = st.empty()

        # Pre-create 5 metric columns to prevent DOM recreation & height jumping
        col1, col2, col3, col4, col5 = st.columns(5)
        m1 = col1.empty()
        m2 = col2.empty()
        m3 = col3.empty()
        m4 = col4.empty()
        m5 = col5.empty()

        status_placeholder = st.empty()
        chart_placeholder = st.empty()

        if not start_btn:
            render_alert_banner(
                thermal_params.T_ambient,
                "NORMAL",
                thermal_params.T_warning,
                thermal_params.T_critical,
                1.0,
                target=alert_placeholder,
            )
            m1.metric("Live Speed", "0.0 km/h")
            m2.metric("Motor Torque (τ = Kt·I)", "0.0 N·m")
            m3.metric("Current (I)", "0.0 A")
            m4.metric("Winding Temp", f"{thermal_params.T_ambient:.1f} °C")
            m5.metric("Motor RPM", "0 RPM")
            status_placeholder.info("Click **'▶️ Start Live Stream'** above to run the animated digital twin simulation.")

            # Initial baseline chart with full time range fixed (prevents layout collapse & flickering)
            init_df = cycle_df.iloc[[0]].copy()
            init_df["v_kmh"] = 0.0
            init_df["torque_nm"] = 0.0
            init_df["current_a"] = 0.0
            init_df["t_motor_c"] = thermal_params.T_ambient
            fig_init = create_plotly_dashboard(
                init_df,
                thermal_params,
                motor_params,
                fixed_x_max=total_duration,
                fixed_y_ranges=True,
            )
            plot_chart(chart_placeholder, fig_init)
        else:
            status_placeholder.empty()
            stream_twin = EVMotorTwin(
                motor_params=motor_params,
                thermal_params=thermal_params,
                vehicle_params=vehicle_params,
                initial_temp=thermal_params.T_ambient,
                enable_thermal_derating=enable_derate,
            )
            sim_stream = MotorSimulator(stream_twin)

            history = []
            dt_sim = cycle_df["time"].iloc[1] - cycle_df["time"].iloc[0] if total_steps > 1 else 0.1
            dt_sleep = max(0.005, dt_sim / sim_speed)

            # Throttle chart updates to eliminate browser blinking and layout thrashing
            last_chart_time = 0.0
            last_ui_time = 0.0
            CHART_REFRESH_SEC = 0.22  # ~4.5 FPS: perfectly smooth for Plotly without blinking
            UI_REFRESH_SEC = 0.06     # ~16 updates/sec for smooth number flow

            step_count = 0
            for step_data in sim_stream.stream_step(cycle_df, initial_temp=thermal_params.T_ambient):
                history.append(step_data)
                step_count += 1

                now = time.time()
                is_last_step = (step_count == total_steps)

                # 1. Update Metrics smoothly in-place (no DOM reconstruction, no height collapse)
                if (now - last_ui_time >= UI_REFRESH_SEC) or is_last_step:
                    last_ui_time = now
                    curr_step = step_data

                    render_alert_banner(
                        curr_step["t_motor_c"],
                        curr_step["alert_level"],
                        thermal_params.T_warning,
                        thermal_params.T_critical,
                        curr_step["derate_factor"],
                        target=alert_placeholder,
                    )

                    m1.metric("Live Speed", f"{curr_step['v_kmh']:.1f} km/h")
                    m2.metric("Motor Torque (τ = Kt·I)", f"{curr_step['torque_nm']:.1f} N·m")
                    m3.metric("Current (I)", f"{curr_step['current_a']:.1f} A")
                    m4.metric(
                        "Winding Temp",
                        f"{curr_step['t_motor_c']:.1f} °C",
                        delta=f"{curr_step['p_cu_kw']:.2f} kW heat",
                        delta_color="inverse",
                    )
                    m5.metric("Motor RPM", f"{curr_step['rpm']:.0f} RPM")
                    progress_placeholder.progress(min(1.0, step_count / total_steps))

                # 2. Update Plotly Chart with fixed axis and uirevision (ZERO blinking!)
                if (now - last_chart_time >= CHART_REFRESH_SEC) or is_last_step:
                    last_chart_time = now
                    df_live = pd.DataFrame(history)
                    fig_live = create_plotly_dashboard(
                        df_live,
                        thermal_params,
                        motor_params,
                        fixed_x_max=total_duration,
                        fixed_y_ranges=True,
                    )
                    plot_chart(chart_placeholder, fig_live)

                time.sleep(dt_sleep)

            progress_placeholder.empty()
            status_placeholder.success("🏁 Drive Cycle Stream Completed! Full mission profile displayed.")

    # ----------------------------------------------------
    # TAB 3: Thermal & Loss Model Breakdown
    # ----------------------------------------------------
    with tab3:
        st.markdown("### 🔬 In-Depth Loss & Thermal Dissipation Physics")
        st.markdown(r"""
        The digital twin models continuous electromechanical losses and heat transfer via:
        1. **Joule Loss (Copper)**: $P_{cu} = I^2 \cdot R(T)$ where $R(T) = R_0 \cdot [1 + \\alpha_{cu}(T - T_{ref})]$
        2. **Core/Iron Loss**: $P_{core} = k_{iron} \cdot \\omega^{1.5}$
        3. **Heat Transfer ODE**: $\\frac{dT}{dt} = \\frac{P_{heat} - P_{cool}}{C_{th}}$
        4. **Forced Convection Cooling**: $P_{cool} = \\frac{T - T_{ambient}}{R_{th} / (1 + k_{cool} \\cdot v)}$
        """)

        # Execute cycle to inspect loss time series
        df_loss, _ = simulator.run_full(cycle_df, initial_temp=thermal_params.T_ambient)

        fig_loss = go.Figure()
        fig_loss.add_trace(go.Scatter(
            x=df_loss["time"],
            y=df_loss["p_cu_kw"],
            name="Joule Loss (I²R) [kW]",
            line=dict(color="#FF5252", width=2.5),
            fill="tozeroy",
            fillcolor="rgba(255, 82, 82, 0.15)",
        ))
        fig_loss.add_trace(go.Scatter(
            x=df_loss["time"],
            y=df_loss["p_cool_kw"],
            name="Heat Dissipation to Ambient [kW]",
            line=dict(color="#448AFF", width=2.0),
        ))
        fig_loss.add_trace(go.Scatter(
            x=df_loss["time"],
            y=df_loss["p_mech_kw"],
            name="Mechanical Output Power [kW]",
            line=dict(color="#00E676", width=2.0, dash="dot"),
        ))
        fig_loss.update_layout(
            title="Instantaneous Power & Thermal Loss Comparison",
            xaxis_title="Time [s]",
            yaxis_title="Power [kW]",
            template="plotly_dark",
            height=450,
            hovermode="x unified",
        )
        plot_chart(st, fig_loss, key="tab3_fig_loss")

        col_l1, col_l2 = st.columns(2)
        with col_l1:
            st.markdown("#### ⚡ Dynamic Winding Resistance Evolution")
            fig_r = go.Figure()
            fig_r.add_trace(go.Scatter(
                x=df_loss["time"],
                y=df_loss["r_winding_ohm"] * 1000.0,
                name="Winding Resistance (mΩ)",
                line=dict(color="#FFA726", width=2.5),
            ))
            fig_r.update_layout(
                xaxis_title="Time [s]",
                yaxis_title="Armature Resistance [mΩ]",
                template="plotly_dark",
                height=300,
            )
            plot_chart(st, fig_r, key="tab3_fig_r")

        with col_l2:
            st.markdown("#### 🛡️ Thermal Derating Protection Factor")
            fig_d = go.Figure()
            fig_d.add_trace(go.Scatter(
                x=df_loss["time"],
                y=df_loss["derate_factor"] * 100.0,
                name="Allowable Power Factor (%)",
                line=dict(color="#29B6F6", width=2.5),
                fill="tozeroy",
                fillcolor="rgba(41, 182, 246, 0.15)",
            ))
            fig_d.update_layout(
                xaxis_title="Time [s]",
                yaxis_title="Derating Factor [%]",
                template="plotly_dark",
                height=300,
            )
            plot_chart(st, fig_d, key="tab3_fig_d")


if __name__ == "__main__":
    main()
