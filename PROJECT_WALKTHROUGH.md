# ⚡ EV Motor Digital Twin — Complete Project Walkthrough & Presentation Guide

---

## 1. Executive Summary & Pitch (What to Say)

> **"Our project is a Digital Twin of an Electric Vehicle (EV) motor.**
>
> In simple terms, it is an active software replica that simulates how a physical EV motor behaves under real driving conditions—such as stop-and-go city traffic or high-speed highway cruising.
>
> It tracks vehicle speed, required shaft torque, electrical current demand, and internal heat buildup in real-time. It features an automated safety alert system: if demanding driving causes the motor temperature to cross safe operating limits, the twin raises visual warnings and initiates thermal derating (automatic power throttling) to safeguard the motor from burnout."

---

## 2. The Real-World Engineering Problem

* **Heat is the #1 enemy of EV motors:** When an EV accelerates or carries heavy loads, electrical currents generate intense internal heat ($I^2R$ losses). If the motor gets too hot, wire insulation melts, permanent magnets lose magnetism, and the motor burns out.
* **Physical sensors have blind spots:** You cannot place physical temperature probes inside every tight copper winding or spinning rotor inside an enclosed motor.
* **Physical testing is slow and risky:** You cannot safely test extreme thermal stress scenarios on expensive prototype cars without risking fires or equipment destruction.
* **The Solution:** A **Digital Twin**. By running a virtual model alongside the vehicle, engineers can **predict** internal temperatures in software, detect risks early, and prevent failures.

---

## 3. Screen-by-Screen Dashboard Guide

### 🎛️ Left Sidebar (Control Panel)

1. **Drive Cycle Profile (The Road Trip):**
   * **Synthetic (Accel / Cruise / Brake):** Balanced test with acceleration to 60 km/h, cruise, sprint to 110 km/h, and braking to 0.
   * **Urban Stop-and-Go:** City traffic with red lights; shows high torque spikes and frequent regenerative braking.
   * **Highway Thermal Stress Test:** High-speed sustained cruising (125–140 km/h) up an uphill incline. *(Triggers the Red Critical Overheat Alert)*.
   * **WLTP Simplified:** Standardized automotive test cycle used by manufacturers to certify EV range and efficiency.
   * **Custom Accel-Cruise-Brake:** Sliders to customize speeds and durations.

2. **Motor Specs (BLDC / DC):**
   * **Torque Constant ($K_t$):** Motor "muscle" efficiency. Higher value = more turning force per Amp of electricity.
   * **Winding Resistance ($R_0$):** Internal electrical resistance of copper coils. Higher resistance = more friction = higher heat generation.
   * **Max Current ($I_{max}$):** The maximum current ceiling allowed by the inverter/battery.
   * **DC Bus Voltage:** Main battery pack voltage (typically 400V).

3. **Thermal Model & Alerts:**
   * **Ambient Temp (°C):** Outside weather (e.g. 25°C room temp vs 45°C hot summer day).
   * **Thermal Mass ($C_{th}$):** How heavy the motor stator metal is (absorbs heat).
   * **Thermal Resistance ($R_{th}$):** Resistance to heat dissipation. Lower = better cooling/radiator.
   * **Speed Cooling Coeff:** Forced air cooling from vehicle speed.
   * **Warning & Critical Thresholds:** 85°C (Warning) and 110°C (Critical Overheat).
   * **Enable Thermal Derating Checkbox:** Automatic safety governor that dials down motor current when temperatures get dangerously high.

4. **Vehicle Dynamics:**
   * Vehicle mass (kg), aerodynamic drag ($C_d$), frontal area ($m^2$), and regenerative braking capture percentage.

---

### 🖥️ Main Screen & Graphs

1. **Alert Status Banner:**
   * 🟢 **NORMAL:** Cool & safe ($< 85^\circ\text{C}$). Full 100% torque available.
   * 🟡 **WARNING:** Approaching limits ($85^\circ\text{C} - 110^\circ\text{C}$). Elevated thermal strain.
   * 🔴 **CRITICAL OVERHEAT:** Exceeded safe limit ($> 110^\circ\text{C}$). Thermal derating active.

2. **Top Metric Cards:**
   * **Peak Temperature:** Highest temperature reached during the trip.
   * **Max Current:** Peak electrical draw from battery.
   * **Peak Torque:** Maximum shaft turning force.
   * **Net Energy (kWh):** Total electricity used minus regenerative energy captured.
   * **Regen Energy (kWh):** Free energy recycled back to the battery.
   * **Copper Loss (kWh):** Energy wasted as internal coil heat.

3. **4 Synchronized Plots (Top to Bottom):**
   * **Plot 1: Speed Profile [km/h]** — Vehicle motion over time.
   * **Plot 2: Motor Torque [N·m]** — Positive for acceleration, negative for regen braking.
   * **Plot 3: Armature Current [A]** — Current flowing from battery; matches torque.
   * **Plot 4: Temperature [°C]** — Red curve showing heat buildup over time, showing Warning and Critical dashed thresholds and shaded danger zones.

---

### 📑 Dashboard Tabs

* **Tab 1 (Full Analytics):** Instant full-trip simulation results and a button to download the telemetry CSV.
* **Tab 2 (Live Real-Time Stream):** Interactive live-streaming mode. Click **"Start Live Stream"** to watch the dials and charts animate step-by-step.
* **Tab 3 (Thermal Breakdown):** Engineering view comparing mechanical power vs heat losses, coil resistance rise, and derating factor.

---

## 4. How to Run the Project

```bash
cd ~/Desktop/ev_motor_digital_twin
./run.sh
```
Open **http://localhost:8501** in your browser.

To run tests:
```bash
cd ~/Desktop/ev_motor_digital_twin
source .venv/bin/activate
python3 -m unittest discover tests
```
