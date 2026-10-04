# ⚡ Digital Twin of an EV Motor

A physics-based digital twin of an Electric Vehicle (EV) BLDC / Permanent Magnet DC motor. Models electromechanical drive dynamics, tractive resistance, $I^2R$ Joule heating losses, heat dissipation, and dynamic thermal protection over realistic automotive drive cycles.

Built with an interactive **Streamlit dashboard** for real-time telemetry streaming, multi-variable synchronized plotting, and temperature threshold alerting.

---

## 📌 Project Structure

```
ev_motor_digital_twin/
├── app.py                  # Streamlit Interactive Dashboard
├── models/
│   ├── __init__.py
│   ├── motor_model.py      # Motor electromechanical & lumped-parameter thermal ODE model
│   └── drive_cycles.py     # Drive cycles (Accel/Cruise/Brake, Urban, Highway, WLTP)
├── simulation/
│   ├── __init__.py
│   └── simulator.py        # Simulation runner, live step streaming & KPI computation
├── tests/
│   └── test_motor.py       # Unit test suite (Torque, I²R heating, cooling, alerts)
├── requirements.txt        # Python package dependencies
├── run.sh                  # Execution launcher script
└── README.md               # Project documentation
```

---

## 📐 Mathematical Formulation

### 1. Electromechanical Torque & Back-EMF
- **Motor Shaft Torque**:
  $$\tau_{\text{motor}} = K_t \cdot I$$
  where:
  - $K_t$: Motor torque constant $[\text{N}\cdot\text{m}/\text{A}]$
  - $I$: Armature current $[\text{A}]$
- **Back-EMF Voltage**:
  $$V_{\text{bemf}} = K_e \cdot \omega_{\text{motor}}$$
- **Terminal Voltage**:
  $$V_{\text{terminal}} = V_{\text{bemf}} + I \cdot R(T)$$

### 2. Vehicle Road Load (Tractive Force)
For vehicle velocity $v$ and acceleration $a$:
$$F_{\text{tractive}} = F_{\text{rolling}} + F_{\text{aero}} + F_{\text{accel}} + F_{\text{grade}}$$
$$F_{\text{rolling}} = c_{rr} \cdot M \cdot g \quad (v > 0)$$
$$F_{\text{aero}} = \frac{1}{2} \rho_{\text{air}} C_d A v^2$$
$$F_{\text{accel}} = M \cdot a$$

Wheel torque is transferred to the motor shaft via single-speed transmission:
$$\tau_{\text{wheel}} = F_{\text{tractive}} \cdot r_{\text{wheel}}$$
$$\tau_{\text{motor}} = \frac{\tau_{\text{wheel}}}{G_r \cdot \eta_{\text{gear}}}$$

### 3. Thermal Dynamics & Joule Heating ($I^2R$)
- **Temperature-Dependent Copper Winding Resistance**:
  $$R(T) = R_0 \cdot \left[1 + \alpha_{cu} \cdot (T - T_{\text{ref}})\right]$$
  where $\alpha_{cu} \approx 0.00393\ \text{K}^{-1}$ is the temperature coefficient of copper.
- **Heat Generation**:
  $$P_{\text{heat}} = I^2 \cdot R(T) + P_{\text{core}}$$
- **Forced Convective Dissipation to Ambient**:
  $$P_{\text{cool}} = \frac{T - T_{\text{ambient}}}{R_{th,\text{eff}}(v)}, \quad R_{th,\text{eff}}(v) = \frac{R_{th,0}}{1 + k_{\text{cool}} \cdot v}$$
- **Thermal Lumped-Capacitance Differential Equation**:
  $$\frac{dT}{dt} = \frac{P_{\text{heat}} - P_{\text{cool}}}{C_{th}}$$
  where $C_{th}$ is thermal mass $[\text{J}/^\circ\text{C}]$.

### 4. Safety Alerts & Thermal Derating
- **NORMAL** ($T < T_{\text{warning}}$): Full motor capability available (100% torque).
- **WARNING** ($T_{\text{warning}} \le T < T_{\text{critical}}$): Visual warning alert; elevated thermal strain.
- **CRITICAL** ($T \ge T_{\text{critical}}$): Critical overheat alert banner. Active BMS/MCU thermal governor throttles max allowable current to prevent winding insulation failure.

---

## 🚀 Setup & Execution

### Option 1: Using the launcher script
```bash
cd ~/Desktop/ev_motor_digital_twin
./run.sh
```

### Option 2: Manual setup
```bash
cd ~/Desktop/ev_motor_digital_twin
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
streamlit run app.py
```
Open your browser at **http://localhost:8501**.

---

## 🧪 Running Unit Tests
```bash
source .venv/bin/activate
python3 -m unittest discover tests
```
All 7 tests validate:
1. Torque constant relation ($\tau = K_t I$)
2. Copper thermal resistance rise with temperature
3. Joule heating temperature elevation ($dT/dt > 0$)
4. Idle cooling back towards ambient
5. Threshold alert triggering (`NORMAL` $\to$ `WARNING` $\to$ `CRITICAL`)
6. Thermal derating factor throttling
7. Full drive-cycle simulation consistency
