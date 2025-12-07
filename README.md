# 5-DOF Robotic Arm Control System

Advanced control system for a 5-DOF robotic arm using ST3215 servo motors. This repository contains **multiple control implementations** ranging from direct Python control to high-performance embedded systems, providing flexibility for different applications and performance requirements.

## 🎯 Overview

This project features **four distinct control approaches**, each with unique strengths:

1. **🚀 Teensy 4.1 QP-MPC** *(Recommended - Production Ready)*
   - 815Hz control loop with Model Predictive Control
   - Sub-millimeter precision with automatic tuning
   - Best for: Research applications, high-precision tasks

2. **⚡ ESP32 MPC Hybrid**
   - 200Hz Jacobian + MPC constraint projection
   - Balanced performance and complexity
   - Best for: Real-time applications, smooth motion

3. **🔧 ESP32 Jacobian IK**
   - Pure Jacobian velocity control with PID
   - Simple and reliable
   - Best for: Basic automation, learning

4. **🐍 Direct Python Control**
   - PC-based control (Jacob IK / Analytical IK)
   - No microcontroller firmware needed
   - Best for: Prototyping, testing, education

## 📁 Complete Project Structure

```
📦 ST3215-5DOF-Robotic-Arm/
│
├── 🚀 microcontroller_arm_control/     # HIGH-PERFORMANCE EMBEDDED
│   ├── firmware_teensy_qp/             # ★ RECOMMENDED: Teensy 4.1 (815Hz QP-MPC)
│   │   ├── src/main.cpp                # Main control loop
│   │   ├── include/
│   │   │   ├── TinyMPC.h               # QP-MPC controller implementation
│   │   │   ├── Kinematics.h            # FK/IK/Jacobian algorithms
│   │   │   ├── JacobianIK.h            # Jacobian velocity control
│   │   │   └── ServoDriver.h           # ST3215 servo communication
│   │   └── platformio.ini
│   │
│   └── pc_software/                    # Python GUI for Teensy
│       ├── arm_gui.py                  # Main control interface
│       ├── mpc_tuner.py                # Automatic MPC optimizer
│       ├── arm_config.json             # Robot configuration
│       └── mpc_params.json             # Tuned MPC parameters
│
├── ⚡ esp32_arm_control/                # ESP32 IMPLEMENTATIONS
│   ├── firmware/                       # ESP32 Jacobian IK (PID-based)
│   │   ├── src/main.cpp                # Pure Jacobian control (~200Hz)
│   │   ├── include/
│   │   │   ├── Kinematics.h
│   │   │   ├── ServoDriver.h
│   │   │   ├── PID.h
│   │   │   └── Trajectory.h
│   │   └── platformio.ini
│   │
│   ├── firmware_mpc/                   # ESP32 Hybrid (Jacobian + MPC)
│   │   ├── src/main.cpp                # Jacobian + MPC constraints
│   │   ├── include/
│   │   │   ├── TinyMPC.h
│   │   │   ├── JacobianController.h
│   │   │   ├── AnalyticalIK.h
│   │   │   └── ArmModel.h
│   │   └── platformio.ini
│   │
│   └── pc_software/                    # Python GUI for ESP32
│       ├── arm_gui.py                  # Control interface (compatible with both firmwares)
│       ├── auto_pid_calibrate.py       # PID auto-tuner
│       └── arm_config.json
│
├── 🐍 jacob_ik/                         # PYTHON JACOBIAN CONTROL
│   ├── arm_controller.py               # Direct USB control (PC-based)
│   ├── motor_gui.py                    # Real-time GUI with 3D visualization
│   ├── kinematics.py                   # FK/IK/Jacobian/DLS implementation
│   ├── trajectory.py                   # Path planning
│   ├── joint_control.py                # Per-joint controllers
│   ├── calibrate_servos.py             # Servo calibration tool
│   └── servo_limits.json               # Joint limits and parameters
│
├── 🔬 analytical_ik/                    # PYTHON ANALYTICAL CONTROL
│   ├── arm_controller.py               # Direct USB control (PC-based)
│   ├── motor_gui.py                    # GUI interface
│   ├── kinematics.py                   # Closed-form IK solution
│   ├── calibrate_servos.py             # Calibration tool
│   └── servo_limits.json
│
├── 🛠️ Arm_Design/                       # MECHANICAL CAD FILES
│   ├── Assembly.SLDASM                 # SolidWorks assembly
│   ├── *.SLDPRT                        # Individual parts
│   └── STL/                            # 3D printable files
│
├── 🧪 waveshare_diagnostic/             # TESTING & DIAGNOSTICS
│   ├── test_motor1.py                  # Individual motor tests
│   ├── scan_baud.py                    # Baud rate scanner
│   └── recover_servos.py               # Servo recovery tools
│
├── 📋 requirements.txt                  # Python dependencies
└── 📖 README.md                         # This file

```

---

## 🚀 Implementation #1: Teensy 4.1 QP-MPC (Recommended)

### Overview
**Highest performance** system with Model Predictive Control running at **815Hz**. Uses Quadratic Programming (QP) solver for optimal control with terminal posture constraints.

### Key Features
- **815Hz Control Loop** (IMXRT1062 @ 600MHz)
- **QP-MPC Controller** with 6 tunable weights
- **Terminal Posture Control** (maintains "elbow-up" configuration)
- **Automatic Parameter Tuning** with deep optimization mode
- **Smooth Path Planning** with cubic spline interpolation
- **Circular Buffer** (200 waypoints) for continuous motion
- **Sub-millimeter Precision** (<0.5mm steady-state error)

### Control Modes
- **M (Cartesian)**: XYZ positioning with Jacobian IK + MPC tracking
- **D (Direct Joint)**: Direct joint angle control (radians) with MPC
- **W (Path)**: Multi-waypoint trajectories with smooth blending
- **Sweep**: Continuous axis scanning for workspace exploration

### Setup & Usage

**Flash Firmware:**
```bash
cd microcontroller_arm_control/firmware_teensy_qp
pio run --target upload
```

**Run GUI:**
```bash
cd ../pc_software
python arm_gui.py
```
- Connect to COM port (921600 baud)
- GUI auto-loads `mpc_params.json`
- Use XYZ control, path planning, or sweep modes

**MPC Tuning:**
```bash
python mpc_tuner.py
```
- **Modes**: Quick (5-10min) / Standard (15-25min) / Deep (1-2hr)
- **Goals**: Fast / Precise / Smooth / Ultimate
- Auto-saves to `mpc_params.json`

### Performance Metrics
- Control Rate: **815Hz**
- Position Error: **<0.5mm**
- Settling Time: **<2s**
- Zero Overshoot (with Ultimate tuning)

### Serial Commands
```
M x y z roll time          # Cartesian move
D q1 q2 q3 q4 roll        # Direct joint (radians)
W x y z roll wait         # Add waypoint
WEXEC                     # Execute path
MPCSET Qpos 150.0         # Set MPC weight
MPCGET                    # Read weights
P                         # Get position
S                         # Stop
```

---

## ⚡ Implementation #2: ESP32 MPC Hybrid

### Overview
**Balanced approach** combining Jacobian velocity control with MPC constraint projection. Runs at ~200Hz on ESP32.

### Key Features
- **200Hz Control Loop** (ESP32-WROOM @ 240MHz)
- **Jacobian Velocity Control** for smooth Cartesian motion
- **MPC Constraint Projection** for safe operation at joint limits
- **Null-space Posture Control** (elbow-up preference)
- Compatible with ESP32 GUI (`esp32_arm_control/pc_software/arm_gui.py`)

### Architecture
1. Jacobian computes desired joint velocities
2. Null-space projection optimizes posture
3. MPC ensures velocities respect constraints
4. Servo driver executes safe commands

### Setup
```bash
cd esp32_arm_control/firmware_mpc
pio run --target upload

cd ../pc_software
python arm_gui.py
```

### When to Use
- Need smooth Jacobian control + safety constraints
- ESP32 platform required
- 200Hz sufficient for application
- Don't need ultimate precision of Teensy MPC

---

## 🔧 Implementation #3: ESP32 Jacobian IK

### Overview
**Simplest embedded solution** using pure Jacobian velocity control with PID. Reliable and easy to understand.

### Key Features
- **~200Hz Control Loop** on ESP32
- **Pure Jacobian Velocity Control**
- **Per-joint PID Controllers** (tuned for smooth motion)
- **Trajectory Generation** for blended paths
- **Auto-PID Tuner** available (`auto_pid_calibrate.py`)

### Control Architecture
1. Inverse kinematics computes target joint angles
2. PIDs generate velocity commands per joint
3. Trajectory planner blends waypoints
4. No MPC overhead - simple and fast

### Setup
```bash
cd esp32_arm_control/firmware
pio run --target upload

cd ../pc_software
python arm_gui.py
```

### PID Tuning
```bash
cd esp32_arm_control/pc_software
python auto_pid_calibrate.py
```

### When to Use
- Learning embedded control systems
- Don't need MPC complexity
- Want simple, maintainable code
- Basic automation tasks

---

## 🐍 Implementation #4: Direct Python Control

### Overview
**PC-based control** without microcontroller firmware. Python directly commands servos via USB. Two variants available:

### A) Jacob IK (Jacobian-based)
**Location**: `jacob_ik/`

**Features:**
- Jacobian velocity control from PC
- Damped Least Squares (DLS) for singularity handling
- Real-time 3D visualization
- Trajectory planning
- Per-joint controllers with PID

**Setup:**
```bash
cd jacob_ik
python calibrate_servos.py  # First time setup
python motor_gui.py
```

**Pros:**
- No firmware needed
- Easy to modify and experiment
- Full Python debugging tools
- Real-time plotting

**Cons:**
- USB latency (~10-50ms)
- Lower control rate (~20-50Hz)
- PC must stay connected

### B) Analytical IK (Closed-form)
**Location**: `analytical_ik/`

**Features:**
- Closed-form inverse kinematics
- No iterative solving
- Fast and deterministic
- Simple architecture

**Setup:**
```bash
cd analytical_ik
python calibrate_servos.py  # First time setup
python motor_gui.py
```

**Pros:**
- Deterministic IK solution
- No Jacobian singularities
- Simple to understand
- Fast computation

**Cons:**
- Limited to specific arm configurations
- Less flexible than Jacobian
- USB latency limitations

### When to Use Python Control
- **Prototyping** new algorithms
- **Testing** hardware before firmware
- **Education** - understanding control theory
- **Research** - need Python ecosystem (numpy, scipy, matplotlib)
- **Quick experiments** without reflashing firmware

---

## 🛠️ Hardware Specifications

### Required Components
- **Microcontroller** (choose one):
  - **Teensy 4.1** (recommended) - 600MHz, 815Hz control
  - **ESP32-WROOM** - 240MHz, 200Hz control
  - **PC with Python** - Direct USB control
  
- **Servos**: 5x ST3215 (or 6 with wrist roll)
- **Power Supply**: 12V for servos
- **USB Cable**: Micro-USB or USB-C depending on board

### Robot Arm Specifications
- **DOF**: 5 (Base, Shoulder, Elbow, Wrist Pitch, Wrist Roll)
- **Link Lengths**:
  - L1 (Shoulder-Elbow): 133.39mm
  - L2 (Elbow-Wrist): 124.97mm
  - L3 (Wrist-Tip): 73.39mm
- **Workspace**: ~300mm radius hemisphere
- **Payload**: TBD (depends on servo torque)

### Servo Configuration & Communication

**Servos:**
- **Model**: Waveshare ST3215 Serial Bus Servos
- **Resolution**: 4096 steps/revolution (0.088°/step)
- **Communication Protocol**: Half-duplex serial (RS-485 style)
- **Baud Rate**: 1000000 bps
- **Daisy Chain**: All servos on single serial bus with unique IDs

**Serial Bus Driver Board:**
- **Required**: Waveshare Serial Bus Servo Driver Board or equivalent
- **Function**: Converts TTL serial to half-duplex servo bus
- **Direction Control**: GPIO pin switches between TX/RX
- **Connections**:
  - Microcontroller TX → Driver TX
  - Microcontroller RX → Driver RX
  - Microcontroller GPIO → Driver DIR (direction control)
  - Driver Bus → Servo chain (daisy-chained)

**STServo SDK:**
- **Protocol**: Proprietary Waveshare protocol (similar to Dynamixel)
- **Python SDK**: Included in `STServo_Python/` directory
- **C++ Drivers**: Custom implementations in firmware (`ServoDriver.h`)
- **Commands**: Position, speed, voltage, temperature, torque enable/disable

**Why Serial Bus?**
- Single communication line for all servos (not PWM)
- Position feedback built-in (closed-loop control)
- Daisy-chain reduces wiring complexity
- Real-time telemetry (position, voltage, temperature)

---

## 🔧 Installation & Setup Guide

### 1. Clone Repository
```bash
git clone https://github.com/Lishen99/ST3215-5DOF-Robotic-Arm.git
cd ST3215-5DOF-Robotic-Arm/STServo_Python
```

### 2. Python Environment
```bash
python -m venv stservo-env

# Windows
.\stservo-env\Scripts\activate

# macOS/Linux
source stservo-env/bin/activate

pip install -r requirements.txt
pip install scipy  # For smooth path interpolation (Teensy)
```

### 3A. Teensy 4.1 Setup (Recommended)
```bash
# Install PlatformIO
pip install platformio

# Flash firmware
cd microcontroller_arm_control/firmware_teensy_qp
pio run --target upload

# Run GUI
cd ../pc_software
python arm_gui.py
# Select COM port, 921600 baud, Connect

# Optional: Run MPC tuner
python mpc_tuner.py
```

### 3B. ESP32 Setup
```bash
# Choose firmware variant
cd esp32_arm_control/firmware          # Pure Jacobian
# OR
cd esp32_arm_control/firmware_mpc      # Hybrid MPC

pio run --target upload

cd ../pc_software
python arm_gui.py
```

### 3C. Python Direct Control
```bash
# Choose implementation
cd jacob_ik          # Jacobian-based
# OR
cd analytical_ik     # Analytical IK

python calibrate_servos.py    # First time: calibrate centers
python motor_gui.py           # Run controller
```

---

## 📊 Performance Comparison

| Feature | Teensy QP-MPC | ESP32 Hybrid | ESP32 Jacobian | Python Direct |
|---------|--------------|--------------|----------------|---------------|
| **Control Rate** | 815Hz | 200Hz | 200Hz | 20-50Hz |
| **Position Error** | <0.5mm | ~1-2mm | ~2-3mm | ~5-10mm |
| **Settling Time** | <2s | ~3s | ~3-5s | ~5-10s |
| **Smoothness** | Excellent | Very Good | Good | Fair |
| **Tuning Complexity** | High (auto) | Medium | Low (PID) | Low |
| **Code Complexity** | High | Medium | Low | Very Low |
| **Setup Difficulty** | Medium | Medium | Easy | Very Easy |
| **Best For** | Research, Precision | Real-time Apps | Learning | Prototyping |

---

## 🎮 Usage Examples

### Example 1: Cartesian Move (Teensy)
```bash
# Connect GUI → arm_gui.py
# Enter coordinates: X=200, Y=100, Z=150
# Click "Move To"
```

**Serial Command:**
```
M 200 100 150 0 2.0    # Move to (200,100,150) in 2s
```

### Example 2: Path Execution with Loop
```bash
# Add waypoints in GUI
# Select "Smooth" mode
# Enable "Loop"
# Click "Execute Path"
```

### Example 3: MPC Tuning
```bash
cd microcontroller_arm_control/pc_software
python mpc_tuner.py

# Select "Deep" mode
# Choose "Ultimate" goal
# Click "Start Tuning"
# Wait 1-2 hours
# Results auto-saved to mpc_params.json
```

### Example 4: Python Jacobian Control
```bash
cd jacob_ik
python motor_gui.py

# Connect to COM port
# Scan motors
# Use "Cartesian Control" panel
# Enter X, Y, Z coordinates
# Click "Move"
```

---

## 🔍 Key Algorithms

### QP-MPC (Teensy Implementation)

**Cost Function:**
```
J = ||q - q_ref||²_Qpos + ||q̇ - q̇_ref||²_Qvel + 
    ||q_final - q_target||²_Qfpos + ||q̇_final||²_Qfvel +
    ||u||²_R + ||Δu||²_Rdelta
```

**Parameters (Ultimate Tuning):**
- `Qpos`: Position tracking weight
- `Qvel`: Velocity tracking weight
- `Qfpos`: Terminal position (posture control)
- `Qfvel`: Terminal velocity (smooth stop)
- `R`: Control effort penalty
- `Rdelta`: Jerk penalty (smoothness)

**Terminal Posture Control:**
- Optimizes final joint configuration
- Maintains "elbow-up" for better workspace reach
- Prevents awkward singularity-prone poses

### Jacobian Velocity Control (All Implementations)

**Damped Least Squares (DLS):**
```python
J_inv = J.T @ inv(J @ J.T + λ²I)
q̇ = J_inv @ ẋ_desired
```

**Adaptive Damping:**
```python
λ = λ_base * (1 + exp(-k * manipulability))
```

**Null-space Projection:**
```python
N = I - J_inv @ J
q̇_posture = N @ (q_neutral - q_current)
q̇_final = q̇_task + α * q̇_posture
```

---

## 🐛 Troubleshooting

### Teensy Not Connecting
- Check COM port in Device Manager
- Verify 921600 baud rate
- Press Teensy button to enter bootloader
- Reflash firmware: `pio run --target upload`

### Motors Not Responding
```bash
cd waveshare_diagnostic
python scan_baud.py        # Find correct baud rate
python test_motor1.py      # Test individual motor
python recover_servos.py   # Factory reset if needed
```

### Poor Motion Quality
1. **Teensy**: Run MPC tuner (`mpc_tuner.py`)
2. **ESP32 Jacobian**: Run PID tuner (`auto_pid_calibrate.py`)
3. **Python**: Adjust damping in `kinematics.py`

### Arm Shaking/Oscillating
- **Teensy**: Increase `Rdelta` (jerk penalty) via `MPCSET Rdelta 0.5`
- **ESP32**: Lower PID D-gains
- **All**: Reduce speed multiplier

### Path Not Smooth
- Enable "Smooth" mode in path planning
- Increase blend radius
- Use more intermediate waypoints
- Check J1 wrapping logic for base rotation

---

## 🏆 Key Innovations

### 1. Terminal Posture MPC
MPC optimizes not just trajectory tracking but also **terminal joint configuration**. This maintains favorable "elbow-up" poses and avoids singularities automatically.

### 2. Intelligent J1 Wrapping
When base rotation exceeds ±180°, system automatically inserts intermediate waypoints to prevent infinite spinning through cable wraps.

### 3. Circular Buffer Management
200-waypoint circular buffer with intelligent top-up ensures smooth motion during long paths without USB latency affecting real-time control.

### 4. Velocity-Based Tuning
MPC tuner penalizes slow movements (<17°/s) that cause backlash jitter, ensuring confident motion while still allowing settling.

### 5. High-Resolution Error Tracking
Uses raw servo positions (4096 steps/rev) instead of floating-point angles for tuning - achieves higher precision in optimization.

---

## 📚 Additional Resources

### Calibration Tools (`waveshare_diagnostic/`)
- `test_motor1.py` - Test individual motor
- `scan_baud.py` - Auto-detect baud rate
- `full_scan.py` - Scan all motor IDs
- `recover_servos.py` - Factory reset servos

### Mechanical Design (`Arm_Design/`)
- SolidWorks assembly and parts
- STL files for 3D printing
- Bearing specifications
- Bill of materials in assembly

### Configuration Files
- `servo_limits.json` - Joint limits, centers, PID gains
- `arm_config.json` - Link lengths, workspace bounds
- `mpc_params.json` - Optimized MPC weights

---

## 🚧 Known Issues & Future Work

### Current Limitations
1. **Motor 6 (Roll)**: Excluded from MPC tuning for safety
2. **Z-Limit**: Currently -150mm, could extend with better singularity handling
3. **Python Control**: USB latency limits performance

### Planned Improvements
- [ ] Adaptive MPC weights based on workspace region
- [ ] Vision-based feedback control
- [ ] Force/torque sensing
- [ ] Multi-arm coordination
- [ ] ROS integration
- [ ] Machine learning for parameter optimization

---

## 📄 License

This project is licensed under the [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0) License](https://creativecommons.org/licenses/by-nc-sa/4.0/).

You are free to share and adapt the material for non-commercial purposes, provided appropriate credit is given and any adaptations are shared under the same license.

---

## 🙏 Credits

- **Hardware/SDK**: Waveshare ST3215 servo motors and STServo SDK
- **MPC Library**: Custom QP-MPC implementation based on TinyMPC concepts
- **Development**: Lishen99
- **Control Theory**: Classical Jacobian IK, MPC optimization, DLS

---

## 📞 Contact & Support

- **GitHub Issues**: [Report bugs or request features](https://github.com/Lishen99/ST3215-5DOF-Robotic-Arm/issues)
- **Discussions**: Share your builds and ask questions
- **Pull Requests**: Contributions welcome!

---

**⚡ Choose Your Implementation:**
- 🚀 **Research/Precision** → Teensy QP-MPC (815Hz)
- ⚡ **Real-time Apps** → ESP32 MPC Hybrid (200Hz)
- 🔧 **Learning/Automation** → ESP32 Jacobian IK
- 🐍 **Prototyping/Testing** → Python Direct Control

**Happy Building! 🤖**
