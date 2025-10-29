# PythonVehicleSimulator Package Analysis

## 📦 Package Structure

```
/home/hyo/PythonVehicleSimulator/
├── src/python_vehicle_simulator/
│   ├── vehicles/
│   │   ├── otter.py          # Otter USV implementation
│   │   ├── DSRV.py           # Deep submergence rescue vehicle
│   │   ├── frigate.py        # Frigate ship
│   │   ├── remus100.py       # REMUS 100 AUV
│   │   └── ...               # Other vehicles
│   ├── lib/
│   │   ├── control.py        # Control algorithms (PID, pole placement)
│   │   ├── gnc.py           # Guidance, Navigation, Control functions
│   │   ├── guidance.py      # Guidance algorithms
│   │   ├── mainLoop.py      # Main simulation loop
│   │   └── models.py        # Mathematical models
│   └── main.py              # Main program entry point
```

## 🚢 Otter USV Analysis

### **Physical Specifications**
- **Length**: 2.0 m
- **Beam**: 1.08 m  
- **Mass**: 55.0 kg (hull) + 25.0 kg (payload) = 80.0 kg total
- **Draft**: Calculated from displacement
- **Max Speed**: 6 knots (3.09 m/s)
- **Propellers**: 2 propellers with differential thrust

### **State Vector (6-DOF)**
```python
eta = [x, y, z, phi, theta, psi]  # Position and attitude
nu = [u, v, w, p, q, r]          # Linear and angular velocities
```

Where:
- `x, y, z`: Position in North-East-Down (NED) frame
- `phi, theta, psi`: Roll, pitch, yaw angles
- `u, v, w`: Surge, sway, heave velocities
- `p, q, r`: Roll, pitch, yaw rates

### **Control Inputs**
```python
u_control = [n1, n2]  # Left and right propeller speeds (rad/s)
```

### **Key Methods**

#### 1. **Dynamics Integration**
```python
def dynamics(self, eta, nu, u_actual, u_control, sampleTime):
    """
    Integrates Otter USV equations of motion using Euler's method
    Returns: [nu, u_actual] - updated velocities and actual control inputs
    """
```

#### 2. **Control Systems**
```python
# Heading autopilot (PID controller)
def headingAutopilot(self, eta, nu, sampleTime):
    """
    PID controller for automatic heading control
    Uses pole placement for gain calculation
    """

# Velocity control (surge + yaw rate)
def velocityControl(self, nu, u_ref, r_ref, sampleTime):
    """
    Velocity controller for surge (u) and yaw rate (r)
    Uses pole placement with 2nd-order reference model
    """

# Step input control
def stepInput(self, t):
    """
    Generates propeller step inputs for testing
    """
```

#### 3. **Control Allocation**
```python
def controlAllocation(self, tau_X, tau_N):
    """
    Converts desired forces to propeller commands
    tau_X: surge force
    tau_N: yaw moment
    Returns: [n1, n2] - propeller speeds
    """
```

## 🔧 Control Architecture

### **1. Heading Autopilot**
- **Input**: Desired heading angle (psi_d)
- **Controller**: PID with pole placement
- **Output**: Propeller commands [n1, n2]
- **Features**: 
  - 3rd-order reference model for smooth trajectories
  - Integral windup protection
  - Saturation limits

### **2. Velocity Control**
- **Input**: Desired surge velocity (u_ref) and yaw rate (r_ref)
- **Controller**: PI control with pole placement
- **Output**: Propeller commands [n1, n2]
- **Features**:
  - 2nd-order reference model
  - Feedforward + feedback control
  - Automatic disturbance rejection

### **3. Step Input Control**
- **Input**: Time-based step commands
- **Output**: Propeller commands [n1, n2]
- **Use**: Testing and validation

## 🌊 Hydrodynamic Model

### **Mass Matrix (M)**
- **Rigid Body Mass**: 80.0 kg total mass
- **Added Mass**: Hydrodynamic added mass coefficients
- **Inertia**: 6x6 mass matrix including added mass

### **Damping Matrix (D)**
- **Linear Damping**: Velocity-dependent damping
- **Nonlinear Damping**: Quadratic yaw damping
- **Cross-flow Drag**: Additional drag forces

### **Hydrostatic Forces (G)**
- **Buoyancy**: Water displacement forces
- **Metacentric Height**: Stability calculations
- **Spring Stiffness**: Restoring forces

### **Coriolis and Centripetal Forces (C)**
- **Rigid Body**: CRB matrix
- **Added Mass**: CA matrix
- **Current Effects**: Relative velocity calculations

## 🎯 Integration with ir-sim

### **State Vector Mapping**
```python
# ir-sim state: [x, y, psi, u, v, r, n1, n2]
# PythonVehicleSimulator state: [x, y, z, phi, theta, psi, u, v, w, p, q, r]

# Mapping:
x_irsim = x_pvs
y_irsim = y_pvs  
psi_irsim = psi_pvs
u_irsim = u_pvs
v_irsim = v_pvs
r_irsim = r_pvs
n1_irsim = n1_pvs
n2_irsim = n2_pvs
```

### **Control Interface**
```python
# ir-sim control: [n1, n2] (propeller speeds)
# PythonVehicleSimulator control: [n1, n2] (same format)

# Direct mapping possible
```

## 🚀 Usage Examples

### **1. Basic Otter Simulation**
```python
from python_vehicle_simulator.vehicles.otter import otter

# Create Otter with heading autopilot
otter_usv = otter('headingAutopilot', psi_d=100.0, V_c=0.3, beta_c=-30.0, tau_X=200.0)

# Simulate
for i in range(N):
    u_control = otter_usv.headingAutopilot(eta, nu, sampleTime)
    nu, u_actual = otter_usv.dynamics(eta, nu, u_actual, u_control, sampleTime)
    eta = attitudeEuler(eta, nu, sampleTime)
```

### **2. Velocity Control**
```python
# Create Otter with velocity control
otter_usv = otter('velocityControl', u_ref=2.0, r_ref=0.1)

# Control loop
u_control = otter_usv.velocityControl(nu, u_ref=2.0, r_ref=0.1, sampleTime)
```

### **3. Step Input Testing**
```python
# Create Otter with step inputs
otter_usv = otter('stepInput')

# Generate step commands
u_control = otter_usv.stepInput(t)
```

## 🔗 Integration with COLREGs

### **State Extraction for COLREGs**
```python
def get_otter_state_for_colregs(otter_usv, eta, nu):
    """
    Extract state information for COLREGs analysis
    """
    # Position
    x, y = eta[0], eta[1]
    
    # Heading (convert to degrees)
    psi_deg = np.degrees(eta[5])
    
    # Speed (from surge and sway velocities)
    u, v = nu[0], nu[1]
    speed = np.sqrt(u**2 + v**2)
    
    # Velocity vector
    velocity = (u, v)
    
    return {
        'position': (x, y),
        'heading': psi_deg,
        'speed': speed,
        'velocity': velocity
    }
```

### **Control Integration**
```python
def colregs_controlled_otter(otter_usv, eta, nu, colregs_action, sampleTime):
    """
    Apply COLREGs action to Otter control
    """
    if colregs_action == "turn_starboard":
        # Increase right propeller, decrease left
        u_control = [n1-10, n2+10]
    elif colregs_action == "turn_port":
        # Increase left propeller, decrease right  
        u_control = [n1+10, n2-10]
    elif colregs_action == "slow_down":
        # Reduce both propellers
        u_control = [n1*0.5, n2*0.5]
    else:
        # Maintain current heading
        u_control = otter_usv.headingAutopilot(eta, nu, sampleTime)
    
    return u_control
```

## 📊 Key Advantages

1. **Realistic Dynamics**: Full 6-DOF hydrodynamics with added mass
2. **Multiple Control Modes**: Heading, velocity, and step input control
3. **Maritime Physics**: Proper buoyancy, damping, and stability
4. **Control Allocation**: Realistic propeller control mapping
5. **Current Effects**: Ocean current modeling
6. **Saturation Limits**: Physical propeller constraints

## 🎯 Perfect for COLREGs Integration

The PythonVehicleSimulator Otter USV is ideal for COLREGs integration because:

1. **Realistic Maritime Dynamics**: Proper ship hydrodynamics
2. **Control Flexibility**: Multiple control modes for different COLREGs actions
3. **State Compatibility**: Easy mapping to ir-sim state vectors
4. **Physical Constraints**: Realistic propeller and speed limits
5. **Current Modeling**: Ocean current effects for realistic scenarios

This makes it perfect for training DRL agents with realistic maritime physics and COLREGs compliance! 🚢⚓🤖
