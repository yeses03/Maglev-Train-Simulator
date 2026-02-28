# Maglev Train Simulation

## Overview

This project is a Python-based simulation of a maglev (magnetic levitation) train system.

The simulator models:

- Vertical levitation using electromagnets
- PID control for height stabilization
- Horizontal propulsion using a linear motor model
- Aerodynamic drag
- Coil heating and thermal shutdown
- Energy consumption tracking
- Performance metrics

The goal is to simulate a realistic engineering-level maglev system rather than a simple animation.

---

## Features

### Electromagnetic Levitation

Magnetic lift force depends on:

- Magnetic constant
- Control signal from PID controller
- Distance between train and track

The system continuously adjusts magnetic force to maintain a target levitation height.

---

### PID Control

Each electromagnet uses a PID controller with:

- Kp (Proportional gain)
- Ki (Integral gain)
- Kd (Derivative gain)

These values control stability, overshoot, and settling time.

---

### Aerodynamic Drag

Drag increases with velocity and limits maximum speed.

---

### Linear Motor Propulsion

Forward motion is generated using a simplified electromagnetic propulsion model.

---

### Thermal Model

Electrical losses generate heat in the coils.

If temperature exceeds the maximum allowed value, the system shuts down automatically.

---

## Installation

Install required libraries:

pip install numpy matplotlib

---

## How To Run

Run the simulation:

python maglev_simulation.py


The program will:

1. Run the simulation
2. Print performance metrics
3. Display graphs for:
   - Levitation height
   - Forward velocity
   - Coil temperature
   - Energy consumption

---

## Configuration

All system parameters are defined in the CONFIG dictionary at the top of the Python file.

You can modify:

- mass
- target_height
- Kp, Ki, Kd
- propulsion current
- drag coefficient
- coil resistance
- maximum temperature

---

## Performance Metrics

The simulation calculates:

- Final forward velocity
- Steady-state height
- Overshoot percentage
- Total energy used
- Final coil temperature
- Shutdown status

---

## Future Improvements

Possible extensions:

- 3D simulation
- Pitch and roll modeling
- GUI interface
- Data export to CSV
- Real-world train comparison

---

## Project Type

Electromechanical Control Systems Simulation  
Advanced Undergraduate Level

---

## License

This project is intended for academic and educational use.
