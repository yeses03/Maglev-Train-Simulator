# Maglev-Train-Simulator

## 2D Multi-Magnet Electromagnetic Suspension (EMS) System  
With PID Control, Aerodynamics, Thermal Modeling, and Propulsion

---

## 📌 Project Overview

This project is a high-fidelity Python simulation of an Electromagnetic Suspension (EMS) Maglev Train system.

The simulator models:

- 2D motion (vertical levitation + horizontal propulsion)
- 4 independent electromagnets
- PID-controlled levitation
- Aerodynamic drag
- Linear motor propulsion
- Electrical power consumption
- Coil heating and thermal shutdown
- Stability and performance metrics

The goal is to simulate real-world engineering constraints encountered in high-speed maglev systems.

---

## ⚙️ Features

### 🔹 Electromagnetic Levitation (EMS)

Magnetic force model:

F = (k · |u|) / d²

Where:
- k = magnetic constant  
- u = control signal (PID output)  
- d = air gap distance  

---

### 🔹 PID Control System

Each of the 4 magnets uses independent PID control:

u(t) = Kp·e(t) + Ki∫e(t)dt + Kd·de/dt

This stabilizes the train at the target levitation height.

---

### 🔹 Aerodynamic Drag

Fd = ½ ρ Cd A v²

Where:
- ρ = air density  
- Cd = drag coefficient  
- A = frontal area  

---

### 🔹 Linear Motor Propulsion

F = B I L

Where:
- B = magnetic flux density  
- I = propulsion current  
- L = conductor length  

---

### 🔹 Thermal Model

Power loss in coils:

P = I²R

Temperature rise:

ΔT = Q / (m c)

The system automatically shuts down if the coil temperature exceeds the maximum safe limit.

---

## 🛠 Installation

Install dependencies:

```bash
pip install numpy matplotlib
