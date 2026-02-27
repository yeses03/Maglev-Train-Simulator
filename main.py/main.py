import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# SYSTEM CONFIGURATION (YOU MODIFY THIS)
# ==========================================

CONFIG = {
    "mass": 500,  # kg
    "gravity": 9.81,
    "magnetic_constant": 2e-5,
    "target_height": 0.015,  # meters

    # Aerodynamics
    "air_density": 1.225,
    "drag_coefficient": 0.15,
    "frontal_area": 1.2,

    # Linear motor
    "B": 0.8,
    "I_propulsion": 200,
    "L": 1.5,

    # Coil thermal properties
    "coil_resistance": 0.5,
    "coil_mass": 50,
    "specific_heat": 385,
    "max_temp": 120,

    # PID gains
    "Kp": 2000,
    "Ki": 500,
    "Kd": 1000,

    "simulation_time": 20,
    "dt": 0.01
}


# ==========================================
# MAGLEV SYSTEM CLASS
# ==========================================

class MaglevTrain:
    def __init__(self, config):
        self.c = config

        self.x = 0
        self.vx = 0

        self.y = 0.01
        self.vy = 0

        self.temp = 25

        self.integral = np.zeros(4)
        self.prev_error = np.zeros(4)

        self.energy_used = 0
        self.shutdown = False

    def magnetic_force(self, distance, control_signal):
        distance = max(distance, 0.001)
        return (self.c["magnetic_constant"] * abs(control_signal)) / (distance ** 2)

    def drag_force(self):
        return 0.5 * self.c["air_density"] * self.c["drag_coefficient"] * \
            self.c["frontal_area"] * self.vx ** 2

    def propulsion_force(self):
        return self.c["B"] * self.c["I_propulsion"] * self.c["L"]

    def update_pid(self, i):
        error = self.c["target_height"] - self.y
        self.integral[i] += error * self.c["dt"]
        derivative = (error - self.prev_error[i]) / self.c["dt"]
        self.prev_error[i] = error

        return (self.c["Kp"] * error +
                self.c["Ki"] * self.integral[i] +
                self.c["Kd"] * derivative)

    def update(self):
        if self.shutdown:
            return

        total_mag_force = 0

        for i in range(4):
            control = self.update_pid(i)
            F = self.magnetic_force(self.y, control)
            total_mag_force += F

            power_loss = (control ** 2) * self.c["coil_resistance"]
            heat = power_loss * self.c["dt"]
            delta_T = heat / (self.c["coil_mass"] * self.c["specific_heat"])
            self.temp += delta_T

            self.energy_used += power_loss * self.c["dt"]

        # Vertical motion
        F_net_y = total_mag_force - self.c["mass"] * self.c["gravity"]
        ay = F_net_y / self.c["mass"]
        self.vy += ay * self.c["dt"]
        self.y += self.vy * self.c["dt"]

        if self.y < 0:
            self.y = 0
            self.vy = 0

        # Horizontal motion
        F_prop = self.propulsion_force()
        F_drag = self.drag_force()

        ax = (F_prop - F_drag) / self.c["mass"]
        self.vx += ax * self.c["dt"]
        self.x += self.vx * self.c["dt"]

        if self.temp > self.c["max_temp"]:
            self.shutdown = True

    def stability_margin(self, total_mag_force):
        return total_mag_force / (self.c["mass"] * self.c["gravity"])


# ==========================================
# SIMULATION LOOP
# ==========================================

train = MaglevTrain(CONFIG)

t_values = []
height_values = []
velocity_values = []
temp_values = []
energy_values = []

t = 0

while t < CONFIG["simulation_time"]:
    train.update()

    t_values.append(t)
    height_values.append(train.y)
    velocity_values.append(train.vx)
    temp_values.append(train.temp)
    energy_values.append(train.energy_used)

    t += CONFIG["dt"]

# ==========================================
# PERFORMANCE METRICS
# ==========================================

steady_state_height = np.mean(height_values[-100:])
overshoot = (max(height_values) - CONFIG["target_height"]) / CONFIG["target_height"] * 100

print("==== PERFORMANCE METRICS ====")
print("Final Speed (m/s):", round(train.vx, 2))
print("Steady State Height (m):", round(steady_state_height, 4))
print("Overshoot (%):", round(overshoot, 2))
print("Final Temperature (°C):", round(train.temp, 2))
print("Energy Used (J):", round(train.energy_used, 2))
print("System Shutdown:", train.shutdown)

# ==========================================
# PLOTTING
# ==========================================

plt.figure(figsize=(12, 8))

plt.subplot(2, 2, 1)
plt.plot(t_values, height_values)
plt.title("Levitation Height")

plt.subplot(2, 2, 2)
plt.plot(t_values, velocity_values)
plt.title("Forward Velocity")

plt.subplot(2, 2, 3)
plt.plot(t_values, temp_values)
plt.title("Coil Temperature")

plt.subplot(2, 2, 4)
plt.plot(t_values, energy_values)
plt.title("Energy Consumption")

plt.tight_layout()
plt.show()