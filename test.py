import numpy as np
import matplotlib.pyplot as plt

# Define constants and parameters
F = 96485.33212  # Faraday's constant (C/mol)
R = 8.314  # gas constant (J/mol K)
T = 298.15  # temperature (K)
ν = 0.5  # Nernst potential (V)
k = 1.0  # mass transfer coefficient
B = 0.1  # Butler-Volmer constant
A = 0.5  # Tafel slope constant
α = 0.5  # transfer coefficient
i0 = 0.01  # exchange current density (A/m^2)
R_m = 0.1  # membrane resistance (Ohm)

# Define the function to calculate V_out
def calculate_v_out(i, ν, k, B, A, α, F, i0, R, T, R_m):
    return ν - (i / k) * (B + A * np.exp((1 - α) * F * (i / i0) / (R * T))) - R_m * i

# Create a range of current densities
current_densities = np.linspace(0, 2, 1000)

# Calculate voltage outputs for each current density
voltage_outputs = [calculate_v_out(i, ν, k, B, A, α, F, i0, R, T, R_m) for i in current_densities]

# Plot the polarization curve
plt.plot(current_densities, voltage_outputs)
plt.xlabel('Current Density (A/m^2)')
plt.ylabel('Voltage Output (V)')
plt.title('Polarization Curve')
plt.grid()
plt.show()
