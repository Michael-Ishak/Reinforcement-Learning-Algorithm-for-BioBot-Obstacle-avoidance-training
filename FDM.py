import numpy as np
import matplotlib.pyplot as plt

# Constants
alpha = 0.5
F = 96485  # Faraday constant, C/mol
R = 8.314  # Gas constant, J/(mol*K)
T = 298  # Temperature, K
E_eq = 1.23  # Equilibrium potential, V
i_0 = 1e-4  # Exchange current density, A/m^2

# Newton-Raphson method to solve for E
def newton_raphson(E_init, i, tol=1e-6, max_iter=100):
    E = E_init
    for _ in range(max_iter):
        i_current = i_0 * (np.exp(alpha * F * (E - E_eq) / (R * T)) - np.exp(-(1 - alpha) * F * (E - E_eq) / (R * T)))
        f_derivative = i_0 * (alpha * F * np.exp(alpha * F * (E - E_eq) / (R * T)) / (R * T) + (1 - alpha) * F * np.exp(-(1 - alpha) * F * (E - E_eq) / (R * T)) / (R * T))
        E_new = E - (i_current - i) / f_derivative

        if abs(E_new - E) < tol:
            break

        E = E_new

    return E

# Generate current density values
i_values = np.linspace(-0.2, 0.2, 100)

# Calculate potential difference for each current density value
E_values = np.array([newton_raphson(E_eq, i) for i in i_values])

# Plot E vs i (polarization curve)
plt.plot(E_values, i_values)
plt.xlabel('Potential Difference (V)')
plt.ylabel('Current Density (A/m^2)')
plt.title('Polarization Curve')
plt.grid(True)
plt.show()

# Plot E vs i (power curve)
plt.plot(E_values, E_values * i_values)
plt.xlabel('Potential Difference (V)')
plt.ylabel('Power Density (W/m^2)')
plt.title('Power Curve')
plt.grid(True)
plt.show()
