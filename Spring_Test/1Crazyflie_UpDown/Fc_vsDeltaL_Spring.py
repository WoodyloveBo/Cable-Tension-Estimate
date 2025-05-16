import matplotlib.pyplot as plt

# Data
F_c = [10.1, 10.96, 12.08, 12.428, 13.67, 15.5, 16.08, 17.34, 17.7, 19.26, 20.48]
delta_L = [0.043, 0.053, 0.065, 0.074, 0.083, 0.124, 0.133, 0.14, 0.147, 0.167, 0.18]

# Create the plot
plt.figure()
plt.plot(F_c, delta_L, marker='o')
plt.xlabel('Weight (g)')
plt.ylabel('ΔL (m)')
plt.title('ΔL vs Weight')
plt.grid(True)
plt.show()