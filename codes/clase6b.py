import control as ct
import numpy as np
import matplotlib.pyplot as plt

m, k, b = 2, 0.1, 0.2

A = np.array([[0, 0, 1, 0, 0],
              [0, 0, 0, 1, 0],
              [0, -1.7019, 0, 1, 0],
              [0, -13.3301, 0, 1, 0],
              [1, 0, 0, 0, 0]])
B = np.array([[0],
             [0],
             [18.2478],
             [21.1299],
             [0]])
C = np.array([[1, 0, 0, 0, 0],
              [0, 1, 0, 0, 0],
              [0, 0, 1, 0, 0],
              [0, 0, 0, 1, 0],
              [0, 0, 0, 0, 1]])
D = np.array([[0], [0], [0], [0], [0]])

sys = ct.ss(A, B, C, D)

Q = np.diag([20, 5, 1, 5, 8])
R = np.array([[0.5]])

K, s, e = ct.lqr(A, B, Q, R)
print(K)

# Controlled System
A_cl = A - B@K
sys_cl = ct.ss(A_cl, B, C, D)
print(A_cl)


x0 = np.array([[0.0],
               [0.0],
               [0.0],
               [0.0],
               [0.0]])
t = np.linspace(0, 10, 500)

# Create step signal: step amplitude and step time
step_amplitude = 1.0
step_time = 2.0  # Step occurs at t=0
u_step_1d = step_amplitude * (t >= step_time)  # 1D array for plotting

# Reshape input for forced_response: shape should be (n_inputs, n_timepoints) or (n_timepoints,)
# For single input system, we can use 1D array or reshape to (1, n_timepoints)
u_step = u_step_1d.reshape(1, -1)  # Shape: (1, n_timepoints)

# Use the step signal as input to the system
t, y = ct.forced_response(sys_cl, T=t, U=u_step, X0=x0)

# forced_response returns y with shape (n_outputs, n_timepoints) for MIMO
# Convert to numpy array and ensure proper shape
y = np.array(y)
# Squeeze any singleton dimensions
y = np.squeeze(y)

# If squeezed to 1D (single output), reshape
if y.ndim == 1:
    y = y.reshape(1, -1)

# Create figure with two subplots: input and outputs
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Plot the step input signal
ax1.plot(t, u_step_1d, 'r-', linewidth=2, label='Step Input')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Input')
ax1.set_title('Step Signal Input')
ax1.legend()
ax1.grid(True)

# Plot each output
for i in range(y.shape[0]):
    output_data = np.squeeze(y[i])  # Ensure 1D array for plotting
    ax2.plot(t, output_data, label=f'State{i+1}')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('States')
ax2.set_title('LQR Step Response')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()
