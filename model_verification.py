
import numpy as np

from src.model.state_space import (
    linearise_continuous,
    controllability_matrix,
    observability_matrix
)

# Reference operating point
theta_ref = 0.0
v_ref = 0.10

x_ref = 0.0
y_ref = 0.0

# ArUco world position
landmark_x = 1.8
landmark_y = 0.0

# State-space model
A, B = linearise_continuous(
    theta_ref,
    v_ref
)

# Controllability
Ctrb = controllability_matrix(A, B)

# Camera range-bearing observation Jacobian
dx = landmark_x - x_ref
dy = landmark_y - y_ref

q = dx**2 + dy**2
r = np.sqrt(q)

C = np.array([
    [-dx / r, -dy / r, 0.0],
    [dy / q, -dx / q, -1.0]
])

# Observability
Obsv = observability_matrix(A, C)

print("A matrix:\n", A)
print("B matrix:\n", B)
print("C matrix:\n", C)

print("\nControllability rank:",
      np.linalg.matrix_rank(Ctrb))

print("Observability rank:",
      np.linalg.matrix_rank(Obsv))

print("Number of states:", A.shape[0])
