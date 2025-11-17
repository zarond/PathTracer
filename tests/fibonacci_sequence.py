import math
import numpy as np
import matplotlib.pyplot as plt

GOLDEN_RATIO = (1 + 5 ** 0.5) / 2 # 1.618034
constants = [(1 + 5 ** 0.5) / 2,
             2**0.5,
             3**0.5,
             7**0.5,
             11**0.5,
             13**0.5,
             17**0.5]

# Number of samples
N = 256

def fibonacci1D(i):
    return math.fmod((i + 1.0) * GOLDEN_RATIO, 1.0)

def fibonacci1D_d(i, d):
    return math.fmod((i + 1.0) * constants[d], 1.0)

def fibonacci2D(i, inv_nbSamples):
    i_f = float(i)
    return [
        math.fmod((i_f + 0.5) * inv_nbSamples, 1.0),
        fibonacci1D(i_f)
    ]

def fibonacci2D_dimensions(i, d):
    i_f = float(i)
    return [
        fibonacci1D_d(i_f, d),
        fibonacci1D_d(i_f, d + 1)
    ]

def fibonacci_points(N_s, offset = 0):
    X = np.empty((N_s, 2))
    N_inv = 1.0 / N_s
    for i in range(N_s):
        v = fibonacci2D(i + offset, N_inv)
        X[i] = v
    return X

def fibonacci_points_dimensions(N_s, d):
    X = np.empty((N_s, 2))
    for i in range(N_s):
        v = fibonacci2D_dimensions(i, d)
        X[i] = v
    return X

samples = fibonacci_points(N)
#samples2 = fibonacci_points(N, 2*N)
#samples3 = fibonacci_points_dimensions(N, 3)

# Plot
plt.figure(figsize=(6, 6))
plt.scatter(samples[:, 0], samples[:, 1], color='blue', marker='o')
#plt.scatter(samples2[:, 0], samples2[:, 1], color='red', marker='x')
#plt.scatter(samples3[:, 0], samples3[:, 1], color='red', marker='o')

plt.title(f"Fibonacci Low-Discrepancy Sequence (N={N})")
plt.xlabel("x")
plt.ylabel("y")
plt.axis("square")
plt.xlim(0.0, 1.0)
plt.ylim(0.0, 1.0)
plt.show()
