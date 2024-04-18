import numpy as np
import scipy
import matplotlib.pyplot as plt

A = np.matrix([[0, 1],
              [-2, -1]])

x0 = np.matrix([[5], [-5]])

t  = np.linspace(0, 10, 200)

eigenvalues = np.linalg.eigvals(A)

print("Eigenvalues:", eigenvalues)

real_part = eigenvalues.real
imaginary_part = eigenvalues.imag

omega_n = np.sqrt(real_part**2 + imaginary_part**2)
zeta = -real_part / omega_n

eigenvectors = np.linalg.eig(A)[1]
print("EigenVectors:", eigenvectors)

print("Natural frequency:", omega_n)
print("Damping ratio:", zeta)

theta = []
theta_dot=[]

for x in t:
    tmp_output_in = scipy.linalg.expm(A*x)
    tmp_output = tmp_output_in* x0
    theta.append(tmp_output.item(0))
    theta_dot.append(tmp_output.item(1))

plt.plot(t, theta)
plt.title("Psi vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Psi")
plt.show()

plt.plot(t, theta_dot)
plt.title("Psi dot vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Psi dot")
plt.show()


omega = omega_n * np.sqrt(1 - zeta**2)

T = 2 * np.pi / omega

print("Period of oscillation:", T)