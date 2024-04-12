import numpy as np
import scipy
import matplotlib.pyplot as plt

A = np.matrix([[0, 1],
              [-0.01, -0.1]])

x0 = np.matrix([[-10], [20]])

t  = np.linspace(0, 200, 201)

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
plt.title("Theta vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Theta")
plt.show()

plt.plot(t, theta_dot)
plt.title("Theta dot vs Time")
plt.xlabel("Time (s)")
plt.ylabel("Theta dot")
plt.show()