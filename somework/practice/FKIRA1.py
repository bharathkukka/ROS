import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to create the DH transformation matrix
def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward kinematics function using your DH parameters
def forward_kinematics(theta1, theta2, theta3, theta4):
    # DH parameters based on your configuration
    a = [0, 0, 250, 150]
    alpha = [0, np.pi/2, 0, -np.pi/2]
    d = [50, 25, 0, 0]
    theta = [theta1, theta2, theta3, theta4]

    # Calculate transformation matrices based on the DH parameters
    T01 = dh_transform(a[0], alpha[0], d[0], theta[0])
    T12 = dh_transform(a[1], alpha[1], d[1], theta[1])
    T23 = dh_transform(a[2], alpha[2], d[2], theta[2])
    T34 = dh_transform(a[3], alpha[3], d[3], theta[3])

    # Multiply the matrices to get the final transformation matrix
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.dot(T03, T34)

    # Extract the end-effector position (x, y, z)
    x = T04[0, 3]
    y = T04[1, 3]
    z = T04[2, 3]

    return x, y, z

# Visualize the robot's workspace
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Define ranges for each joint angle
theta1_range = np.linspace(0, 2*np.pi, 10)  # Joint 1 can rotate 360 degrees
theta2_range = np.linspace(-np.pi/2, np.pi/2, 10)  # Joint 2 (up and down motion)
theta3_range = np.linspace(-np.pi/2, np.pi/2, 10)  # Joint 3 (same range as Joint 2)
theta4_range = np.linspace(-np.pi/2, np.pi/2, 5)  # Joint 4 (servo, limited range)

for theta1 in theta1_range:
    for theta2 in theta2_range:
        for theta3 in theta3_range:
            for theta4 in theta4_range:
                x, y, z = forward_kinematics(theta1, theta2, theta3, theta4)
                ax.scatter(x, y, z, c='blue', marker='o')

# Example pick and place points
X_pick, Y_pick, Z_pick = 50, 50, 0  # Example pick coordinates
X_place, Y_place, Z_place = 100, 100, 0  # Example place coordinates

ax.scatter(X_pick, Y_pick, Z_pick, c='red', marker='^', label="Pick Location")
ax.scatter(X_place, Y_place, Z_place, c='green', marker='s', label="Place Location")

plt.title("Robot Workspace with Pick and Place Locations")
plt.legend()
plt.show()
