import numpy as np
import matplotlib.pyplot as plt

# Robot Link Parameters (based on your setup)
a1 = 0.2
a2 = 0.15
a3 = 0.08
d1 = 0.06
d2 = 0.06
d3 = 0.06
d4 = 0.03

# Forward Kinematics using non-standard DH
def dh_transform(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
        [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha),  np.cos(alpha),  np.cos(alpha)*d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(q):
    DH_params = [
        [0,    np.pi/2, d1, q[0]],
        [a1,   0,       0,  q[1]],
        [a2,   0,       0,  q[2]],
        [a3,   np.pi/2, d2, q[3]],
        [0,   -np.pi/2, d3, q[4]],
        [0,    0,       d4, q[5]]
    ]

    T = np.eye(4)
    for param in DH_params:
        T = T @ dh_transform(*param)
    return T[:3, 3]  # Return only the position

# Generate random samples in joint space
def generate_workspace(num_samples=5000):
    positions = []
    for _ in range(num_samples):
        q = np.random.uniform(-np.pi, np.pi, 6)
        pos = forward_kinematics(q)
        positions.append(pos)
    return np.array(positions)

# Plotting the workspace
def plot_workspace(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, alpha=0.5)
    ax.set_title('6DOF Robot Workspace')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.grid(True)
    ax.set_box_aspect([1,1,1])
    plt.show()

workspace_points = generate_workspace(5000)
plot_workspace(workspace_points)
