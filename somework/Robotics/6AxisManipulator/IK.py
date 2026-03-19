import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Robot Parameters
a1, a2, a3 = 0.2, 0.15, 0.08
d1, d2, d3, d4 = 0.06, 0.06, 0.06, 0.03

# Desired End-Effector Pose
P_desired = np.array([-0.27, -2.52, 0])
R_desired = np.eye(3)

# Initial Joint Angles (guess)
q = np.zeros(6)

# Inverse Kinematics Settings
tol = 1e-6
max_iters = 10

def compute_A(a, alpha, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
        [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_and_jacobian(q):
    DH = [
        [0,      np.pi/2, d1, q[0]],
        [a1,     0,       0,  q[1]],
        [a2,     0,       0,  q[2]],
        [a3,     np.pi/2, d2, q[3]],
        [0,     -np.pi/2, d3, q[4]],
        [0,      0,       d4, q[5]]
    ]

    T = np.eye(4)
    z = np.array([0, 0, 1])
    p = np.array([0, 0, 0])
    joint_positions = [p.copy()]
    J_v = []
    J_w = []

    for i in range(6):
        a, alpha, d, theta = DH[i]
        A = compute_A(a, alpha, d, theta)
        T = T @ A
        R = T[:3, :3]
        P = T[:3, 3]
        joint_positions.append(P.copy())

        J_v.append(np.cross(R @ z, (P - p)))
        J_w.append(R @ z)
        p = P

    J = np.vstack((np.array(J_v).T, np.array(J_w).T))
    return P, R, J, np.array(joint_positions).T

def plot_robot(joint_positions, P_desired):
    fig = plt.figure(figsize=(6,6))
    ax = fig.add_subplot(111, projection='3d')
    # Cleaned version without redundancy
    ax.plot(joint_positions[0], joint_positions[1], joint_positions[2], '-bo', linewidth=2)
    ax.scatter(0, 0, 0, color='k', s=50)
    ax.scatter(P_desired[0], P_desired[1], P_desired[2], color='r', marker='*', s=100)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('6DOF Robot Visualization')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-3, 1])
    ax.set_zlim([0, 1])
    ax.view_init(elev=30, azim=45)
    plt.grid(True)
    plt.pause(1)
    plt.clf()

# IK Loop
for i in range(max_iters):
    P_curr, R_curr, J, joint_positions = forward_kinematics_and_jacobian(q)

    # Compute error
    e_p = P_desired - P_curr
    e_o = 0.5 * (np.cross(R_curr[:,0], R_desired[:,0]) +
                 np.cross(R_curr[:,1], R_desired[:,1]) +
                 np.cross(R_curr[:,2], R_desired[:,2]))
    error = np.concatenate((e_p, e_o))

    if np.linalg.norm(error) < tol:
        print("Converged!")
        break

    delta_q = np.linalg.pinv(J) @ error
    q += delta_q

    plot_robot(joint_positions, P_desired)

else:
    print("Max iterations reached without convergence.")

# Final visualization
plot_robot(joint_positions, P_desired)
plt.show()
