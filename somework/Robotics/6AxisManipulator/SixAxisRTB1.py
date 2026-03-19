from roboticstoolbox import DHRobot, RevoluteDH
from math import pi
import matplotlib as plt
import matplotlib.pyplot as plt
import matplotlib

# Use a GUI backend explicitly
matplotlib.use('TkAgg')  # Or 'Qt5Agg' if you're using PyQt

class SixAxisManipulator(DHRobot):
    def __init__(self):
        super().__init__(
                [
                    RevoluteDH(alpha=pi/2, d= 0.6),
                    RevoluteDH(a=0.2),
                    RevoluteDH(a=0.15),
                    RevoluteDH(a=0.08, d=0.06, alpha=pi/2),
                    RevoluteDH(alpha=-pi/2, d=0.06),
                    RevoluteDH(d=0.05),
                ], name="SAM"
        )
        self.qz = [0, 0, 0, 0, 0, 0]

robot = SixAxisManipulator()

# If using Jupyter Notebook, ensure inline plotting
# %matplotlib inline  # Uncomment this line if using Jupyter Notebook

# Plot at zero configuration (qz)
robot.plot(robot.qz)

# Forward kinematics at a given configuration
T = robot.fkine([0.1 ,0.2, 0.3, 0.4, 0.5, 0.6])

# Solve for inverse kinematics using Levenberg-Marquardt
sol = robot.ikine_LM(T)
# print("IK Solution:", sol)

# Plot a different configuration q
q = [0, 0, 0, 0, 0, 0]  # Zero configuration (same as qz)
robot.plot(robot.qz)
 # Should work here
