# import roboticstoolbox as rtb
# panda = rtb.models.URDF.Panda()
# # print(panda)
# T = panda.fkine(panda.qz, end='panda_hand')
# print(T)
# qz=[0,0,0,0,0,0,0]
# panda.plot(qz, backend="swift")

from roboticstoolbox import DHRobot, RevoluteDH
from math import pi
import matplotlib.pyplot as plt
import matplotlib

# Use a GUI backend explicitly
matplotlib.use('TkAgg')  # Or 'Qt5Agg' if you're using PyQt

class SixAxisManipulator(DHRobot):
    def __init__(self):
        super().__init__(
            [
                RevoluteDH(alpha=pi/2, d=0.6),
                RevoluteDH(a=0.2),
                RevoluteDH(a=0.15),
                RevoluteDH(a=0.08, d=0.06, alpha=pi/2),
                RevoluteDH(alpha=-pi/2, d=0.06),
                RevoluteDH(d=0.05),
            ],
            name="SAM"
        )
        self.qz = [0, 0, 0, 0, 0, 0]

robot = SixAxisManipulator()

robot.plot(robot.qz, backend='pyplot')

plt.show()  # This opens the plot window

# 👇 Keeps the script alive until you press Enter
input("Press Enter to close...")

