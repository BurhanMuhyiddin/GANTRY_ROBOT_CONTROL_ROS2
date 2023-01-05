import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

file_path = "/home/brh/robot2_ws/src/robot_control/data/vel_acc.txt"
df = pd.read_csv(file_path)

vel = df[["velx", "vely", "velz"]].to_numpy()
acc = df[["accx", "accy", "accz"]].to_numpy()

joint_names = ['slider_fb_link_middle_link_a_joint', 'virtual_link_slider_fb_link_joint', 'sslider_ud_link_virtual_link_joint']

# plt.figure(1)
# plt.plot(pos_np)
# plt.xlabel('time [s]')
# plt.ylabel('position [m]')
# plt.legend(joint_names)

plt.figure(2)
plt.plot(vel)
plt.xlabel('time [s]')
plt.ylabel('velocity [m/s]')
plt.legend(joint_names)

plt.figure(3)
plt.plot(acc)
plt.xlabel('time [s]')
plt.ylabel('acceleration [m/s^2]')
plt.legend(joint_names)

plt.show()