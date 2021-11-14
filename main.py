import cv2
import copy
from insert_peg import insert_peg
import numpy as np
from torque_control import Torque

env = insert_peg()
spec = env.action_spec()
time_step = env.reset()

duration = 10  # Seconds
frames = []
ticks = []
rewards = []
observations = []
torque = Torque(time_step.observation['arm_pos'][:, np.newaxis], time_step.observation['arm_vel'][:, np.newaxis])
# initial PID
while env.physics.data.time < duration:
    goal = np.array([[0.25], [-0.15], [-1.55]])
    u = torque.torque_calc(time_step.observation['arm_pos'][:, np.newaxis],
                           time_step.observation['arm_vel'][:, np.newaxis], goal)
    time_step = env.step(u.squeeze())
    camera0 = env.physics.render(camera_id=0, height=480, width=640)  # global vision
    camera1 = env.physics.render(camera_id=1, height=480, width=640)  # local vision
    frames.append(np.hstack((camera0, camera1)))
    cv2.imshow('camera0', camera0)  # show img
    if cv2.waitKey(1) == ord('q'):
        break
    rewards.append(time_step.reward)
    observations.append(copy.deepcopy(time_step.observation))  # arm_pos,arm_vel and touch(only scalar)
    ticks.append(env.physics.data.time)

# Show video and plot reward and observations
num_sensors = len(time_step.observation)
print(observations)
print(len(frames))
for i in range(len(frames)):
    cv2.imshow('video', frames[i])
    if cv2.waitKey(10) == ord('q'):
        break
