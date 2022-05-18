from urdfpy import URDF
import numpy as np

robot = URDF.load('/home/joschua/Coding/forceControl/master-project/c++/models/urdf/yumi_left.urdf')

# print the joint names
for joint in robot.actuated_joints:
    print(joint.name)

fk = robot.link_fk(cfg=np.array([2, 3, 0, -1, 0, 0, 2]))

for i in range(len(robot.links)):
    print(fk[robot.links[i]])
a = 1