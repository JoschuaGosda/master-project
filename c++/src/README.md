The kinematics can be specified via a xml or urdf file in Robotics Library. At first the DH-parameters of an past project were used and implemented according to the tutorial of RL. RL uses the standard convention for specifing the dh parameter, namely:

1. **d** - specifies the translation along the previous z axis
2. **theta** - is the rotation around the previous z axis to align with the new x axis
3. **a** - defines the translation along the new x axis
4. **alpha** - gives the rotation around the new x axis to align with the new z axis

Unfortunately the parameters are for an older version of YuMi called Frida and do not match the TCP position and orientation when comparing against RobotStudio in a given joint configuration.
Fortunately there is another option to specifies the kinematics for a robot over a urdf file and there is already a model existing in the repo of OrebroUniversity. Their urdf-description file was then edited (which is not recommended but works) in order to specify the kinematic configuration for the left and right arm of YuMi. When comparing the resulting positions and orientations of the forward kinematics there is still an offset between the manual forward kinematics and RobotStudio. This is because the TCP-reference frame is not the same, as it can be seen in picture xy. 
