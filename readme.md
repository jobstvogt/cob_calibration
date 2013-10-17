cob_calibration
===============

Calibrates cameras, arm and torso of Care-O-bot



Prepare the robot for calibration
====
---
described [here](http://www.ros.org/wiki/cob_calibration/Tutorials/preparation)

<!---
1. move to calibration position
```roslaunch cob_calibration_executive move_arm_to_calibration_position.launch```
mount calibration object(defined in step 4 of configuration) on arm
--->


Automatic camera and robot calibration
====
---

Overview
---
This tutorial guides you through the automatic calibration of
the cameras and kinematic components of the Care-O-bot 3. The calibration 
procedure is divided into two automated steps:

1. **Camera calibration**: The stereo camera system of Care-O-bot is calibrated intrinsically.

2. **Kinematic robot calibration**: This step performs extrinsic camera calibration 
(hand-eye calibration of the stereo camera and kinect) and estimates various kinematic parameters of the robot (position and orientation of arm and torso on the base of Care-O-bot).


Prerequisites
---


* ```roscore``` is running
* Care-O-bot bringup software is running.
* Cameras, head-axis, arm and torso are initialized and working.
* Calibration pattern is attached to arm.
* You created a temporary overlay of "cob_calibration_data". The calibration
results are stored in this unary stack. The overlay should be deleted again after
the calibration process.



Running calibration
---

1. **Collect data**

 start the data collection by calling ```roslaunch cob_calibration_executive collect_robot_calibration_data.launch```.
 
 This will start all needed nodes and services. The robot now moves to the sample positions calculated 
 in step 6 of the configuration.
 
 The progress can be seen by ```rostopic echo /calibration/data_collection/progress```.
 
 Wait until capture is finished and stop it with CTRL-C.
 The bagfile with the measurement and the images for the camera calibration are stored in "/tmp/cal/".

2. **Calibrate cameras**
 Execute ```roslaunch cob_camera_calibration calibrate_stereo.launch``` to start the stereo camera calibration.

 The results will be stored in the calibration file specified in the "camera.yaml" configuration file. 
 
 This step should take approximately 3 minutes.

 For robots with only one calibrated camera this step is not required.

3. **Calibrate robot**
 To calibrate the robot run
 ```roslaunch cob_robot_calibration run_robot_calibration.launch```.
 
 The calculation takes about 5 to 10 minutes.


4. **Update urdf**
 ```roslaunch cob_robot_calibration update_calibration_urdf.launch```
 
 copies the result of the optimization to the robot urdf file

5. **Restart bringup**


Final steps
---


1. Verify the calibration result with rviz by checking the kinect pointcloud while the arm with checkerboard is in front of the cameras. The point cloud should align with the simulated arm.
2. Commit the new calibration to git (cob_calibration_data) and push it to github. Create a pull request for ipa320/cob_calibration_data and ask your robot administrator to pull the new calibration to the robot for everybody.
3. Once the pull request has been accepted and the calibration has been updated on the robot, you can remove your local overlay of cob_calibration_data.
4. Remove the checkerboard from the arm and reattach the hand. (activate the emergency stop before attaching the schunk hand).


Configure calibration(needs to be more verbose)
====
---

1. create a new configuration folder in the package "cob_calibration_config"
named after the robot.

2. create the file "user_defined/cameras.yaml"
define how many cameras are involved
for each camera define "topic";"frame_id"(from robot urdf), " property"( position in urdf)
"file_prefix" (for camera calibration)

3. calibration_seed.yaml: teach in on hardware(or simulation)

4. calibration_pattern:
in most cases copy the existing cb9x6 for robots at the ipa


5. generate template for optimization and autogenerated files
bringup robot (in simulation or on real robot)
```roslaunch cob_robot_calibration generate_config.launch```


6. generate calibration positions
with running ik services
```roslaunch cob_calibration_executive``` 

7. create free_0.yaml --> free_2.yaml
for care-o-bot first step with free cb_arm transformation
 second step with cameras mount position added
third step with all unknown transformations
