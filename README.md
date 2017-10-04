# Serial Number checks

# Hardware setup
```
sudo chmod a+xwr /dev/ttyUSB0
```

```
sudo gedit /etc/openni2/PS1080.ini
```
find and uncomment the following line
```
;GMCMode=0
```
Save the file and exit or if the file does not exist copy PS1080.ini from the project folder into `/usr/lib/​OpenNI2/Drivers or /usr/local/lib/OpenNI2/Drivers`

Verify the depth correction is turned off by running the openni2.launch, you should see a difference in the point cloud.  With the GMCMode  line commented out, you will see several "vertical" lines in the point cloud.  This is due to some optimization that is trying to smooth out the data.  With the line uncommented, the lines should go away.  If you don't know what you are looking for, come grab me and I can show you.


# Calibrate the RGB Camera
## Run the calibration
in terminal 1 run:
```
roslaunch intrinsic_cal rgb_auto.launch 
```
in terminal 2 run:
```
rosservice call /RobocylCalService "allowable_cost_per_observation: 1.0"
```

## Save the files
Copy the new file which is located at ~/.ros/camera_info/depth_PS1080_PrimeSense.yaml. rename the copied file to <serial number>_rgb.yaml and place in two folders:
- data folder
- industrial_calibration/rgbd_depth_correction/yaml/


# Calibrate the IR Camera
## Run the calibration
in terminal 1 run:
```
roslaunch intrinsic_cal ir_auto.launch 
```
in terminal 2 run:
```
rosservice call /RobocylCalService "allowable_cost_per_observation: 1.0"
```
## save the files
Copy the new file which is located at ~/.ros/camera_info/depth_PS1080_PrimeSense.yaml. rename the copied file to <serial number>_ir.yaml and place in two folders:
- data folder
- industrial_calibration/rgbd_depth_correction/yaml/


# Test the Calibrations
edit the launch file to specify the location of the ir and rgb cameras.
In terminal 1 run:
```
roslaunch industrial_extrinsic_cal rgbd_cal_test.launch
```
in terminal 2 run open rviz and display the pointcloud2 and tf
in terminal 3 run: 
```
rosservice call /RoboExCalService "allowable_cost_per_observation: 1.0"
```

## Depth Correction

### ROS environment
launch the depth correction package in terminal 1:
```
source devel/setup.bash
roslaunch rgbd_depth_correction calibrate.launch file:=<insert serial number>
```
in terminal 2:
```
source devel/setup.bash
roslaunch robo_cylinder robo_cylinder.launch
```

### Start the calibration
Step 1, set the camera approximatly 4ft from the wall and initialize the calibration. in terminal 3 run the following:
```
rosservice call /move_meters "meters: 0.4"
rosservice call /pixel_depth_calibration "{}"
```
view terminal 1 and make sure it solves

step 2. in terminal 3 run the following
```
rosservice call /move_meters "meters: 0.8"
rosservice call /store_cloud "{}" 
```
view terminal 1 and make sure it solves

Step 3. Repeat step 2 at 0.6, 0.4, 0.2, and 0.0

Step 4.  process the gathered point clouds. in terminal 3 run:
```
rosservice call /depth_calibration "{}"
``` 
Step 5.  save results, the location is displayes in terminal 1. rename the output pcd and yaml file to the serial number of the camera
## Validate results

launch corrected point clouds (ensure the node reads the correct files from Step 5:)
```
roslaunch rgbd_depth_correction correction.launch file:=<insert serial number>
```
apply changes
```
rosservice call /calibration_service "allowable_cost_per_observation: 1.0"
```
## Save files
1. save <serial number>.yaml and <serial number>.pcd to data folder
2. take screen shots in rviz and save to data folder
