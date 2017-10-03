#Serial Number checks
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

# Peform Depth Calibration
## Run calibration
roslaunch rgbd_depth_calibration automated_calibrate.launch

## Validate results

launch corrected point clouds (ensure the node reads the correct files from Step 5:)
```
roslaunch rgbd_depth_correction correction.launch file:=<insert serial number>
```
apply changes
```
rosservice call /calibration_service "allowable_cost_per_observation: 1.0"
## Save files
1. save <serial number>.yaml and <serial number>.pcd to data folder
2. take screen shots in rviz and save to data folder
