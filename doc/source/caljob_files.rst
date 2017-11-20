Calibration Job Files
========================

The calibration job file is what brings together both the camera and target file to define what pictures are to be taken in order to complete the calibration process.  This file can be as simple as one picture if performing calibration using one camera and one calibration target, or it could be quite lengthy if there are multiple cameras and multiple images that must be taken.

Here is an example calibration job file:

.. literalinclude:: yaml/caljob_definition.yaml
   :language: yaml
   :lines: 1-20


A calibration job can consist of many different scenes, and one or more observations per scene.  
A calibration job must have at least one scene and each scene must have at least one observation.
 * scene -
 * scene_id - Each scene must have a scene ID. IDs must start at zero and increment by one everytime a new scene is created
 * trigger -
 * trigger_action_server - If performing calibration with a robot, this is the ROS MoveIt! action server used to move the robot
 * joint_values - If performing calibration with a robot, this is the set of joint values which are used to 
   command the robot to the desired position before acquiring the image.
 * observation -
 * camera - the name of the camera to take a picture from, the name must be one which was defined in the
   camera calibration file.
 * target - the name of the target to look for in the image, the name must be one which was defined in the 
   target calibration file.
 * roi_x/y_min/max - The region of interest to find the target in.  This is useful if there are multiple  
   targets visible in the scene or there are objects in the background which may cause interference when trying to 
   find the calibration target.
 * cost_type - The cost function to use when adding the observation

Cost Functions
--------------

The cost function is used to define what parameters are fixed and what parameters are being optimized.  If intrinsic calibration is being
performed, then the fixed parameters are the target observations (points with respect to each other) and the parameters being optimized are
the camera intrinsics (focal length, center point, distortion).  If extrinsic calibration is being performed, then the fixed parameters are
the camera intrinsics a the parameters being optimized are the camera location (x,y,z,r,p,y).

Here are a list of all of the currently available cost functions.  These may be added to if a new use case is found, so this list may not be always up to date:
 * cameraPntResidualDist
 * projectPntNoDistortion
 * cameraCircResidualDist
 * cameraCircResidual
 * cameraPntResidual
 * cameraReprjErrorWithDistortion
 * CameraReprjErrorWithDistortionPK
 * CameraReprjError
 * TriangulationError
 * CameraReprjErrorPK
 * TargetCameraReprjError
 * TargetCameraReprjErrorPK
 * LinkTargetCameraReprjError
 * LinkTargetCameraReprjErrorPK
 * PosedTargetCameraReprjErrorPK
 * LinkCameraTargetReprjError
 * LinkCameraTargetReprjErrorPK
 * CircleCameraReprjErrorWithDistortion
 * CircleCameraReprjErrorWithDistortionPK
 * CircleCameraReprjError
 * CircleCameraReprjErrorPK
 * FixedCircleTargetCameraReprjErrorWithDistortion
 * FixedCircleTargetCameraReprjErrorWithDistortionPK
 * CircleTargetCameraReprjErrorWithDistortion
 * SimpleCircleTargetCameraReprjErrorWithDistortionPK
 * CircleTargetCameraReprjErrorWithDistortionPK
 * CircleTargetCameraReprjError
 * CircleTargetCameraReprjErrorPK
 * LinkCircleTargetCameraReprjError
 * LinkCircleTargetCameraReprjErrorPK
 * LinkCameraCircleTargetReprjError
 * LinkCameraCircleTargetReprjErrorPK
 * FixedCircleTargetCameraReprjErrorPK
 * RailICal
 * LinkRangeCameraTargetReprjErrorPK





