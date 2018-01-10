.. _camera_calibration_files:

Camera Calibration Files
========================

In order to perform calibration, you need to first define all of the cameras that you will be calibrating.
This file specifies certain specific camera information such as image height/width, camera frame name, image topic, etc.
The first item to identify is whether your camera is moving or static in the scene.  A static camera is one which has a fixed
transform (e.g. a statically mounted camera on the ceiling, or a statically mounted camera on a robot).  A moving camera is one which moves from image to image and does not have a fixed transform (e.g. performing intrinsic calibration of camera where the camera is waved in front of the target or the target is waved in front of the camera).  For most extrinsic calibration setups, the `static_camera` type will be used.  The `moving_camera` type is primarily used for intrinsic calibration in the afore mentioned scenario (although some intrinsic calibration routines do use the `static_camera` type if the camera is mounted on a robot or linear rail).

Here is an example camera calibration file:

.. literalinclude:: yaml/camera_definition.yaml
   :language: yaml
   :lines: 1-50

Here is a list of all of the possible fields and what they are used for:
 * camera_name - the name of the camera, used for creating the calibration job file
 * trigger - See :doc:`trigger_type` for more information
 * image_topic - the topic to grab the 2D image from
 * camera_optical_frame - the TF frame which the 2D image is taken in
 * camera_mounting_frame - 
 * angle_axis_ax/ay/az - 
 * position_x/y/z - 
 * camera intrinsics (focal length, center point, distortion) - optional; camera info can be pulled from the camera info topic, if available.  If not, these fields must be filled out if using a 2D camera


