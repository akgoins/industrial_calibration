.. _TransformInterface:

Calibration Transform Interface
===============================
The transform interface is the method that is used to either lookup or update the transforms
that are used for calibration and ususally only applies to extrinsic calibration.  Several different
transform interfaces are provided, but not all of them currently have a use case.

Transform interface determines how the frame is looked up and whether the new calibration results should be pushed or not.

  * **ros_lti** - ROSListenerTransInterface: performs a TF lookup, final calibration results will not be pushed back to the mutable joint state publisher (if available).  This is used primarily if a transform is needed for calibration (i.e. a fixed calibration target) but the transform is already known or is calibrated by some other means.  For example if a calibration target position is found by touching off using a robot tool tip and the transform is manually entered into the URDF, then a listener transform interface can be used to look up the transform when trying to calibrate a camera since the target transform has already been calibrated and fixed.
  * **ros_bti** - ROSBroadcastTransInterface: publishes a transform pose to the TF topic.  This is used when you want to have some quick feedback on the location of a transform.  For example, if this interface is used when calibrating a target, the current target transform will be published.  This interface should not be used if the TF frames already are being published by another node or if the frames exist in the URDF as they will be published by the robot state publisher.
  * **ros_camera_lti** - ROSCameraListenerTransInterface: This is functionally the same as the ros_lti interface except that the transform returned is the inverse.
  * **ros_camera_bti** - ROSCameraBroadcastTransInterface: This is functionally the same as the ros_bti interface except that the resulting calibrated transform is inverted before being pushed.  This is used for cameras since the calibration target points must be transformed into the camera's frame of reference (T camera->world * T world->target) but the transform that is published is inverted (T world->camera).
  * **ros_camera_housing_lti** - ROSCameraHousingListenerTInterface:  This is similar to the ros_lti except that it allows having a fixed translation/rotation after the transform being optimized.  This used used in the case that there is a camera/device which one or more lenses which are not located at the base frame of the device.  For example, the ASUS and Kinect have two cameras (rgb and ir) which have a fixed, known offset from the camera housing base.  The calibration job needs to calibrate the transform for the base of the camera, however, the image data is being published in their respective camera lens frames.  This can also be used for stereo pair cameras which have a left/right lens.
  * **ros_camera_housing_bti** - ROSCameraHousingBroadcastTInterface: This is functionally the same as the ros_bti interface except that the resulting calibrated transform is inverted before being pushed.  This is used for cameras since the calibration target points must be transformed into the camera's frame of reference (T camera->world * T world->target) but the transform that is published is inverted (T world->camera).
  * **ros_camera_housing_cti** - ROSCameraHousingCalTInterface: This interface is used in conjunction with the mutable joint state publisher and allows for both pulling and pushing transforms.  The resulting calibrated transform is inverted before being pushed.  This is used for cameras since the calibration target points must be transformed into the camera's frame of reference (T camera->world * T world->target) but the transform that is published is inverted (T world->camera).
  * **ros_scti** - ROSSimpleCalTInterface: This interface is used in conjunction with the mutable joint state publisher and allows for both pulling and pushing transforms.  For use with targets.  Pushes the un-inverted transform.
  * **ros_camera_scti** - ROSSimpleCameraCalTInterface: This interface is used in conjunction with the mutable joint state publisher and allows for both pulling and pushing transforms.  For use with cameras.  The resulting calibrated transform is inverted before being pushed.  This is used for cameras since the calibration target points must be transformed into the camera's frame of reference (T camera->world * T world->target) but the transform that is published is inverted (T world->camera).
  * **ros_default_ti** - DefaultTransformInterface: This interface is used if one is not provided or the name does not match on in the available list.  This interface does not provide any functionality to lookup or publish a transform and is only for preventing any runtime errors if a transform interface is not provided.

Here is a summary of the different transform interfaces and how they work:

+------------------------+-----------+---------------+-----------+----------------------+
|                        | Pulls     | Pushes        | Transform | Store Location       |
|                        | Transform | Transform     | Offset    |                      |
+========================+===========+===============+===========+======================+
| ros_lti                | yes       | no            | no offset | N/A                  |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_bti                | no        | yes           |  none     | save to launch file  |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_camera_lti         |     yes   |   no          |  yes      |      N/A             |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_camera_bti         | no        | yes, inverted |  yes?     | save to launch file  |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_camera_housing_lti |  yes      |    no         |   yes     |     N/A              |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_camera_housing_bti |  no       | yes, inverted |   yes     |  save to launch file |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_camera_housing_cti |  yes      | yes, inverted |   yes     | mutable joint states |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_scti               |  yes      |  yes          |    no     | mutable joint states |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_camera_scti        |   yes     | yes, inverted |    no     | mutable joint states |
+------------------------+-----------+---------------+-----------+----------------------+
| ros_default_ti         |    no     |     no        |   no      |      N/A             |
+------------------------+-----------+---------------+-----------+----------------------+

Transform Use Cases
-------------------

Here is a short description on some of the use cases for each transform interface.

  * **ros_lti** - None
  * **ros_bti** - None
  * **ros_camera_lti** - None
  * **ros_camera_bti** - None
  * **ros_camera_housing_lti** - None
  * **ros_camera_housing_bti** - None
  * **ros_camera_housing_cti** - Calibrating a camera which has multiple lenses that are not located at the camera base link.
  * **ros_scti** - Calibrating a calibration target
  * **ros_camera_scti** - Calibrating a camera which has a single lens for which the transform is located in the lens optical center.
  * **ros_default_ti** - None

