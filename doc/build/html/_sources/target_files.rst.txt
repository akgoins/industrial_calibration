Calibration Target Files
========================

The target file defines what the calibration target looks like that you will be using.  Most setups will use one calibration target, but it is possible to have more than one calibration target.  For example, two cameras with different focal lengths could use different sized calibration targets which are both mounted on the same surface.  Just like with the camera calibration file, there are two options for either a static target or a moving target.  Please refer to the :doc:`camera_files` page for a description of what a static/moving target is.

Here is an example target calibration file:

.. literalinclude:: yaml/circlegrid5x7.yaml
   :language: yaml
   :lines: 1-50

Here is a list of all of the possible fields and what they are used for:

 * target_name - the name of the calibration target, to be used in the calibration job file
 * target_type - the type of calibration target.  Possible enumerations are: 
  * 0 = checker board
  * 1 = circle grid
  * 2 = modified circle grid (origin dot is larger than all other dots)
  * 3 = circle balls (i.e. retro reflective markers)

 * target_frame - the name of the TF frame which represents the origin of the target
 * target_rows - the number of rows in the calibration target
 * target_cols - the number of columns in the calibration target
 * circle_dia - the diameter of the circles in the target
 * angle_axis_ax/ay/az - 
 * position_x/y/z - 
 * transform_interface - the transform interface type to use when looking up the ROS TF frames.  The transform interface determines how the frame is looked up and whether the new calibration results should be pushed or not.
  * ros_lti - listener transform interface: performs a TF lookup, final calibration results will not be pushed back to the mutable joint state publisher (if available)
  * ros_bti - broadcast transform interface: broadcasts the transform
  * ros_camera_lti
  * ros_camera_bti
  * ros_camera_housing_lti
  * ros_camera_housing_bti
  * ros_camera_housing_cti
  * ros_scti
  * ros_camera_scti
  * ros_default_ti

End of list
