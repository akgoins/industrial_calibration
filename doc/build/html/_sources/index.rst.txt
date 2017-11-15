.. industrial_calibration documentation master file, created by
   sphinx-quickstart on Tue Nov 14 16:09:54 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to industrial_calibration's documentation!
==================================================

This repository is for performing both intrinsic and extrinsic calibration for a variety of 2D and 3D cameras.
The intrinsic parameters of the camera (focal length, center point, distortion parameters, etc.) are primarily used
for 2D cameras.  Depending on the type of 3D camera, intrinsic calibration may not be necessary.  Once intrinsic
calibration is performed (if any is needed), then extrinsic calibration can be executed.  The extrinsic parameters
refer to the cameras location in the environment (x,y,z,r,p,y).

Most users will want to get started quickly calibrating a camera.  For a quick start see 
the Calibration Examples.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   intro
   Calibration Examples <calibration_examples>
   Calibration Libraries <calibration_libraries>
   Calibration Nodes <calibration_nodes>
   Calibration YAML Files <calibration_files>
   



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
