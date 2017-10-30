/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <industrial_extrinsic_cal/camera_observer_trigger.h>
#include <industrial_extrinsic_cal/user_accept.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp>
#include <intrinsic_cal/rail_ical_location.h>
#include <robo_cylinder/HomeCmd.h>
#include <robo_cylinder/MoveMeters.h>
#include <robo_cylinder/MovePulses.h>
#include <robo_cylinder/PowerIO.h>
#include <robo_cylinder/StatusUpdate.h>
#include <robo_cylinder/VelAcc.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"


using std::string;
using boost::shared_ptr;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using industrial_extrinsic_cal::Target;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::ROSCameraObserver;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::Point3d;
using industrial_extrinsic_cal::Camera;
using industrial_extrinsic_cal::CameraParameters;
using industrial_extrinsic_cal::NoWaitTrigger;

class RobocylCalService
{
public:
  RobocylCalService(ros::NodeHandle nh);
  ~RobocylCalService();
  bool MoveAndReportPose(double rail_position, Pose6d &P);
  bool ReportPose(double rail_position, Pose6d &P);
  void  initMCircleTarget(int rows, int cols, double circle_dia, double spacing);
  void cameraCallback(const sensor_msgs::Image& image);

private:
  void manualCal();
  void autoCal();

  ros::NodeHandle nh_;
  ros::Subscriber rgb_sub_;
  ros::Publisher rgb_pub_;
  ros::ServiceClient move_client_; /**< a client for calling the service to move the robo-cylinder to a new location */
  ros::ServiceClient power_client_; /**< a client for calling the service to turn on the robo-cylinder */
  ros::ServiceClient home_client_; /**< a client for calling the service to move robo-cylinder to its home position */
  shared_ptr<Target> target_;
  shared_ptr<Camera> camera_;
  double focal_length_x_;
  double focal_length_y_;
  double center_x_;
  double center_y_;
  string image_topic_;
  string camera_name_;
  int target_type_;
  int target_rows_;
  int target_cols_;
  double circle_spacing_;
  double circle_diameter_;
  int num_camera_locations_;
  double camera_spacing_;
  int image_height_;
  int image_width_;
  double D0_;
  double qx_, qy_, qz_, qw_;
  CameraParameters camera_parameters_;
  double allowable_cost_per_observation_;
  bool auto_;

};
