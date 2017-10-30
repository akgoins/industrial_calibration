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

#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <industrial_extrinsic_cal/camera_observer_trigger.h>
#include <industrial_extrinsic_cal/user_accept.h>
#include <industrial_extrinsic_cal/ros_camera_observer.h>
#include <industrial_extrinsic_cal/basic_types.h>
#include <industrial_extrinsic_cal/ceres_costs_utils.h> 
#include <industrial_extrinsic_cal/ceres_costs_utils.hpp> 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>
#include <industrial_extrinsic_cal/calibrationAction.h>
#include <industrial_extrinsic_cal/calibrate.h>
#include <industrial_extrinsic_cal/covariance.h>
#include <industrial_extrinsic_cal/find_target.h>
#include <industrial_extrinsic_cal/camera_definition.h>
#include <industrial_extrinsic_cal/ros_transform_interface.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/types.h"

using std::string;
using boost::shared_ptr;
using boost::make_shared;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using industrial_extrinsic_cal::Target;
using industrial_extrinsic_cal::CameraObservations;
using industrial_extrinsic_cal::ROSCameraObserver;
using industrial_extrinsic_cal::Roi;
using industrial_extrinsic_cal::Pose6d;
using industrial_extrinsic_cal::Point3d;
class RangeExCalService 
{
public:

  typedef actionlib::SimpleActionServer<industrial_extrinsic_cal::calibrationAction> CalibrationActionServer;

  RangeExCalService(ros::NodeHandle nh);
  ~RangeExCalService()  {  } ;
  bool executeCallBack( industrial_extrinsic_cal::find_target::Request &req, industrial_extrinsic_cal::find_target::Response &res);
  bool actionCallback(const industrial_extrinsic_cal::calibrationGoalConstPtr& goal);
  void  initMCircleTarget(int rows, int cols, double circle_dia, double spacing);
  void interpolate(double l, double m, double u, double lv, double &mv, double uv)
  {
    mv = lv +  (m-l)/(u-l)*(uv-lv);
  }
private:
  ros::NodeHandle nh_;
  ros::ServiceServer range_excal_server_;
  shared_ptr<Target> target_;
  shared_ptr<industrial_extrinsic_cal::Camera> camera_;
  string camera_name_;
  string image_topic_;
  string camera_frame_;
  std::string target_frame_;
  string camera_mounting_frame_;
  string cloud_topic_;
  int image_height_;
  int image_width_;
  int target_type_;
  int target_rows_;
  int target_cols_;
  Roi roi_;
  CalibrationActionServer action_server_;
};

RangeExCalService::RangeExCalService(ros::NodeHandle nh):
action_server_(nh_,"run_calibration",boost::bind(&RangeExCalService::actionCallback, this, _1), false)
{
  nh_ = nh;
  ros::NodeHandle pnh("~");

  int rows, cols;
  double diameter, spacing;
  if(!pnh.getParam( "image_topic", image_topic_)){
    ROS_ERROR("Must set param:  image_topic");
  }
  if(!pnh.getParam( "cloud_topic", cloud_topic_)){
    ROS_ERROR("Must set param:  cloud_topic");
  }
  if(!pnh.getParam( "camera_name", camera_name_)){
    ROS_ERROR("Must set param: camera_name");
  }
  if(!pnh.getParam( "image_height", image_height_)){
    ROS_ERROR("Must set param: image_height");
  }
  if(!pnh.getParam( "image_width", image_width_)){
    ROS_ERROR("Must set param: image_width");
  }
  if(!pnh.getParam( "target_frame", target_frame_)){
    ROS_ERROR("Must set param: target_frame");
  }
  if(!pnh.getParam( "camera_frame", camera_frame_)){
    ROS_ERROR("Must set param: camera_frame");
  }
  if(!pnh.getParam( "camera_mounting_frame", camera_mounting_frame_)){
    ROS_ERROR("Must set param: camera_mounting_frame");
  }
  if(!pnh.getParam( "ROI_xmin", roi_.x_min)){
    ROS_ERROR("Must set param: ROI_xmin");
  }
  if(!pnh.getParam( "ROI_ymin", roi_.y_min)){
    ROS_ERROR("Must set param: ROI_ymin");
  }
  if(!pnh.getParam( "ROI_xmax", roi_.x_max)){
    ROS_ERROR("Must set param: ROI_xmax");
  }
  if(!pnh.getParam( "ROI_ymax", roi_.y_max)){
    ROS_ERROR("Must set param: ROI_ymax");
  }

  target_type_ == pattern_options::ModifiedCircleGrid;
  if(!pnh.getParam( "target_rows", target_rows_)){
    ROS_ERROR("Must set param:  target_rows");
  }
  if(!pnh.getParam( "target_cols", target_cols_)){
    ROS_ERROR("Must set param:  target_cols");
  }
  if(!pnh.getParam( "target_circle_dia", diameter)){
    ROS_ERROR("Must set param:  target_circle_dia");
  }
  if(!pnh.getParam( "target_spacing", spacing)){
    ROS_ERROR("Must set param:  target_spacing");
  }
  initMCircleTarget(target_rows_, target_cols_, diameter, spacing);
  
  std::string service_name;
  if(!pnh.getParam("service_name", service_name)){
    service_name = "RangeExCalService";
  }
  industrial_extrinsic_cal::CameraParameters temp_parameters;
  temp_parameters.height = image_height_;
  temp_parameters.width = image_width_;
  camera_ = boost::make_shared<industrial_extrinsic_cal::Camera>(camera_name_, temp_parameters, false);
  // use the same service type as ususal for calibration, no need to create a new one
  range_excal_server_ =nh.advertiseService(service_name.c_str(), &RangeExCalService::executeCallBack, this);
  camera_->camera_observer_ = boost::make_shared<ROSCameraObserver>(image_topic_, camera_name_);
  action_server_.start();
}

bool RangeExCalService::actionCallback(const industrial_extrinsic_cal::calibrationGoalConstPtr& goal)
{
  industrial_extrinsic_cal::find_target::Request request;
  industrial_extrinsic_cal::find_target::Response response;
  request.allowable_cost_per_observation = goal->allowable_cost_per_observation;
  if(executeCallBack(request, response)){
    action_server_.setSucceeded();
    return(true);
  }
  action_server_.setAborted();
  return(false);
}
bool RangeExCalService::executeCallBack( industrial_extrinsic_cal::find_target::Request &req,
           industrial_extrinsic_cal::find_target::Response &res)
{
  ros::NodeHandle nh;
  CameraObservations camera_observations;

  // set initial conditions to something that should converge when looking more or less straight at a target
  //shared_ptr<industrial_extrinsic_cal::TransformInterface>  temp_ti =
  //  boost::make_shared<industrial_extrinsic_cal::ROSSimpleCameraCalTInterface>(camera_mounting_frame_, camera_frame_ );

  shared_ptr<industrial_extrinsic_cal::TransformInterface>  temp_ti =
    boost::make_shared<industrial_extrinsic_cal::ROSCameraHousingCalTInterface>(target_frame_, camera_frame_ , camera_mounting_frame_);
  std::string ref_frame("dummy");
  temp_ti->setReferenceFrame(ref_frame);
  camera_->setTransformInterface(temp_ti);
  camera_->pullTransform();


  camera_->camera_observer_->clearObservations();
  camera_->camera_observer_->clearTargets();

  industrial_extrinsic_cal::Cost_function cost_type; // don't need to set because this node assumes its cost type
  
  // get the observations from the intensity image
  camera_->camera_observer_->addTarget(target_, roi_, cost_type);

  // get the range data from the 3D camera
  boost::shared_ptr<sensor_msgs::PointCloud2 const> msg;
  msg  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic_, ros::Duration(10));
  if(!msg){
    ROS_ERROR_STREAM("No point cloud recieved");
    return false;
  }
  ROS_INFO("Received point cloud of size %d X %d", msg->width, msg->height);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud);

  // convert point cloud to an OpenCV image by extracting the color from each point
  cv::Mat rgb_depth_image = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC3);

  for(int j = 0; j < cloud->height; ++j)
  {
    for(int i = 0; i < cloud->width; ++i)
    {
      pcl::PointXYZRGB pt(cloud->points[i + j * cloud->width]);
      if(std::isnan(pt.x))
      {
        continue;
      }
      cv::Vec3b color;
      color[0] = pt.r; color[1] = pt.g; color[2] = pt.b;
      rgb_depth_image.at<cv::Vec3b>(cv::Point(i, j)) = color;
    }
  }
  // convert image to mono
  cv::Mat mono_image = cv::Mat::zeros(rgb_depth_image.cols , rgb_depth_image.rows, CV_8UC1);
  cv::cvtColor(rgb_depth_image, mono_image, cv::COLOR_RGB2GRAY);

  camera_->camera_observer_->getObservations(mono_image, camera_observations);

  //camera_->camera_observer_->triggerCamera();
  //while (!camera_->camera_observer_->observationsDone()) ;
  //camera_->camera_observer_->getObservations(camera_observations);

  int num_observations = (int) camera_observations.size();
  if(num_observations != target_rows_* target_cols_){
    ROS_ERROR("Range Camera Extrinisc Calibration: target not found. Only %d points", num_observations);
    return(false);
  }



  ROS_INFO("Setting up the problem");
  Problem problem;
  pcl::PointXYZ first_pt, last_pt;
  for(int i=0; i<num_observations; i++)
  {
    // the x,y location of the center of the region is found with sub-pixel accuracy
    // therefore the position in x,y,z of this point in space must be interpolated between the four surrounding point cloud points
    double image_x = camera_observations[i].image_loc_x; 
    double image_y = camera_observations[i].image_loc_y;

    unsigned int fx = floor(image_x); 
    unsigned int fy = floor(image_y);
    unsigned int cx = ceil(image_x);
    unsigned int cy = ceil(image_y);
    int height = cloud->height; // here we are assuming the height and width of the image and the point cloud are identical
    int width = cloud->width;// and that there is a one to one correspondence between pixels and cloud points
    Point3d tpoint; // target point
    tpoint.x =target_->pts_[i].x;
    tpoint.y = target_->pts_[i].y;
    tpoint.z = target_->pts_[i].z; 
    // four quadrants for interpolation
    pcl::PointXYZRGB pff(cloud->points[fx + fy*width]);
    pcl::PointXYZRGB pfc(cloud->points[fx + cy*width]);
    pcl::PointXYZRGB pcf(cloud->points[cx + fy*width]);
    pcl::PointXYZRGB pcc(cloud->points[cx + cy*width]);
    if(std::isnan(pff.x) || std::isnan(pfc.x) || std::isnan(pcf.x) || std::isnan(pcc.x))
    {
      ROS_WARN("one or more points has NaN depth data");
    }
    double pctx = image_x - fx;// percentages since floor is always less than image_x by less than 1
    double pcty = image_y - fy;
    pcl::PointXYZ pt1(pff.x + pctx*(pfc.x-pff.x), pff.y + pctx*(pfc.y-pff.y), pff.z+ pctx*(pfc.z-pff.z)); // interpolate along x with y low
    pcl::PointXYZ pt2(pfc.x + pctx*(pcf.x-pcc.x), pfc.y + pctx*(pcf.y-pcc.y), pfc.z+ pctx*(pcf.z-pcc.z)); // interpolate along x with y low
    pcl::PointXYZ pt3(pt1.x + pcty*(pt2.x-pt1.x), pt1.y + pcty*(pt2.y-pt1.y), pt1.z+ pcty*(pt2.z-pt1.z)); // interp along y between p1&p2
    
    if(i == 0)
    {
      first_pt.x = pff.x;
      first_pt.y = pff.y;
      first_pt.z = pff.z;
    }
    else if(i == num_observations - 1)
    {
      last_pt.x = pff.x;
      last_pt.y = pff.y;
      last_pt.z = pff.z;
    }
    //ROS_INFO("image(%f %f) pff(%f %f %f), tpt(%f %f %f)", image_x, image_y, pff.x, pff.y, pff.z, tpoint.x, tpoint.y, tpoint.z);
    // using the image_location x and y, determine the best estimate of x,y,z
    if(std::isnan(pt3.x))
    {
      continue;
    }
    CostFunction* cost_function =
      industrial_extrinsic_cal::RangeSensorExtrinsicCal::Create(pt3.x, pt3.y, pt3.z, tpoint);
    problem.AddResidualBlock(cost_function, NULL , camera_->camera_parameters_.pb_extrinsics);
  }

  // compute distance of diagonal and compare to actual distance
  ROS_INFO("first point: %4.3f, %4.3f, %4.3f", first_pt.x, first_pt.y, first_pt.z);
  ROS_INFO("last point: %4.3f, %4.3f, %4.3f", last_pt.x, last_pt.y, last_pt.z);
  double dist = pow(first_pt.x - last_pt.x, 2) + pow(first_pt.y - last_pt.y, 2) + pow(first_pt.z - last_pt.z, 2);
  dist = sqrt(dist);
  int last = num_observations - 1;
  double real_dist = pow(target_->pts_[0].x - target_->pts_[last].x, 2) + pow(target_->pts_[0].y - target_->pts_[last].y, 2) + pow(target_->pts_[0].z - target_->pts_[last].z, 2);
  real_dist = sqrt(real_dist);
  double error = fabs(dist - real_dist)/real_dist * 100.0;
  ROS_INFO("observed corner distance: %5.3f,  target distance: %5.3f, percent error: %5.3f", dist, real_dist, error);

  ROS_INFO("Solving the problem");
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 15;
  ceres::Solve(options, &problem, &summary);
  ROS_INFO("Ceres Solve Finished!!" );

  if(summary.termination_type != ceres::NO_CONVERGENCE
     ){
    // Set final cost and error
    double error_per_observation = summary.final_cost/num_observations;
    res.final_cost_per_observation  = error_per_observation;
    ROS_INFO("cost per observation = %f", error_per_observation);
    res.percent_error = error;

    // calculate target pose from camera pose
    tf::Transform camera_trans;
    tf::Vector3 position;
    tf::Quaternion orientation;
    position.setX(camera_->camera_parameters_.position[0]);
    position.setY(camera_->camera_parameters_.position[1]);
    position.setZ(camera_->camera_parameters_.position[2]);
    camera_trans.setOrigin(position);
    double x, y, z, w;
    Pose6d temp_pose(camera_->camera_parameters_.position[0], camera_->camera_parameters_.position[1], camera_->camera_parameters_.position[2],
        camera_->camera_parameters_.angle_axis[0], camera_->camera_parameters_.angle_axis[1], camera_->camera_parameters_.angle_axis[2]);
    temp_pose.getQuaternion(x, y, z, w);
    orientation.setX(x);
    orientation.setY(y);
    orientation.setZ(z);
    orientation.setW(w);
    camera_trans.setRotation(orientation);
    //tf::Transform target_trans = camera_trans.inverse();

    // populate return message
    res.final_pose.position.x = camera_trans.getOrigin().getX();
    res.final_pose.position.y = camera_trans.getOrigin().getY();
    res.final_pose.position.z = camera_trans.getOrigin().getZ();
    res.final_pose.orientation.w = camera_trans.getRotation().getW();
    res.final_pose.orientation.x = camera_trans.getRotation().getX();
    res.final_pose.orientation.y = camera_trans.getRotation().getY();
    res.final_pose.orientation.z = camera_trans.getRotation().getZ();

    if(error_per_observation <= req.allowable_cost_per_observation){
      camera_->pushTransform();
      return true;
    }
    else{
      ROS_ERROR("allowable cost exceeded %f > %f", error_per_observation, req.allowable_cost_per_observation);
      return(false);
    }
  }
}

void RangeExCalService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ =  boost::make_shared<industrial_extrinsic_cal::Target>();
  target_->is_moving_ = true;
  target_->target_name_ = "modified_circle_target";
  target_->target_frame_ = "target_frame";
  target_->target_type_ =  2;
  target_->circle_grid_parameters_.pattern_rows =rows;
  target_->circle_grid_parameters_.pattern_cols = cols;
  target_->circle_grid_parameters_.circle_diameter = circle_dia;
  target_->circle_grid_parameters_.is_symmetric = true; 
  // create a grid of points
  target_->pts_.clear();
  target_->num_points_ = rows*cols;
  for(int i=0; i<rows; i++){
    for(int j=0; j<cols; j++){
      Point3d point;
      point.x = j*spacing;
      point.y = (rows -1 -i)*spacing;
      point.z = 0.0;
      target_->pts_.push_back(point);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "range_excal_service");
  ros::NodeHandle node_handle;
  RangeExCalService range_excal(node_handle);
  ros::spin();
  ros::waitForShutdown();
  return 0;
}
