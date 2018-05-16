#include "intrinsic_cal/robocyl_ical.h"

RobocylCalService::RobocylCalService(ros::NodeHandle nh)
{

  nh_ = nh;
  ros::NodeHandle pnh("~");
  allowable_cost_per_observation_ = 1.0;

  if(!pnh.getParam( "image_topic", image_topic_)){
    ROS_ERROR("Must set param:  image_topic");
  }

  if(!pnh.getParam( "camera_name", camera_name_)){
    ROS_ERROR("Must set param:  camera_name");
  }

  target_type_ == pattern_options::ModifiedCircleGrid;

  if(!pnh.getParam( "target_rows", target_rows_)){
    ROS_ERROR("Must set param:  target_rows");
  }
  if(!pnh.getParam( "target_cols", target_cols_)){
    ROS_ERROR("Must set param:  target_cols");
  }
  if(!pnh.getParam( "target_circle_dia", circle_diameter_)){
    ROS_ERROR("Must set param:  target_circle_dia");
  }
  if(!pnh.getParam( "target_spacing", circle_spacing_)){
    ROS_ERROR("Must set param:  target_spacing");
  }
  if(!pnh.getParam( "num_camera_locations", num_camera_locations_)){
    ROS_ERROR("Must set param:  num_camera_locations");
  }
  if(!pnh.getParam( "camera_spacing", camera_spacing_)){
    ROS_ERROR("Must set param:  camera_spacing");
  }
  if(!pnh.getParam( "image_height", image_height_)){
    ROS_ERROR("Must set param:  image_height_");
  }
  if(!pnh.getParam( "image_width", image_width_)){
    ROS_ERROR("Must set param:  image_width_");
  }
  if(!pnh.getParam( "target_to_rail_distance", D0_)){
    ROS_ERROR("Must set param:  target_to_rail_distance");
  }

  if(!pnh.getParam( "allowable_cost_per_observation", allowable_cost_per_observation_)){
    ROS_ERROR("Must set param:  allowable_cost_per_observation");
  }

  if(!pnh.getParam( "auto", auto_)){
    ROS_ERROR("Must set param:  auto");
  }

  bool use_quaternion = false;
  if(pnh.getParam( "qx", qx_))
  {
    if(pnh.getParam( "qy", qy_))
    {
      if(pnh.getParam( "qz", qz_))
      {
        if(pnh.getParam( "qw", qw_))
        {
          use_quaternion = true;
        }
      }
    }
  }
  string move_client_name("/move_meters");
  if(!pnh.getParam( "move_client", move_client_name)){
    ROS_WARN("move_client = %s", move_client_name.c_str());
  }
  string power_client_name("/power_io");
  if(!pnh.getParam( "power_client", power_client_name)){
    ROS_WARN("power_client = %s", power_client_name.c_str());
  }
  string home_client_name("/home");
  if(!pnh.getParam( "home_client", home_client_name)){
    ROS_WARN("home_client = %s", home_client_name.c_str());
  }

  move_client_  = nh.serviceClient<robo_cylinder::MoveMeters>(move_client_name);
  power_client_  = nh.serviceClient<robo_cylinder::PowerIO>(power_client_name);
  home_client_  = nh.serviceClient<robo_cylinder::HomeCmd>(home_client_name);

  u_int32_t queue_size = 5;
  rgb_sub_ = nh_.subscribe("color_image", queue_size, &RobocylCalService::cameraCallback, this);
  rgb_pub_ = nh_.advertise<sensor_msgs::Image>("color_image_center", 1);

  ros::spinOnce();
  if(!use_quaternion)
  {
    tf::Matrix3x3 m;
    m[0][0] = 1; m[0][1] =   0; m[0][2] = 0;
    m[1][0] = 0; m[1][1] = -1.; m[1][2] = 0;
    m[2][0] = 0; m[2][1] =   0; m[2][2] = -1;
    Pose6d Ptemp;
    Ptemp.setBasis(m);
    Ptemp.setOrigin(-0.1, -0.1, D0_);
    Ptemp.getQuaternion(qx_, qy_, qz_, qw_);
    Ptemp.show("initial pose");
    ROS_WARN("parameters qx, qy, qz, and qw not provided, using default values of (%.2f, %.2f, %.2f, %.2f)", qx_, qy_, qz_, qw_);
  }

  bool is_moving = true;
  camera_ =  shared_ptr<industrial_extrinsic_cal::Camera>(new industrial_extrinsic_cal::Camera("my_camera", camera_parameters_, is_moving));
  camera_->trigger_ = shared_ptr<NoWaitTrigger>(new NoWaitTrigger());
  camera_->camera_observer_ = shared_ptr<ROSCameraObserver>(new ROSCameraObserver(image_topic_, camera_name_));

  int pos = image_topic_.find_last_of('/');
  std::string topic = image_topic_.substr(0, pos);
  ros::service::waitForService(topic+"/set_camera_info",ros::Duration(15)); //should be wait for message to match pull camera info method
  if(!camera_->camera_observer_->pullCameraInfo(camera_->camera_parameters_.focal_length_x,
                                           camera_->camera_parameters_.focal_length_y,
                                           camera_->camera_parameters_.center_x,
                                           camera_->camera_parameters_.center_y,
                                           camera_->camera_parameters_.distortion_k1,
                                           camera_->camera_parameters_.distortion_k2,
                                           camera_->camera_parameters_.distortion_k3,
                                           camera_->camera_parameters_.distortion_p1,
                                           camera_->camera_parameters_.distortion_p2,
                                           image_width_, image_height_))
  {
    ROS_FATAL("Could not get camera information for %s from topic %s. Shutting down node.", camera_name_.c_str(), image_topic_.c_str());
    ros::shutdown();
  }

  ROS_INFO("initial camera info focal:%f %f center:%f %f  radial:%f %f %f tang: %f %f",
            camera_->camera_parameters_.focal_length_x,
            camera_->camera_parameters_.focal_length_y,
            camera_->camera_parameters_.center_x,
            camera_->camera_parameters_.center_y,
            camera_->camera_parameters_.distortion_k1,
            camera_->camera_parameters_.distortion_k2,
            camera_->camera_parameters_.distortion_k3,
            camera_->camera_parameters_.distortion_p1,
            camera_->camera_parameters_.distortion_p2);

  initMCircleTarget(target_rows_, target_cols_, circle_diameter_, circle_spacing_);

  if(auto_) autoCal();
    else

  manualCal();

}

RobocylCalService::~RobocylCalService(){}

void RobocylCalService::cameraCallback(const sensor_msgs::Image &image)
{
  cv_bridge::CvImagePtr bridge = cv_bridge::toCvCopy(image, image.encoding);

  cv::Mat mod_img = bridge->image;
  cv::circle(mod_img, cv::Point2d(image.width / 2.0, image.height / 2.0), 4, cv::Scalar(255,0,0), 2);
  bridge->image = mod_img;

  sensor_msgs::Image out_img;
  bridge->toImageMsg(out_img);
  rgb_pub_.publish(out_img);

}

void RobocylCalService::manualCal()
{
  CameraObservations camera_observations;
  int num_observations;
  int total_observations=0;
  camera_->camera_observer_->clearObservations();
  camera_->camera_observer_->clearTargets();
  Pose6d TtoC1, TtoC2; // transforms points in target frame into either Camera1 or Camera2 frames
  Pose6d C1toT, C2toT; // transforms points in camera1,2 frames into Target Frame
  Pose6d C2toC1; // transform points in camera2 frame into camera1 frame


  //move rail to back
  ros::spin();
  while(true){
    ROS_WARN_STREAM("SET RAIL TO 0 then enter the command: \nrostopic pub --once /manual_cal intrinsic_cal/rail_ical_location \"rail_location: 0.0\")");
    const std::string message_topic = "/manual_cal";
    intrinsic_cal::rail_ical_location command = *(ros::topic::waitForMessage<intrinsic_cal::rail_ical_location>(message_topic));
    if(command.rail_location==0.0){
      ROS_ERROR_STREAM("entered");
      ReportPose(0.0, TtoC1);
      break;
    }
    else ROS_WARN_STREAM("Move rail to 0.0 meters");
  }

  //move rail to front
  while(true){
    float rail_front_location = 0.8;
    ROS_WARN_STREAM("SET RAIL TO 0.8 then enter the command: \nrostopic pub --once /manual_cal intrinsic_cal/rail_ical_location \"rail_location: 0.0\")");
    const std::string message_topic = "/manual_cal";
    intrinsic_cal::rail_ical_location command = *(ros::topic::waitForMessage<intrinsic_cal::rail_ical_location>(message_topic));
    if(command.rail_location == rail_front_location){
      ReportPose(rail_front_location,TtoC2);
      break;
    }
    else ROS_WARN_STREAM("Move rail to 0.8 meters");
  }

  C2toT = TtoC2.getInverse(); // this transform points in C2 frame into Target frame
  C2toC1 = TtoC1*C2toT; // this transforms points in C2 frame into Target then into C1 frame as one multiply
  tf::Vector3 mv = C2toC1.getOrigin(); // the origin is the vector in C1 coordinates
  mv.normalize();

  ROS_INFO("mv: %lf %lf %lf",mv.getX(),mv.getY(),mv.getZ());
  // set the roi to the whole image
  Roi roi;
  roi.x_min = 0;
  roi.y_min = 0;
  roi.x_max = image_width_;
  roi.y_max = image_height_;

  industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::CameraReprjErrorWithDistortion;
  Problem problem;

  // set initial conditions,
  target_->pose_.setQuaternion(qx_, qy_, qz_, qw_);
  target_->pose_.setOrigin(0.011, 0.05, D0_);
  target_->pose_.show("initial target pose");

  for(int i=0; i<num_camera_locations_; i++){
    double Dist = i*camera_spacing_;
    Point3d rail_position;
    rail_position.x = -Dist*mv.getX();
    rail_position.y = -Dist*mv.getY();
    rail_position.z = -Dist*mv.getZ();

    while(true){
      ROS_WARN_STREAM("SET RAIL TO" << Dist << "then enter the command: \nrostopic pub --once /manual_cal intrinsic_cal/rail_ical_location \"rail_location: " << Dist << " \")");
      const std::string message_topic = "/manual_cal";
      intrinsic_cal::rail_ical_location command = *(ros::topic::waitForMessage<intrinsic_cal::rail_ical_location>(message_topic));
      if(command.rail_location == Dist){
        ReportPose(0.0,TtoC2);
        break;
      }
      else ROS_WARN_STREAM("Move rail to " << Dist <<" meters");
    }

    // gather next image
    camera_->camera_observer_->clearTargets();
    camera_->camera_observer_->clearObservations();
    camera_->camera_observer_->addTarget(target_, roi, cost_type);
    camera_->camera_observer_->triggerCamera();
    while (!camera_->camera_observer_->observationsDone()) ;
    if(camera_->camera_observer_->getObservations(camera_observations)){
      ROS_INFO("Found %d observations",(int) camera_observations.size());
      num_observations = (int) camera_observations.size();
      if(num_observations != target_rows_* target_cols_){
  ROS_ERROR("Target Locator could not find target %d", num_observations);
      }
      else{
  // add a new cost to the problem for each observation
  CostFunction* cost_function[num_observations];
  total_observations += num_observations;
        for(int j=0; j<num_observations; j++){
          double image_x = camera_observations[j].image_loc_x;
          double image_y = camera_observations[j].image_loc_y;
          Point3d point = target_->pts_[camera_observations[j].point_id]; // don't assume ordering from camera observer
          cost_function[j] = industrial_extrinsic_cal::RailICal3::Create(image_x, image_y, rail_position, point);
          problem.AddResidualBlock(cost_function[j], NULL ,
                 camera_->camera_parameters_.pb_intrinsics,
                 target_->pose_.pb_pose);
        } // for each observation at this camera_location
      } // end target size matches observation number
    }// end if get observations successful
  }// end for each camera location

  ROS_WARN_STREAM("DONE");
}

void RobocylCalService::autoCal()
{
  CameraObservations camera_observations;
  robo_cylinder::MoveMeters::Request mm_request; /**< request when transform is part of a mutable set */
  robo_cylinder::MoveMeters::Response mm_response; /**< request when transform is part of a mutable set */

  int num_observations;
  int total_observations=0;
  double rxry[2]; // pitch and yaw of camera relative to rail
  rxry[0] = 0.0;
  rxry[1] = 0.0;

  camera_->camera_observer_->clearObservations();
  camera_->camera_observer_->clearTargets();

  Pose6d TtoC1, TtoC2; // transforms points in target frame into either Camera1 or Camera2 frames
  Pose6d C2toT; // transforms points in camera1,2 frames into Target Frame
  Pose6d C2toC1; // transform points in camera2 frame into camera1 frame

  if(!MoveAndReportPose(0.0, TtoC1))
  {
    ROS_ERROR("Could not find target at location 0.  Shutting down node.");
    return;
  }
  if(!MoveAndReportPose((num_camera_locations_ - 1) * camera_spacing_, TtoC2))
  {
    ROS_ERROR_STREAM("Could not find target at location " << (num_camera_locations_-1)*camera_spacing_ << ". Shutting down node. ");
    return;
  }

  C2toT = TtoC2.getInverse(); // this transform points in C2 frame into Target frame
  C2toC1 = C2toT*TtoC1; // this transforms points in C2 frame into Target then into C1 frame as one multiply
  C2toC1 = C2toC1.getInverse();
  tf::Vector3 mv = C2toC1.getOrigin(); // the origin is the vector in C1 coordinates
  mv.normalize();

  ROS_INFO("mv: %lf %lf %lf",mv.getX(),mv.getY(),mv.getZ());
  // set the roi to the whole image
  Roi roi;
  roi.x_min = 0;
  roi.y_min = 0;
  roi.x_max = image_width_;
  roi.y_max = image_height_;

  industrial_extrinsic_cal::Cost_function cost_type = industrial_extrinsic_cal::cost_functions::CameraReprjErrorWithDistortion;
  Problem problem;

  // set initial conditions,
  target_->pose_.setQuaternion(qx_, qy_, qz_, qw_);
  target_->pose_.setOrigin(0.011, 0.05, D0_);
  target_->pose_.show("initial target pose");
  ros::NodeHandle pnh("~");
  bool camera_ready = false;
  pnh.setParam("camera_ready", camera_ready);
  for(int i=0; i<num_camera_locations_; i++){
    double Dist = i*camera_spacing_;
    Point3d rail_position;
    rail_position.x = -Dist*mv.getX();
    rail_position.y = -Dist*mv.getY();
    rail_position.z = -Dist*mv.getZ();

    mm_request.meters = Dist;
    while(true){
      ROS_INFO("Attempting to moving to %lf",Dist);
      if(move_client_.call(mm_request, mm_response)) break; // this call blocks until camera is moved
    }

    // gather next image
    camera_->camera_observer_->clearTargets();
    camera_->camera_observer_->clearObservations();
    camera_->camera_observer_->addTarget(target_, roi, cost_type);
    camera_->camera_observer_->triggerCamera();
    while (!camera_->camera_observer_->observationsDone()) ;
    if(camera_->camera_observer_->getObservations(camera_observations)){
      ROS_INFO("Found %d observations",(int) camera_observations.size());
      num_observations = (int) camera_observations.size();
      if(num_observations != target_rows_* target_cols_){
        ROS_ERROR("Target Locator could not find target %d", num_observations);
      }
      else{
        // add a new cost to the problem for each observation
        CostFunction* cost_function[num_observations];
        total_observations += num_observations;
        for(int i=0; i<num_observations; i++){
          double image_x = camera_observations[i].image_loc_x;
          double image_y = camera_observations[i].image_loc_y;
          Point3d point = target_->pts_[camera_observations[i].point_id]; // don't assume ordering from camera observer
          cost_function[i] = industrial_extrinsic_cal::RailICal3::Create(image_x, image_y, rail_position, point);
          problem.AddResidualBlock(cost_function[i], NULL ,
                 camera_->camera_parameters_.pb_intrinsics,
                 target_->pose_.pb_pose);
        } // for each observation at this camera_location
      } // end target size matches observation number
    }// end if get observations successful
  }// end for each camera location

  // set up and solve the problem
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 2000;
  ceres::Solve(options, &problem, &summary);
  if(summary.termination_type != ceres::NO_CONVERGENCE){
    double initial_cost = summary.initial_cost/total_observations;
    double final_cost = summary.final_cost/total_observations;
    ROS_INFO("Problem solved, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
    target_->pose_.show("target_pose");
    ROS_INFO("camera_matrix data: [ %lf, 0.0, %lf, 0.0, %lf, %lf, 0.0, 0.0, 1.0]",
       camera_->camera_parameters_.focal_length_x,
       camera_->camera_parameters_.center_x,
       camera_->camera_parameters_.focal_length_y,
       camera_->camera_parameters_.center_y);
    ROS_INFO("distortion data: [ %lf,  %lf,  %lf,  %lf,  %lf]",
       camera_->camera_parameters_.distortion_k1,
       camera_->camera_parameters_.distortion_k2,
       camera_->camera_parameters_.distortion_p1,
       camera_->camera_parameters_.distortion_p2,
       camera_->camera_parameters_.distortion_k3);
    ROS_INFO("projection_matrix data: [ %lf, 0.0, %lf, 0.0, 0.0, %lf, %lf, 0.0, 0.0, 0.0, 1.0, 0.0]",
       camera_->camera_parameters_.focal_length_x,
       camera_->camera_parameters_.center_x,
       camera_->camera_parameters_.focal_length_y,
       camera_->camera_parameters_.center_y);
    if(final_cost <= allowable_cost_per_observation_){
      ROS_INFO_STREAM("CALIBRATION SUCCEEDED");
      camera_->camera_observer_->pushCameraInfo(camera_->camera_parameters_.focal_length_x,
                                                camera_->camera_parameters_.focal_length_y,
                                                camera_->camera_parameters_.center_x,
                                                camera_->camera_parameters_.center_y,
                                                camera_->camera_parameters_.distortion_k1,
                                                camera_->camera_parameters_.distortion_k2,
                                                camera_->camera_parameters_.distortion_k3,
                                                camera_->camera_parameters_.distortion_p1,
                                                camera_->camera_parameters_.distortion_p2);
    }
    else{
      ROS_INFO_STREAM("CALIBRATION FAILED");
    }

  }
}


bool RobocylCalService::MoveAndReportPose(double rail_position, Pose6d &P)
{
  robo_cylinder::MoveMeters::Request mm_request; /**< request when transform is part of a mutable set */
  robo_cylinder::MoveMeters::Response mm_response; /**< request when transform is part of a mutable set */

  // Move camera into position
  ROS_INFO("moving to %lf",rail_position);
  mm_request.meters = rail_position;
  while(true){
    ROS_INFO("Attempting to moving to %lf",rail_position);
    if(move_client_.call(mm_request, mm_response)) break; // this call blocks until camera is moved
  }
  return ReportPose(rail_position, P);
}

bool RobocylCalService::ReportPose(double rail_position, Pose6d &P)
{
  CameraObservations camera_observations;
  int num_observations;
  int total_observations=0;
  double fx,fy,cx,cy;
  double k1,k2,k3,p1,p2;

  fx = camera_->camera_parameters_.focal_length_x;
  fy = camera_->camera_parameters_.focal_length_y;
  cx = camera_->camera_parameters_.center_x;
  cy = camera_->camera_parameters_.center_y;
  k1 = camera_->camera_parameters_.distortion_k1;
  k2 = camera_->camera_parameters_.distortion_k2;
  k3 = camera_->camera_parameters_.distortion_k3;
  p1 = camera_->camera_parameters_.distortion_p1;
  p2 = camera_->camera_parameters_.distortion_p2;;

  // set the roi to the whole image
  Roi roi;
  roi.x_min = 0;
  roi.y_min = 0;
  roi.x_max = image_width_;
  roi.y_max = image_height_;
  industrial_extrinsic_cal::Cost_function cost_type;
  Problem problem;
  Pose6d pose;

  // set initial conditions,
  pose.setQuaternion(qx_, qy_, qz_, qw_);
  pose.setOrigin(0.011, 0.05, D0_-rail_position);
  pose.show("initial pose");

  camera_->camera_observer_->clearObservations();
  camera_->camera_observer_->clearTargets();
  camera_->camera_observer_->addTarget(target_, roi, cost_type);
  camera_->camera_observer_->triggerCamera();
  while (!camera_->camera_observer_->observationsDone()) ;
  if(camera_->camera_observer_->getObservations(camera_observations)){
    ROS_INFO("Found %d observations",(int) camera_observations.size());
    num_observations = (int) camera_observations.size();
    if(num_observations != target_rows_* target_cols_){
      ROS_ERROR("Target Locator could not find target %d", num_observations);
      P = pose;
      return(false);
    }

    // add a new cost to the problem for each observation
    CostFunction* cost_function[num_observations];
    total_observations += num_observations;
    for(int i=0; i<num_observations; i++){
      double image_x = camera_observations[i].image_loc_x;
      double image_y = camera_observations[i].image_loc_y;
      Point3d point = target_->pts_[camera_observations[i].point_id]; // don't assume ordering from camera observer
      //    ROS_INFO("%lf %lf %lf %lf %lf",image_x, image_y, point.x, point.y, point.z);
      //    cost_function[i] =  industrial_extrinsic_cal::CameraReprjErrorPK::Create(image_x, image_y, fx, fy, cx, cy, point);
      cost_function[i] =  industrial_extrinsic_cal::DistortedCameraFinder::Create(image_x, image_y, fx, fy, cx, cy, k1, k2, k3, p1, p2, point);
      problem.AddResidualBlock(cost_function[i], NULL, pose.pb_pose);
    }
  }

  // set up and solve the problem
  Solver::Options options;
  Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 2000;
  ceres::Solve(options, &problem, &summary);
  if(summary.termination_type != ceres::NO_CONVERGENCE){
    double initial_cost = summary.initial_cost/total_observations;
    double final_cost = summary.final_cost/total_observations;
    if(final_cost <= allowable_cost_per_observation_){
      ROS_INFO("Found Pose, initial cost = %lf, final cost = %lf", initial_cost, final_cost);
      pose.show("Returned Pose");
      P = pose;
      return true;
    }
    else{
      ROS_ERROR("finding pose: allowable cost exceeded %f > %f", final_cost, allowable_cost_per_observation_);
      P = pose;
      return(false);
    }
  }
  else{
    ROS_ERROR("finding pose: No Convergence");
    P = pose;
    return(false);
  }
}

void RobocylCalService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{
  target_ =  shared_ptr<industrial_extrinsic_cal::Target>(new industrial_extrinsic_cal::Target());
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
/*
void RobocylCalService::initMCircleTarget(int rows, int cols, double circle_dia, double spacing)
{

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
  for(int i=rows-1; i>=0; i--){
    for(int j=0; j<cols; j++){
      Point3d point;
      point.x = j*spacing;
      point.y = i*spacing;
      point.z = 0.0;
      target_->pts_.push_back(point);
    }
  }
}
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rail_cal_service");
  ros::NodeHandle node_handle;
  RobocylCalService rail_cal(node_handle);


  ros::spin();
  ros::waitForShutdown();
  return 0;
}
