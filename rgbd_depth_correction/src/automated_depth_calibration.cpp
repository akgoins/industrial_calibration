#include <ros/ros.h>
#include <robo_cylinder/MoveMeters.h>
#include <std_srvs/Empty.h>

//TODO make this more generic, possible push to robo cylinder
std::vector<float> generate_points(float start, float end, int steps){
    std::vector<float> points;
    float distance = std::abs(end-start);
    ROS_INFO_STREAM(distance);
    for(int i=1; i<=steps; i++){
        points.push_back(start-distance/float(steps)*float(i));
        ROS_INFO_STREAM(start-distance/float(steps)*float(i));
    }
    return points;
}

//TODO Timeouts for rail
//TODO wait for rail service
int main(int argc, char  *argv[])
{
  ros::init(argc, argv, "automated_calibration");

  ros::NodeHandle pnh("~");

  //make sure that the nessesary services are available


  float start_rail_location;
  float end_rail_location;
  int data_points;
  //read in params from launch file
  if (!(pnh.getParam("start_rail_location", start_rail_location) &&
        pnh.getParam("end_rail_location", end_rail_location) &&
        pnh.getParam("data_points", data_points)
        )){
      ROS_ERROR_STREAM("Required parameters are not set, shutting down automated calibration node :(");
      ros::shutdown();
  }



  //wait 20 seconds for robo cylinder to launch
  ros::service::waitForService("/move_meters", ros::Duration(20));

  ros::NodeHandle nh;

  ros::ServiceClient move_rail_client = nh.serviceClient<robo_cylinder::MoveMeters>("/move_meters");
  ros::ServiceClient start_depth_calibration_client = nh.serviceClient<std_srvs::Empty>("/pixel_depth_calibration");
  ros::ServiceClient add_calibration_data_client = nh.serviceClient<std_srvs::Empty>("/store_cloud");
  ros::ServiceClient solve_calibration_client = nh.serviceClient<std_srvs::Empty>("/depth_calibration");

  //move to start position
  robo_cylinder::MoveMeters move_service;
  move_service.request.meters = (start_rail_location-end_rail_location)/2;
  while(1){
      if(move_rail_client.call(move_service)){
          std_srvs::Empty start_cal_service;
          ROS_INFO_STREAM("Starting calibration service and collecting mid point point cloud at " << move_service.request.meters);
          if(start_depth_calibration_client.call(start_cal_service))
              break; //this means the rail is at the position and the call work so continue on after this while(1)
          else{
              ROS_ERROR_STREAM("Start depth calibration call failed. Shutting down :(");
              ros::shutdown();
          }
      }
  }

  std::vector<float> rail_locations = generate_points(start_rail_location, end_rail_location, data_points);

  for (std::vector<float>::iterator it = rail_locations.begin() ; it != rail_locations.end(); ++it){
    ROS_INFO_STREAM("commanding rail to  " << *it);
    while(1){
        robo_cylinder::MoveMeters move_service;
        move_service.request.meters = *it;
        if(move_rail_client.call(move_service)){
            std_srvs::Empty add_cal_service;
          if(add_calibration_data_client.call(add_cal_service))
            break;
          else{
            ROS_ERROR_STREAM("add depth calibration call failed. Shutting down :(");
            ros::shutdown();
          }
        }
    }
  }

  ROS_INFO_STREAM("All data collected.  Calculating Calibration.");
  std_srvs::Empty solve_cal_service;
  if(!solve_calibration_client.call(solve_cal_service)){
    ROS_ERROR_STREAM("Solve Calibration Service call failed. Shutting down :(");
    //ros::shutdown();
  }
  else
    ROS_INFO_STREAM("Calibration calculation complete.");


  ros::spin();

  return 0;
}
