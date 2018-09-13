#include "timer.hpp"
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>


ros::Publisher path_pub;
using namespace JPS;

nav_msgs::Path plan(char* map){
  
  // Read the map from yaml
  MapReader<Vec3i, Vec3f> reader(map, true); // Map read from a given file
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, map);
  }


  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time();
  
  // store map in map_util
  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());

  const Vec3f start(reader.start(0), reader.start(1), reader.start(2));
  const Vec3f goal(reader.goal(0), reader.goal(1), reader.goal(2));

  std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); // Declare a planner
  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->updateMap();

  planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS

  auto path_jps = planner_ptr->getRawPath();
  for(const auto& it: path_jps){
	geometry_msgs::PoseStamped posestamp;
	posestamp.header.frame_id = "map";
	posestamp.header.stamp = ros::Time();
	posestamp.pose.position.x = it.transpose()[0];
	posestamp.pose.position.y = it.transpose()[1];
	posestamp.pose.position.z = it.transpose()[2];
	path.poses.push_back(posestamp);
  }
  return path;
  
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan3d_to_path");
    ros::NodeHandle nh("~");
    path_pub = nh.advertise<nav_msgs::Path>("path", 1);
	if(argc != 2) {
		printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
		return -1;
	}
	nav_msgs::Path path = plan(argv[1]);
	while (ros::ok()){
		  path_pub.publish(path);
		  ros::Duration(5).sleep();
	}
		//~ ros::spinOnce();
    return 0;
}
