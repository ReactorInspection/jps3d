#include "timer.hpp"
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>



ros::Publisher path_pub;
ros::Publisher dmp_path_pub;
geometry_msgs::PoseStamped ps_start;
geometry_msgs::PoseStamped ps_goal;
bool replan_flag;

using namespace JPS;

void waypointsCallback(const nav_msgs::Path::ConstPtr& msg){
	ROS_INFO("waypoints received. start and/or goal moved");
	if (msg->poses.size()>2) ROS_WARN("first 2 waypoints treated as start, goal. have not yet implemented handling additional points");
	if (msg->poses.size()<2) {
		ROS_ERROR("must publish at least two waypoints");
		return;
	}
	ps_start = msg->poses[0];
	ps_goal = msg->poses[1];
	ps_start.header.frame_id = msg->header.frame_id;
	ps_goal.header.frame_id = msg->header.frame_id;
	replan_flag = true;
}

std::shared_ptr<VoxelMapUtil> readMap(char* map){
  // Read the map from yaml
  ROS_INFO("reading map (takes a few seconds)");
  MapReader<Vec3i, Vec3f> reader(map, true); // Map read from a given file
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, map);
  }
  
  // store map in map_util
  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  return map_util;
}

std::shared_ptr<JPSPlanner3D> setUpJPS(std::shared_ptr<VoxelMapUtil>  map_util){
  std::shared_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); // Declare a planner
  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->updateMap();
  return planner_ptr;
}

std::shared_ptr<DMPlanner3D> setUpDMP(std::shared_ptr<VoxelMapUtil>  map_util){
  std::shared_ptr<DMPlanner3D> dmplanner_ptr(new DMPlanner3D(true)); // Declare a planner
  dmplanner_ptr->setMapUtil(map_util); // Set collision checking function
  dmplanner_ptr->updateMap();
  dmplanner_ptr->setPotentialRadius(Vec3f(1.0, 1.0, 1.0)); // Set 2D potential field radius
  dmplanner_ptr->setSearchRadius(Vec3f(0.5, 0.5, 0.5)); // Set the valid search region around given path
  return dmplanner_ptr;
}



nav_msgs::Path do_planning(std::shared_ptr<JPSPlanner3D> planner_ptr , std::shared_ptr<DMPlanner3D> dmplanner_ptr){
  nav_msgs::Path path;
  path.header.frame_id = "yaml";
  path.header.stamp = ros::Time();
  
  if (ps_goal.pose.position.x == ps_start.pose.position.x && ps_goal.pose.position.y == ps_start.pose.position.y && ps_goal.pose.position.z == ps_start.pose.position.z){
	   ROS_ERROR("start and goal poses are the same. Have you published waypoints?");
	   return path;
   }
   else ROS_INFO("running planner");

  //~ // give goals in the map frame, but give to planner in the yaml frame
  tf::StampedTransform transform;
  tf::TransformListener listener;

  try{
	listener.waitForTransform("yaml", "map", ros::Time(0), ros::Duration(2));
	listener.lookupTransform("yaml", "map", ros::Time(0), transform);
	listener.transformPose("yaml",ps_start, ps_start);
	listener.transformPose("yaml",ps_goal, ps_goal);
   }
   catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
   }
	const Vec3f start(ps_start.pose.position.x,ps_start.pose.position.y,ps_start.pose.position.z);
	const Vec3f goal(ps_goal.pose.position.x,ps_goal.pose.position.y,ps_goal.pose.position.z);


  planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS

  auto path_jps = planner_ptr->getRawPath();
  geometry_msgs::PoseStamped posestamp;
  posestamp.header.frame_id = "yaml";
  for(const auto& it: path_jps){
	posestamp.header.stamp = ros::Time();
	posestamp.pose.position.x = it.transpose()[0];
	posestamp.pose.position.y = it.transpose()[1];
	posestamp.pose.position.z = it.transpose()[2];
	path.poses.push_back(posestamp);
  }
  
  // Run DMP planner
  dmplanner_ptr->computePath(start, goal, path_jps); // Compute the path given the jps path
  const auto path_dmp = dmplanner_ptr->getPath();
  path.poses.clear();
  for(const auto& it: path_dmp){
	posestamp.header.frame_id = "yaml";
	posestamp.header.stamp = ros::Time();
	posestamp.pose.position.x = it.transpose()[0];
	posestamp.pose.position.y = it.transpose()[1];
	posestamp.pose.position.z = it.transpose()[2];
	path.poses.push_back(posestamp);
    dmp_path_pub.publish(path);

  }
  return path;
  
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan3d_to_path");
    ros::NodeHandle nh("~");
    path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    path_pub = nh.advertise<nav_msgs::Path>("dmp_path", 1);
	ros::Subscriber sub = nh.subscribe("/waypoints", 1000, waypointsCallback);

	if(argc != 2) {
		printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
		return -1;
	}
	
	std::shared_ptr<VoxelMapUtil> map_util = readMap(argv[1]);
	std::shared_ptr<JPSPlanner3D> planner_ptr = setUpJPS(map_util);
	std::shared_ptr<DMPlanner3D> dmplanner_ptr = setUpDMP(map_util);
	nav_msgs::Path path;
	replan_flag = true;
	while (ros::ok()){
		  ros::spinOnce();
		  if (replan_flag){
			path = do_planning(planner_ptr, dmplanner_ptr);
			replan_flag = false;
		  }
		  path_pub.publish(path);
		  ros::Duration(1).sleep();
	}
    return 0;
}
