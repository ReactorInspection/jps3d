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
#include <visualization_msgs/Marker.h>



ros::Publisher path_pub;
ros::Publisher dmp_path_pub;
geometry_msgs::PoseStamped ps_start;
geometry_msgs::PoseStamped ps_goal;
bool replan_flag;

using namespace JPS;

void waypointsCallback(const nav_msgs::Path::ConstPtr& msg){
	ROS_INFO("waypoints received");
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

std::shared_ptr<JPSPlanner3D> setUpJPS(const std::shared_ptr<VoxelMapUtil> &map_util){
  std::shared_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); // Declare a planner
  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->updateMap();
  return planner_ptr;
}

std::shared_ptr<DMPlanner3D> setUpDMP(const std::shared_ptr<VoxelMapUtil> &map_util){
  std::shared_ptr<DMPlanner3D> dmplanner_ptr(new DMPlanner3D(true)); // Declare a planner
  dmplanner_ptr->setMap(map_util,Vec3f(0.0, 0.0, 0.0)); // Set collision checking function
  //~ dmplanner_ptr->updateMap();
  dmplanner_ptr->setPotentialRadius(Vec3f(20.5, 20.5, 20.5)); // Set 2D potential field radius
  dmplanner_ptr->setSearchRadius(Vec3f(1.5, 1.5, 1.5)); // Set the valid search region around given path
  return dmplanner_ptr;
}



std::vector<nav_msgs::Path> do_planning(const std::shared_ptr<JPSPlanner3D> &planner_ptr , const std::shared_ptr<DMPlanner3D> &dmplanner_ptr){
  nav_msgs::Path path;
  std::vector<nav_msgs::Path> paths;
  path.header.frame_id = "world";
  path.header.stamp = ros::Time();
  
  if (ps_goal.pose.position.x == ps_start.pose.position.x && ps_goal.pose.position.y == ps_start.pose.position.y && ps_goal.pose.position.z == ps_start.pose.position.z){
	   ROS_ERROR("start and goal poses are the same. Have you published waypoints?");
	   return paths;
   }
   else ROS_INFO("running planner");

  //~ // give goals in the map frame, but give to planner in the yaml frame
  tf::StampedTransform transform;
  tf::TransformListener listener;

  try{
	listener.waitForTransform("yaml", "map", ros::Time(0), ros::Duration(2));
	listener.waitForTransform("yaml", "world", ros::Time(0), ros::Duration(2));
	listener.lookupTransform("yaml", "map", ros::Time(0), transform);
	listener.lookupTransform("yaml", "world", ros::Time(0), transform);
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
  for(const auto& it: path_jps){
	posestamp.header.stamp = ros::Time();
    posestamp.header.frame_id = "yaml";
	posestamp.pose.position.x = it.transpose()[0];
	posestamp.pose.position.y = it.transpose()[1];
	posestamp.pose.position.z = it.transpose()[2];
	posestamp.pose.orientation.w = 1;
	posestamp.pose.orientation.x = 0;
	posestamp.pose.orientation.y = 0;
	posestamp.pose.orientation.z = 0;
	listener.transformPose("world",posestamp, posestamp);
	path.poses.push_back(posestamp);
  }
  paths.push_back(path);

  // Run DMP planner
  dmplanner_ptr->computePath(start, goal, path_jps); // Compute the path given the jps path
  const auto path_dmp = dmplanner_ptr->getPath();
  path.poses.clear();
  for(const auto& it: path_dmp){
	posestamp.header.stamp = ros::Time();
    posestamp.header.frame_id = "yaml";
	posestamp.pose.position.x = it.transpose()[0];
	posestamp.pose.position.y = it.transpose()[1];
	posestamp.pose.position.z = it.transpose()[2];
	posestamp.pose.orientation.w = 1;
	posestamp.pose.orientation.x = 0;
	posestamp.pose.orientation.y = 0;
	posestamp.pose.orientation.z = 0;
	listener.transformPose("world",posestamp, posestamp);
	path.poses.push_back(posestamp);
  }  	
  paths.push_back(path);

  return paths;
  
}

ros::Publisher marker_pub;

void checkMap(const std::shared_ptr<VoxelMapUtil> &map_util){
	ROS_INFO("checking map");
	const Vec3i dim = map_util->getDim();
	const double res = map_util->getRes();
	visualization_msgs::Marker marker;
	marker.header.frame_id = "yaml";
	marker.header.stamp = ros::Time();
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	marker.scale.x = res;
	marker.scale.y = res;
	marker.scale.z = res;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	for(int x = 0; x < dim(0); x ++) {
		for(int y = 0; y < dim(1); y ++) {
			for(int z = 0; z < dim(2); z ++) {
			  if(!map_util->isFree(Vec3i(x, y,z))) {
				Vec3f pt = map_util->intToFloat(Vec3i(x, y,z));
				geometry_msgs::Point point;
				point.x = pt(0);
				point.y = pt(1);
				point.z = pt(2);
				marker.points.push_back(point);
	}}}}

    marker_pub.publish( marker );
    ROS_INFO("finished checking map");
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "plan3d_to_path");
    ros::NodeHandle nh("~");
    path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    dmp_path_pub = nh.advertise<nav_msgs::Path>("dmp_path", 1);
	ros::Subscriber sub = nh.subscribe("/waypoints", 1000, waypointsCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("map_check", 1);

	if(argc != 2) {
		printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
		return -1;
	}
	
	std::shared_ptr<VoxelMapUtil> map_util = readMap(argv[1]);
	std::vector<int8_t> cmap_ = map_util->getMap();
	std::shared_ptr<JPSPlanner3D> planner_ptr = setUpJPS(map_util);
	std::shared_ptr<DMPlanner3D> dmplanner_ptr = setUpDMP(map_util);
	std::vector<nav_msgs::Path> paths;
	replan_flag = false;
	checkMap(map_util);
	while (ros::ok()){
		  ros::spinOnce();
		  if (replan_flag){
			//~ map_util = readMap(argv[1]);
			dmplanner_ptr = setUpDMP(map_util);

			paths = do_planning(planner_ptr, dmplanner_ptr);
			replan_flag = false;
		  }
		  if (!paths.empty()){
			  path_pub.publish(paths[0]);
			  dmp_path_pub.publish(paths[1]);
		  }
		  ros::Duration(1).sleep();
	}
    return 0;
}
