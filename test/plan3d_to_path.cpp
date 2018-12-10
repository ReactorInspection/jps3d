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
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <pluto_msgs/VoxelMap.h>

using namespace JPS;


ros::Publisher path_pub;
ros::Publisher dmp_path_pub;
ros::Publisher marker_pub;
geometry_msgs::PoseStamped ps_start;
geometry_msgs::PoseStamped ps_goal;
bool replan_flag;
nav_msgs::Odometry odom;
std::shared_ptr<VoxelMapUtil> map_util;
bool map_initialized_ = false;
bool yaml_map = false;
std::string robot_name, gazebo_frame, mapper_frame, planning_frame;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	odom = *msg;
}
void waypointsCallback(const nav_msgs::Path::ConstPtr& msg){
	ROS_INFO("waypoints received");
	if (msg->poses.size()>2) ROS_WARN("first 2 waypoints treated as start, goal. have not yet implemented handling additional points");
	if (msg->poses.size()<1) {
		ROS_ERROR("must publish at least one waypoint");
		return;
	}
	if (msg->poses.size()==1){
		ps_start.pose = odom.pose.pose;
		ps_start.header.frame_id = odom.header.frame_id;
		ps_goal = msg->poses[0];
		ps_goal.header.frame_id = msg->header.frame_id;
		ROS_INFO("%f %f %f",ps_start.pose.position.x,ps_start.pose.position.y,ps_start.pose.position.z);
	}
	else{
		ps_start = msg->poses[0];
		ps_start.header.frame_id = msg->header.frame_id;
		ps_goal = msg->poses[1];
		ps_goal.header.frame_id = msg->header.frame_id;
	}
	replan_flag = true;
}

void checkMap(){
	ROS_INFO("checking map");
	const Vec3i dim = map_util->getDim();
	const double res = map_util->getRes();
	visualization_msgs::Marker marker;
	marker.header.frame_id = planning_frame;
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
}

void mapCallback(const pluto_msgs::VoxelMap::ConstPtr& msg) {
	pluto_msgs::VoxelMap map_ = *msg;
	Vec3f ori(map_.origin.x, map_.origin.y, map_.origin.z);
	Vec3i dim(map_.dim.x, map_.dim.y, map_.dim.z);
	decimal_t res = map_.resolution;
	std::vector<signed char> map = map_.data;
	map_util->setMap(ori, dim, map, res);
	if (!map_initialized_){
		map_initialized_ = true;
		ROS_INFO("Map initialized!");
	}
	else ROS_INFO("Map updated");
	checkMap();

}

void readMap(char* map){
  // Read the map from yaml
  ROS_INFO("reading map (takes a few seconds)");
  MapReader<Vec3i, Vec3f> reader(map, true); // Map read from a given file
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, map);
  }
  
  // store map in map_util
  //~ std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  map_initialized_ = true;
  ROS_INFO("Map initialized!");

}

	
std::shared_ptr<JPSPlanner3D> setUpJPS(){
  std::shared_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); // Declare a planner
  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->updateMap();
  return planner_ptr;
}

std::shared_ptr<DMPlanner3D> setUpDMP(){
  std::shared_ptr<DMPlanner3D> dmplanner_ptr(new DMPlanner3D(true)); // Declare a planner
  dmplanner_ptr->setMap(map_util,Vec3f(0.0, 0.0, 0.0)); // Set collision checking function
  //~ dmplanner_ptr->updateMap();
  dmplanner_ptr->setPotentialRadius(Vec3f(2.5, 2.5, 2.5)); // Set 3D potential field radius
  dmplanner_ptr->setSearchRadius(Vec3f(1.5, 1.5, 1.5)); // Set the valid search region around given path
  return dmplanner_ptr;
}

nav_msgs::Path create_path(auto planner_path, tf::TransformListener &listener){
  geometry_msgs::PoseStamped posestamp;
  nav_msgs::Path path;
  path.header.frame_id = gazebo_frame;
  path.header.stamp = ros::Time();

  for(const auto& it: planner_path){
	posestamp.header.stamp = ros::Time();
    posestamp.header.frame_id = planning_frame;
	posestamp.pose.position.x = it.transpose()[0];
	posestamp.pose.position.y = it.transpose()[1];
	posestamp.pose.position.z = it.transpose()[2];
	posestamp.pose.orientation.w = 1;
	posestamp.pose.orientation.x = 0;
	posestamp.pose.orientation.y = 0;
	posestamp.pose.orientation.z = 0;
	listener.transformPose(gazebo_frame,posestamp, posestamp);
	// linear interpolation to add points to path for ewok to run better
	if (path.poses.size()>0){
		float diff_x = posestamp.pose.position.x - path.poses.back().pose.position.x;
		float diff_y = posestamp.pose.position.y - path.poses.back().pose.position.y;
		float diff_z = posestamp.pose.position.z - path.poses.back().pose.position.z;
		int num_steps = int(sqrt(diff_x*diff_x+diff_y*diff_y+diff_z*diff_z)/0.5);
		geometry_msgs::PoseStamped interm_pt = posestamp;
		geometry_msgs::PoseStamped last_pt = path.poses.back();
		for (int i = 1; i < num_steps; i++){
			interm_pt.pose.position.x = diff_x*i/num_steps + last_pt.pose.position.x;
			interm_pt.pose.position.y = diff_y*i/num_steps + last_pt.pose.position.y;
			interm_pt.pose.position.z = diff_z*i/num_steps + last_pt.pose.position.z;
			path.poses.push_back(interm_pt);
		}
	}
	path.poses.push_back(posestamp);
  }
  return path;  	

}

std::vector<nav_msgs::Path> do_planning(const std::shared_ptr<JPSPlanner3D> &planner_ptr , const std::shared_ptr<DMPlanner3D> &dmplanner_ptr){

  std::vector<nav_msgs::Path> paths;
    
  if (ps_goal.pose.position.x == ps_start.pose.position.x && ps_goal.pose.position.y == ps_start.pose.position.y && ps_goal.pose.position.z == ps_start.pose.position.z){
	   ROS_ERROR("start and goal poses are the same. Have you published waypoints?");
	   return paths;
   }
   else ROS_INFO("running planner");

  tf::StampedTransform transform;
  tf::TransformListener listener;


  try{
	listener.waitForTransform(mapper_frame, planning_frame, ros::Time(0), ros::Duration(1));
	listener.waitForTransform(gazebo_frame, planning_frame, ros::Time(0), ros::Duration(1));
	listener.transformPose(planning_frame,ros::Time(0),ps_start,ps_start.header.frame_id, ps_start);
	listener.transformPose(planning_frame,ros::Time(0),ps_goal,ps_goal.header.frame_id, ps_goal);
   }
   catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
	 return paths;
   }

	const Vec3f start(ps_start.pose.position.x,ps_start.pose.position.y,ps_start.pose.position.z);
	const Vec3f goal(ps_goal.pose.position.x,ps_goal.pose.position.y,ps_goal.pose.position.z);

  // Run JPS Planner
  planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS
  auto path_jps = planner_ptr->getRawPath();
  paths.push_back(create_path(path_jps,listener));

  // Run DMP planner
  dmplanner_ptr->computePath(start, goal, path_jps); // Compute the path given the jps path
  const auto path_dmp = dmplanner_ptr->getPath();
  paths.push_back(create_path(path_dmp,listener));
  
  return paths;
  
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "plan3d_to_path");
    ros::NodeHandle nh("~");
    path_pub = nh.advertise<nav_msgs::Path>("jps_path", 1);
    dmp_path_pub = nh.advertise<nav_msgs::Path>("dmp_path", 1);
    std::string waypoint_topic, voxel_topic;
	ros::param::param<std::string>("waypoint_topic", waypoint_topic, "/waypoints");
    ros::param::param<std::string>("voxel_topic", voxel_topic, "/cloud_to_map/voxel_map");

	ros::Subscriber sub = nh.subscribe(waypoint_topic, 1000, waypointsCallback);
	nh.param<std::string>("robot_name", robot_name, "ddk");
	ros::Subscriber odom_sub = nh.subscribe("/" + robot_name + "/ground_truth/odom", 10, odomCallback);
	ros::Subscriber map_sub = nh.subscribe(voxel_topic, 2, mapCallback);
	map_util = std::make_shared<VoxelMapUtil>();

    marker_pub = nh.advertise<visualization_msgs::Marker>("map_check", 1);
	
	//~ We do planning in the frame of the yaml file or VoxelMap (not octomap)
	//~ For the yaml case, there are 3 frames, world (gazebo), map (octomap), and yaml (written map file, has to be shifted if the octomap has any negative values)
	//~ There is also a base_link for the robot local frame, but it is not needed by the planning (we use the ground_truth/odom topic in the world frame)
	//~ For the VoxelMap taken in by subscription case, there is the world (gazebo) and the map (local frame to the robot created by e.g. SLAM)

    nh.param<std::string>("gazebo_frame", gazebo_frame, "world");
    nh.param<std::string>("mapper_frame", mapper_frame, "map");
	if(argc == 2) {
		ROS_INFO("Using yaml map");
		readMap(argv[1]);
		yaml_map = true;
	}
	if (yaml_map) planning_frame = "yaml";
	else planning_frame = mapper_frame;

    while(ros::ok() && !map_initialized_) {
		ROS_WARN_ONCE("Map not initialized!");
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}

	std::shared_ptr<JPSPlanner3D> planner_ptr = setUpJPS();
	std::shared_ptr<DMPlanner3D> dmplanner_ptr = setUpDMP();
	std::vector<nav_msgs::Path> paths;
	replan_flag = false;
	checkMap();
	while (ros::ok()){
		  ros::spinOnce();
		  if (replan_flag){
			dmplanner_ptr = setUpDMP();
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
