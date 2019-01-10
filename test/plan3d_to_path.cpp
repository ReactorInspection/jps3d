#include "timer.hpp"
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <jps_planner/distance_map_planner/distance_map_planner.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <pluto_msgs/VoxelMap.h>

using namespace JPS;

class Plan3DToPath
{
public:
  Plan3DToPath();
private:
  void setUpJPS();
  void setUpDMP();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void mapCallback(const pluto_msgs::VoxelMap::ConstPtr& msg);
  void waypointsCallback(const nav_msgs::Path::ConstPtr& msg);
  void readMap(std::string map_file);
  void publishMap();
  void do_planning(geometry_msgs::PoseStamped& ps_start, geometry_msgs::PoseStamped& ps_goal, std::vector<nav_msgs::Path>& paths);
  void create_path(vec_Vec3f planner_path, nav_msgs::Path& path, const tf::StampedTransform& transform, const ros::Time current_time);

  ros::NodeHandle nh_, pnh_;

  ros::Publisher path_pub_, dmp_path_pub_, map_marker_pub_;
  ros::Subscriber map_sub_, odom_sub_, waypoints_sub_;
  nav_msgs::Odometry odom_;

  tf::TransformListener listener_;

  std::shared_ptr<VoxelMapUtil> map_util_;
  bool map_initialized_;
  bool yaml_map_;
  std::string yaml_file_;
  std::string world_frame_, planning_frame_;

  std::unique_ptr<JPSPlanner3D> planner_ptr_;
  std::unique_ptr<DMPlanner3D> dmplanner_ptr_;

  //mutex for changes of variables
  boost::mutex map_mutex_, odom_mutex_;
};

Plan3DToPath::Plan3DToPath()
{
  pnh_ = ros::NodeHandle("~");

  pnh_.param("yaml_map", yaml_map_, false);
  pnh_.param<std::string>("yaml_file", yaml_file_, "map.yaml");

  //~ We do planning in the frame of the yaml file or VoxelMap (not octomap)
  //~ For the yaml case, there are 3 frames, world (gazebo), map (octomap), and yaml (written map file, has to be shifted if the octomap has any negative values)
  //~ There is also a base_link for the robot local frame, but it is not needed by the planning (we use the ground_truth/odom topic in the world frame)
  //~ For the VoxelMap taken in by subscription case, there is the world (gazebo) and the map (local frame to the robot created by e.g. SLAM)
  pnh_.param<std::string>("world_frame", world_frame_, "/world");

  map_initialized_ = false;

  if(yaml_map_)
  {
    ROS_WARN("Using yaml map %s", yaml_file_.c_str());
    readMap(yaml_file_);
    publishMap();
  }

  map_util_ = std::make_shared<VoxelMapUtil>();

  //#TODO Setup planners, map_utils_ needs to be initialized, maybe init with empty?
  //setUpJPS();
  //setUpDMP();

  path_pub_ = nh_.advertise<nav_msgs::Path>("jps_path", 1);
  dmp_path_pub_ = nh_.advertise<nav_msgs::Path>("dmp_path", 1);
  map_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("map_check", 1);

  if(!yaml_map_)
    map_sub_ = nh_.subscribe<pluto_msgs::VoxelMap>("rb_to_voxel_map", 1, boost::bind(&Plan3DToPath::mapCallback, this, _1));

  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&Plan3DToPath::odomCallback, this, _1));
  waypoints_sub_ = nh_.subscribe<nav_msgs::Path>("waypoints", 2, boost::bind(&Plan3DToPath::waypointsCallback,  this, _1));

}

void Plan3DToPath::setUpJPS()
{
  planner_ptr_.reset(new JPSPlanner3D(true)); // Declare a planner
  planner_ptr_->setMapUtil(map_util_); // Set collision checking function
  planner_ptr_->updateMap();
}

void Plan3DToPath::setUpDMP()
{
  dmplanner_ptr_.reset(new DMPlanner3D(true)); // Declare a planner
  dmplanner_ptr_->setMap(map_util_,Vec3f(0.0, 0.0, 0.0)); // Set collision checking function
  //~ dmplanner_ptr->updateMap();
  dmplanner_ptr_->setPotentialRadius(Vec3f(2.5, 2.5, 2.5)); // Set 3D potential field radius
  dmplanner_ptr_->setSearchRadius(Vec3f(1.5, 1.5, 1.5)); // Set the valid search region around given path
}

void Plan3DToPath::readMap(std::string map_file)
{
  // Read the map from yaml
  ROS_INFO("reading map (takes a few seconds)");
  MapReader<Vec3i, Vec3f> reader(map_file.c_str(), true);
  if(!reader.exist())
  {
    ROS_ERROR("Cannot read input file [%s]!",map_file.c_str());
    return;
  }

  // store map in map_util_
  map_util_->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());
  planning_frame_ = "yaml";
  map_initialized_ = true;
  ROS_INFO("Map initialized!");
}

void Plan3DToPath::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(odom_mutex_);
  odom_ = *msg;
}

void Plan3DToPath::mapCallback(const pluto_msgs::VoxelMap::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(map_mutex_);

  pluto_msgs::VoxelMap vo_map = *msg;
  Vec3f ori(vo_map.origin.x, vo_map.origin.y, vo_map.origin.z);
  Vec3i dim(vo_map.dim.x, vo_map.dim.y, vo_map.dim.z);
  decimal_t res = vo_map.resolution;
  std::vector<signed char> map = vo_map.data;
  map_util_->setMap(ori, dim, map, res);
  planning_frame_ = vo_map.header.frame_id;
  if (!map_initialized_)
  {
    map_initialized_ = true;
    ROS_INFO("Map initialized! frame %s", planning_frame_.c_str());
  }
  publishMap();
}

void Plan3DToPath::waypointsCallback(const nav_msgs::Path::ConstPtr& msg)
{
  geometry_msgs::PoseStamped ps_start, ps_goal;
  int wp_size = msg->poses.size();

  ROS_INFO("%d waypoints received", wp_size);
  if(wp_size > 2)
    ROS_WARN("First 2 waypoints treated as start, goal. have not yet implemented handling additional points");
  if(wp_size < 1)
  {
    ROS_ERROR("Must publish at least one waypoint");
    return;
  }
  if(wp_size == 1)
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    ps_start.pose = odom_.pose.pose;
    ps_start.header.frame_id = odom_.header.frame_id;
    ps_goal = msg->poses[0];
    ps_goal.header.frame_id = msg->header.frame_id;
    ROS_INFO("%f %f %f %s",ps_start.pose.position.x,ps_start.pose.position.y,ps_start.pose.position.z, ps_start.header.frame_id.c_str());
  }
  else
  {
    ps_start = msg->poses[0];
    ps_start.header.frame_id = msg->header.frame_id;
    ps_goal = msg->poses[1];
    ps_goal.header.frame_id = msg->header.frame_id;
  }

  std::vector<nav_msgs::Path> paths;

  boost::mutex::scoped_lock lock(map_mutex_);
  {

  if (!map_initialized_)
    ROS_ERROR("No map initialized");

  //#TODO does the planner needs to be reset everytime?
  setUpJPS();
  setUpDMP();

  do_planning(ps_start, ps_goal, paths);
  }
}

void Plan3DToPath::publishMap()
{
  const Vec3i dim = map_util_->getDim();
  //~ const Vec3f origin = map_util_->getOrigin();
  const double res = map_util_->getRes();
  visualization_msgs::Marker marker;
  marker.header.frame_id = planning_frame_;
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
        if(!map_util_->isFree(Vec3i(x, y,z))) {
        Vec3f pt = map_util_->intToFloat(Vec3i(x, y,z));
        geometry_msgs::Point point;
        point.x = pt(0);
        point.y = pt(1);
        point.z = pt(2);
        marker.points.push_back(point);
  }}}}

  map_marker_pub_.publish(marker);
}

void Plan3DToPath::create_path(vec_Vec3f planner_path, nav_msgs::Path& path, const tf::StampedTransform& transform_p_to_w, const ros::Time current_time)
{
  geometry_msgs::PoseStamped posestamp;
  path.header.frame_id = world_frame_;
  path.header.stamp = current_time;

  Eigen::Affine3d dT_p_w;
  tf::transformTFToEigen(transform_p_to_w, dT_p_w);
  Eigen::Affine3f T_p_w = dT_p_w.cast<float>();

  for(const auto& it: planner_path)
  {
    posestamp.header.stamp = current_time;
    posestamp.header.frame_id = planning_frame_;
    posestamp.pose.orientation.w = 1;
    posestamp.pose.orientation.x = 0;
    posestamp.pose.orientation.y = 0;
    posestamp.pose.orientation.z = 0;

    Eigen::Vector4f p;
    p[0] = it.transpose()[0];
    p[1] = it.transpose()[1];
    p[2] = it.transpose()[2];
    p[3] = 1;
    p = T_p_w * p;
    posestamp.pose.position.x = p[0];
    posestamp.pose.position.y = p[1];
    posestamp.pose.position.z = p[2];

    //posestamp.pose.position.x = it.transpose()[0];
    //posestamp.pose.position.y = it.transpose()[1];
    //posestamp.pose.position.z = it.transpose()[2];
    //listener_.transformPose(world_frame_,posestamp, posestamp);

    // linear interpolation to add points to path for ewok to run better
    if (path.poses.size() > 0)
    {
      float diff_x = posestamp.pose.position.x - path.poses.back().pose.position.x;
      float diff_y = posestamp.pose.position.y - path.poses.back().pose.position.y;
      float diff_z = posestamp.pose.position.z - path.poses.back().pose.position.z;
      int num_steps = int(sqrt(diff_x*diff_x+diff_y*diff_y+diff_z*diff_z)/0.5);
      geometry_msgs::PoseStamped interm_pt = posestamp;
      geometry_msgs::PoseStamped last_pt = path.poses.back();
      for (int i = 1; i < num_steps; i++)
      {
        interm_pt.pose.position.x = diff_x*i/num_steps + last_pt.pose.position.x;
        interm_pt.pose.position.y = diff_y*i/num_steps + last_pt.pose.position.y;
        interm_pt.pose.position.z = diff_z*i/num_steps + last_pt.pose.position.z;
        path.poses.push_back(interm_pt);
      }
    }
    path.poses.push_back(posestamp);
  }
}

void Plan3DToPath::do_planning(geometry_msgs::PoseStamped& ps_start, geometry_msgs::PoseStamped& ps_goal, std::vector<nav_msgs::Path>& paths)
{
  /*
  if (ps_goal.pose.position.x == ps_start.pose.position.x && ps_goal.pose.position.y == ps_start.pose.position.y && ps_goal.pose.position.z == ps_start.pose.position.z)
  {
    ROS_ERROR("Start and goal poses are the same. Have you published waypoints?");
    return;
  }*/
  ROS_INFO("Running planner");

  tf::StampedTransform transform_p_to_w;
  ros::Time current_time = ros::Time::now();
  try
  {
    listener_.waitForTransform(world_frame_, planning_frame_, current_time, ros::Duration(0.5));
    listener_.lookupTransform(world_frame_, planning_frame_, current_time, transform_p_to_w);
    listener_.transformPose(planning_frame_, current_time, ps_start, ps_start.header.frame_id, ps_start);
    listener_.transformPose(planning_frame_, current_time, ps_goal, ps_goal.header.frame_id, ps_goal);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return;
  }

  const Vec3f start(ps_start.pose.position.x,ps_start.pose.position.y,ps_start.pose.position.z);
  const Vec3f goal(ps_goal.pose.position.x,ps_goal.pose.position.y,ps_goal.pose.position.z);

  // Run JPS Planner
  planner_ptr_->plan(start, goal, 1, true); // Plan from start to goal using JPS
  auto path_jps = planner_ptr_->getRawPath();
  nav_msgs::Path nav_path_jps;
  create_path(path_jps, nav_path_jps, transform_p_to_w, current_time);
  paths.push_back(nav_path_jps);

  // Run DMP planner
  dmplanner_ptr_->computePath(start, goal, path_jps); // Compute the path given the jps path
  auto path_dmp = dmplanner_ptr_->getPath();
  nav_msgs::Path nav_path_dmp;
  create_path(path_dmp, nav_path_dmp, transform_p_to_w, current_time);
  paths.push_back(nav_path_dmp);

  if (!paths.empty())
  {
    path_pub_.publish(paths[0]);
    dmp_path_pub_.publish(paths[1]);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan3d_to_path");

  Plan3DToPath p3dp;

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
