#include "timer.hpp"
#include "read_map.hpp"
#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <nav_msgs/Path.h>


using namespace JPS;

int main(int argc, char ** argv){
  if(argc != 2) {
    printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
    return -1;
  }

  // Read the map from yaml
  MapReader<Vec3i, Vec3f> reader(argv[1], true); // Map read from a given file
  if(!reader.exist()) {
    printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
    return -1;
  }

  // store map in map_util
  std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
  map_util->setMap(reader.origin(), reader.dim(), reader.data(), reader.resolution());

  const Vec3f start(reader.start(0), reader.start(1), reader.start(2));
  const Vec3f goal(reader.goal(0), reader.goal(1), reader.goal(2));

  std::unique_ptr<JPSPlanner3D> planner_ptr(new JPSPlanner3D(true)); // Declare a planner
  planner_ptr->setMapUtil(map_util); // Set collision checking function
  planner_ptr->updateMap();

  bool valid_jps = planner_ptr->plan(start, goal, 1, true); // Plan from start to goal using JPS
  printf("JPS Path Distance: %f\n", total_distance3f(planner_ptr->getRawPath()));
  printf("JPS Path: \n");
  auto path_jps = planner_ptr->getRawPath();
  for(const auto& it: path_jps)
    std::cout << it.transpose() << std::endl;

  return 0;
}
