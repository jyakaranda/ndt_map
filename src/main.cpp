#include "ndt_map/ndt_map.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_node");

  ros::NodeHandle nh, pnh("~");
  NDTMap *map = new NDTMap(nh, pnh);

  std::thread visual_thread(&NDTMap::visualThread, map);
  std::thread loop_thread(&NDTMap::loopClosureThread, map);

  map->run();

  ros::spin();

  return 0;
}