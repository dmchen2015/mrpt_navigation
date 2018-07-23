#include <object_listener_node.h>
#include <mrpt_bridge/map.h>
#include <mrpt/maps/CBeaconMap.h>

ObjectListener::ObjectListener(ros::NodeHandle& n) : n_(n)
{}

void ObjectListener::init()
{
  std::string ini_file;
  std::string map_file;
//  n_param_.param<std::string>("ini_file", ini_file, "map.ini");
//  ROS_INFO("ini_file: %s", ini_file.c_str());
  n_param_.param<std::string>("map_file", map_file, "map.simplemap");

  ROS_INFO("map_file: %s", map_file.c_str());

//  if (ini_file == std::string(""))
//  {
//    ROS_ERROR("specify a ini file destination and make sure the directory exists.");
//  }
  if (map_file == std::string(""))
  {
    ROS_ERROR("specify a simplemap file destination and make sure the directory exists.");
  }

  map_file_.open(map_file, std::fstream::out);
  if (!map_file_.is_open()){
    ROS_ERROR("cannot open file: %s, make sure the directory exists.", map_file.c_str());
  }

  sub_map_ = n_.subscribe("map", 1, &ObjectListener::callbackMap, this);
  sub_object_detections_ = n_.subscribe("object_detections", 1, &ObjectListener::callbackObjectDetections, this);
  metric_map_ = boost::make_shared<CMultiMetricMap>();
  //metric_map_->m_gridMaps.push_back(new mrpt::maps::COccupancyGridMap2D());
  //metric_map_->m_beaconMap.push_back(new mrpt::maps::CBeaconMap());
}

void callbackMap(nav_msgs::OccupancyGrid &_msg)
{
  //mrpt_bridge::convert()
}

void callbackObjectDetections(tuw_object_msgs::ObjectDetection &_msg)
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_listener_node");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
