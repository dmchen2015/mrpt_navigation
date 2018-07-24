#include <object_listener_node.h>
#include <mrpt_bridge/map.h>
#include <mrpt/maps/CBearing.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/io/CFileOutputStream.h>


ObjectListenerNode::ObjectListenerNode(ros::NodeHandle& n) : n_(n)
{}

ObjectListenerNode::~ObjectListenerNode()
{
  map_file_.close();
}

void ObjectListenerNode::init()
{
  std::string ini_file;
//  n_param_.param<std::string>("ini_file", ini_file, "map.ini");
//  ROS_INFO("ini_file: %s", ini_file.c_str());
  n_param_.param<std::string>(std::string("simplemap_file"), map_file_path_, std::string(""));
  n_param_.param<std::string>(std::string("bitmap_file"), bitmap_file_path_, std::string(""));

  ROS_INFO("map_file: %s", map_file_path_.c_str());
  ROS_INFO("bitmap_file: %s", bitmap_file_path_.c_str());

//  if (ini_file == std::string(""))
//  {
//    ROS_ERROR("specify a ini file destination and make sure the directory exists.");
//  }
  if (map_file_path_ == std::string(""))
  {
    ROS_ERROR("specify a simplemap file destination and make sure the directory exists.");
  }
  if (bitmap_file_path_ == std::string(""))
  {
    ROS_ERROR("specify a bitmap file destination and make sure the directory exists.");
  }

  map_file_.open(map_file_path_, std::fstream::out);
  if (!map_file_.is_open()){
    ROS_ERROR("cannot open file: %s, make sure the directory exists.", map_file_path_.c_str());
  }
  else
  {
    map_file_.close();
  }

  sub_map_ = n_.subscribe("map", 1, &ObjectListenerNode::callbackMap, this);
  sub_object_detections_ = n_.subscribe("object_detections", 1, &ObjectListenerNode::callbackObjectDetections, this);
  metric_map_ = boost::make_shared<mrpt::maps::CMultiMetricMap>();
  mrpt::containers::deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr> grid_map(mrpt::maps::COccupancyGridMap2D::Create());
  mrpt::containers::deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr> beacon_map(mrpt::maps::CBeaconMap::Create());
  metric_map_->m_gridMaps.push_back(grid_map);
  metric_map_->maps.push_back(beacon_map);
}

void ObjectListenerNode::callbackMap(const nav_msgs::OccupancyGrid &_msg)
{
  ASSERT_(metric_map_->m_gridMaps.size() == 1);
  mrpt_bridge::convert(_msg, *metric_map_->m_gridMaps[0]);
  mrpt::io::CFileOutputStream fileOut(map_file_path_);
  mrpt::serialization::archiveFrom(fileOut) << *metric_map_;
  metric_map_->m_gridMaps[0]->saveAsBitmapFile(bitmap_file_path_);
}

void ObjectListenerNode::callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg)
{
  for (std::vector<tuw_object_msgs::ObjectWithCovariance>::const_iterator it = _msg.objects.begin(); it != _msg.objects.end(); ++it)
  {
    if (it->object.shape == tuw_object_msgs::Object::SHAPE_DOOR)
    {
      mrpt::maps::CBearing bear;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_listener_node");
  ros::NodeHandle nh;

  ObjectListenerNode obj_listener_node(nh);
  obj_listener_node.init();

  while (ros::ok())
  {
    ros::spin();
  }

  ROS_INFO("Hello world!");
}
