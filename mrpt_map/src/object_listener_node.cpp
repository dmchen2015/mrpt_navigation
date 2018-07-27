#include <object_listener_node.h>
#include <mrpt_bridge/map.h>
#include <mrpt/maps/CBearing.h>
#include <mrpt/maps/CBearingMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/io/CFileOutputStream.h>


#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/stock_objects.h>


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
  n_param_.param<bool>(std::string("load_map"), load_map_, false);
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

  if (load_map_)
  {
    ROS_INFO("loading and displaying map.");
  }
  else
  {
    ROS_INFO("listening to objects which are stored in the provided map file.");
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
  sub_object_detections_ = n_.subscribe("map_doors", 1, &ObjectListenerNode::callbackObjectDetections, this);
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
      const auto o_id = it->object.ids[0];
      mrpt::maps::CBearing::Ptr bear;
      mrpt::maps::CBearingMap::Ptr bearings = metric_map_->m_bearingMap;
//      std::vector<mrpt::maps::CBearing::Ptr>::iterator it_b = std::find_if(bearings->begin(),bearings->end(),
//                                                       [&o_id] (const mrpt::maps::CBearing::Ptr b)
//                                                                  {
//                                                                      return b->m_ID == o_id;
//                                                                  });
      auto it_b = bearings->begin();
      for (; it_b != bearings->end(); ++it_b)
      {
        if ((*it_b)->m_ID == o_id)
        {
          break;
        }
      }
      if (it_b != bearings->end())
      {
        bear = *it_b; // update
      }
      else
      {
        bear = mrpt::maps::CBearing::Create(); //create
      }
      const auto &position = it->object.pose.position;
      const auto &orientation = it->object.pose.orientation;

      Eigen::Quaterniond q(orientation.w,orientation.x,orientation.y,orientation.z);
      auto qeuler = q.toRotationMatrix().eulerAngles(0,1,2); //roll, pitch, yaw

      bear->m_fixed_pose.setFromValues(position.x, position.y, position.z,qeuler[2],qeuler[1],qeuler[0]);
      bear->m_ID = o_id;
      bear->m_typePDF = mrpt::maps::CBearing::pdfNO;
      if (it_b == bearings->end())
      {
        bearings->push_back(bear);
      }
    }
  }
}

void ObjectListenerNode::display()
{
  MRPT_START

  using namespace mrpt::opengl;
  if (!window_)
  {
    window_ = mrpt::gui::CDisplayWindow3D::Create("Constructed Map",500,500);
    window_->setCameraZoom(40);
    window_->setCameraAzimuthDeg(-50);
    window_->setCameraElevationDeg(70);
  }
  COpenGLScene::Ptr scene;
  scene = mrpt::make_aligned_shared<COpenGLScene>();
  mrpt::opengl::CGridPlaneXY::Ptr groundPlane =
    mrpt::make_aligned_shared<mrpt::opengl::CGridPlaneXY>(
      -200, 200, -200, 200, 0, 5);
  groundPlane->setColor(0.4, 0.4, 0.4);
  scene->insert(groundPlane);

  mrpt::opengl::CSetOfObjects::Ptr objs =
    mrpt::make_aligned_shared<
      mrpt::opengl::CSetOfObjects>();
  metric_map_->getAs3DObject(objs);
  scene->insert(objs);

  COpenGLScene::Ptr &scenePtr = window_->get3DSceneAndLock();
  scenePtr = scene;
  window_->unlockAccess3DScene();
  window_->forceRepaint();
  ROS_INFO("repainting scene");

  MRPT_END
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_listener_node");
  ros::NodeHandle nh;

  ObjectListenerNode obj_listener_node(nh);
  obj_listener_node.init();
  ros::Rate rate(2);

  while (ros::ok())
  {
    ros::spinOnce();
    obj_listener_node.display();
    rate.sleep();
  }

  ROS_INFO("Hello world!");
}
