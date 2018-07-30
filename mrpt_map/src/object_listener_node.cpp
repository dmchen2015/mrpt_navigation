#include <object_listener_node.h>
#include <mrpt_bridge/map.h>
#include <mrpt/maps/CBearing.h>
#include <mrpt/maps/CBearingMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CFileGZInputStream.h>

#include <mrpt/config/CConfigFile.h>

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/stock_objects.h>

ObjectListenerNode::ObjectListenerNode(ros::NodeHandle& n) : n_(n)
{}

ObjectListenerNode::~ObjectListenerNode()
{}

void ObjectListenerNode::init()
{
  using namespace mrpt::io;

  n_param_.param<std::string>(std::string("simplemap_file"), params_.map_file_path_, std::string(""));
  n_param_.param<std::string>(std::string("bitmap_file"), params_.bitmap_file_path_, std::string(""));
  n_param_.param<std::string>(std::string("ini_file"), params_.ini_file_path_, std::string(""));
  n_param_.param<bool>(std::string("load_map"), params_.load_map_, false);
  n_param_.param<bool>(std::string("update_map"), params_.update_map_, true);
  ROS_INFO("map_file: %s", params_.map_file_path_.c_str());
  ROS_INFO("bitmap_file: %s", params_.bitmap_file_path_.c_str());
  ROS_INFO("ini file: %s", params_.ini_file_path_.c_str());

  if (params_.load_map_)
  {
    ASSERT_FILE_EXISTS_(params_.ini_file_path_);
  }
  ASSERT_FILE_EXISTS_(params_.map_file_path_);
  ASSERT_FILE_EXISTS_(params_.bitmap_file_path_);

  if (params_.load_map_)
  {
    ROS_INFO("loading and displaying map.");
  }
  else
  {
    ROS_INFO("listening to objects which are stored in the provided map file.");
  }

  if (params_.update_map_)
  {
    sub_map_ = n_.subscribe("map", 1, &ObjectListenerNode::callbackMap, this);
    sub_object_detections_ = n_.subscribe("map_doors", 1, &ObjectListenerNode::callbackObjectDetections, this);
  }
  metric_map_ = boost::make_shared<mrpt::maps::CMultiMetricMap>();
  if (!params_.load_map_)
  {
    mrpt::containers::deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr> grid_map(mrpt::maps::COccupancyGridMap2D::Create());
    mrpt::containers::deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr> bearing_map(mrpt::maps::CBearingMap::Create());
    metric_map_->m_gridMaps.push_back(grid_map);
    metric_map_->maps.push_back(bearing_map);
  }
  else
  {
    mrpt::config::CConfigFile ini_file;
    ini_file.setFileName(params_.ini_file_path_);
    if (!mrpt_bridge::MapHdl::loadMap(metric_map_, ini_file, params_.map_file_path_, "metricMap"))
    {
      ROS_ERROR("map could not be loaded.");
    }
//    CFileGZInputStream f(params_.map_file_path_);
//#if MRPT_VERSION >= 0x199
//    mrpt::serialization::archiveFrom(f) >> metric_map_;
//#else
//    f >> metric_map_;
//#endif
//    ASSERTMSG_(
//      metric_map_.size() > 0,
//      "Metric map was aparently loaded OK, but it is empty!");

  }
}

void ObjectListenerNode::callbackMap(const nav_msgs::OccupancyGrid &_msg)
{
  ASSERT_(metric_map_->m_gridMaps.size() == 1);
  mrpt_bridge::convert(_msg, *metric_map_->m_gridMaps[0]);
  mrpt::io::CFileOutputStream fileOut(params_.map_file_path_);
  mrpt::serialization::archiveFrom(fileOut) << *metric_map_;
  metric_map_->m_gridMaps[0]->saveAsBitmapFile(params_.bitmap_file_path_);
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
      if (!bearings && params_.update_map_)
      {
        ROS_ERROR("attempting to update multimetric map without initializing a bearingmap first!");
      }
      std::vector<mrpt::maps::CBearing::Ptr>::iterator it_b = std::find_if(bearings->begin(),bearings->end(),
                                                       [&o_id] (const mrpt::maps::CBearing::Ptr b)
                                                                  {
                                                                      return b->m_ID == o_id;
                                                                  });
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
  if (!metric_map_->m_bearingMap)
  {
    std::cout << "no bearingmap" << std::endl;
  }
  if (!metric_map_->m_gridMaps[0])
  {
    std::cout << "no gridmaps" << std::endl;
  }
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
