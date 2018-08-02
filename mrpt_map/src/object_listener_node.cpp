#include <object_listener_node.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/pose.h>
#include <mrpt/maps/CBearing.h>
#include <mrpt/maps/CBearingMap.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CFileInputStream.h>

#include <string>

#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/math/types_math.h>

#include <mrpt/config/CConfigFile.h>

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/stock_objects.h>

ObjectListenerNode::ParametersNode::ParametersNode() : nh("~")
{
    nh.param<std::string>(std::string("simplemap_file"), map_file_path_, std::string(""));
    nh.param<std::string>(std::string("bitmap_file"), bitmap_file_path_, std::string(""));
    nh.param<std::string>(std::string("ini_file"), ini_file_path_, std::string(""));
    nh.param<std::string>(std::string("base_frame_id"), base_frame_id, "base_link");
    nh.param<std::string>(std::string("tf_prefix"), tf_prefix, "");
    nh.param<bool>(std::string("load_map"), load_map, false);
    nh.param<bool>(std::string("update_map"), update_map, true);
    nh.param<bool>(std::string("insert_as_simplemap"), insert_as_simplemap, false);
    ROS_INFO("tf_prefix: %s", tf_prefix.c_str());
    ROS_INFO("map_file: %s", map_file_path_.c_str());
    ROS_INFO("bitmap_file: %s", bitmap_file_path_.c_str());
    ROS_INFO("ini file: %s", ini_file_path_.c_str());
    ROS_INFO("base_frame_id: %s", base_frame_id.c_str());
    ROS_INFO("update map: %s", update_map ? "True" : "False");
    ROS_INFO("load map: %s", load_map ? "True" : "False");
}

ObjectListenerNode::ObjectListenerNode(ros::NodeHandle& n) : n_(n)
{
  params_ = new ObjectListenerNode::ParametersNode();
}

ObjectListenerNode::~ObjectListenerNode()
{
  delete params_;
}

ObjectListenerNode::ParametersNode* ObjectListenerNode::param()
{
  return (ObjectListenerNode::ParametersNode*) params_;
}

void ObjectListenerNode::init()
{
  using namespace mrpt::io;

  if (params_->load_map)
  {
    ASSERT_FILE_EXISTS_(params_->ini_file_path_);
    ASSERT_FILE_EXISTS_(params_->map_file_path_);
  }

  if (params_->load_map)
  {
    ROS_INFO("loading and displaying map.");
  }
  else
  {
    ROS_INFO("listening to objects which are stored in the provided map file.");
  }

  if (params_->update_map)
  {
    sub_map_ = n_.subscribe("map", 1, &ObjectListenerNode::callbackMap, this);
    sub_object_detections_ = n_.subscribe("map_doors", 1, &ObjectListenerNode::callbackObjectDetections, this);
  }
  if (!params_->load_map)
  {
    using namespace mrpt::containers;
    deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr> grid_map(mrpt::maps::COccupancyGridMap2D::Create());
    deepcopy_poly_ptr<mrpt::maps::CMetricMap::Ptr> bearing_map(mrpt::maps::CBearingMap::Create());
    metric_map_.m_gridMaps.push_back(grid_map);
    metric_map_.maps.push_back(bearing_map);
  }
  else
  {
    mrpt::config::CConfigFile ini_file;
    ini_file.setFileName(params_->ini_file_path_);
    MRPT_TODO("use a simplemap here");

    if (params_->map_file_path_.find("simplemap") == std::string::npos)
    {

      CFileInputStream f(params_->map_file_path_);
#if MRPT_VERSION >= 0x199
      mrpt::serialization::archiveFrom(f) >> metric_map_;
#else
      f >> metric_map_;
#endif

      ASSERTMSG_(
        std::distance(metric_map_.begin(), metric_map_.end()) > 0,
        "Metric map was aparently loaded OK, but it is empty!");

    }
    else
    {

      if (!mrpt_bridge::MapHdl::loadMap(metric_map_, ini_file, params_->map_file_path_, "metricMap"))
      {
        ROS_ERROR("map could not be loaded.");
      }

    }
  }
}

void ObjectListenerNode::callbackMap(const nav_msgs::OccupancyGrid &_msg)
{
  ASSERT_(metric_map_.m_gridMaps.size() == 1);
  mrpt_bridge::convert(_msg, *metric_map_.m_gridMaps[0]);
  metric_map_.m_gridMaps[0]->saveAsBitmapFile(params_->bitmap_file_path_);
  if (params_->insert_as_simplemap)
  {
    MRPT_TODO("Implement simplemap");
    ROS_ERROR("Not implemented");
  }
  if (!getStaticTF(_msg.header.frame_id, map_pose_))
  {
    ROS_ERROR("Map tf not present, cannot continue");
  }
}

void ObjectListenerNode::saveMap()
{
  mrpt::io::CFileOutputStream fileOut(params_->map_file_path_);
  mrpt::serialization::archiveFrom(fileOut) << metric_map_;
}

void ObjectListenerNode::callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg)
{
  using namespace mrpt::obs;
  using namespace mrpt::maps;
  using namespace mrpt::containers;
  using namespace mrpt::poses;

  CObservationBearingRange obs;
  obs.setSensorPose(map_pose_);
  obs.fieldOfView_pitch = M_PI/180.0 * 270.0;

  for (std::vector<tuw_object_msgs::ObjectWithCovariance>::const_iterator it = _msg.objects.begin();
       it != _msg.objects.end(); ++it)
  {
    if (it->object.shape == tuw_object_msgs::Object::SHAPE_DOOR)
    {
      const auto o_id = it->object.ids[0];
      CBearing::Ptr bear;
      if (!metric_map_.m_bearingMap && params_->update_map)
      {
        metric_map_.maps.push_back(deepcopy_poly_ptr<CMetricMap::Ptr>(
                                     CBearingMap::Create()));
      }

      CPose3D pose;
      mrpt_bridge::convert(it->object.pose, pose);

      CObservationBearingRange::TMeasurement d;
      {
        d.landmarkID = o_id;
        d.pitch = 0;
        double dx = pose.x() - map_pose_.x();
        double dy = pose.y() - map_pose_.y();
        d.yaw = atan2(dy, dx);
        d.range = pose.distance3DTo(map_pose_.x(), map_pose_.y(), pose.z());
      }
      obs.sensedData.push_back(d);
    }
  }
  metric_map_.m_bearingMap->insertObservation(dynamic_cast<mrpt::obs::CObservation*>(&obs), &map_pose_);
}

void ObjectListenerNode::display()
{
  MRPT_START
  if (!metric_map_.m_bearingMap)
  {
    std::cout << "no bearingmap" << std::endl;
  }
  if (!metric_map_.m_gridMaps[0])
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
  metric_map_.getAs3DObject(objs);
  scene->insert(objs);

  COpenGLScene::Ptr &scenePtr = window_->get3DSceneAndLock();
  scenePtr = scene;
  window_->unlockAccess3DScene();
  window_->forceRepaint();
  ROS_INFO("repainting scene");

  MRPT_END
}

bool ObjectListenerNode::getStaticTF(std::string source_frame, mrpt::poses::CPose3D &des)
{
  std::string target_frame_id = tf::resolve(param()->tf_prefix, param()->base_frame_id);
  std::string source_frame_id = source_frame;
  std::string key = target_frame_id + "->" + source_frame_id;
  mrpt::poses::CPose3D pose;
  tf::StampedTransform transform;

  if (static_tf_.find(key) == static_tf_.end()) {

      try
      {
          if (param()->debug)
              ROS_INFO(
                  "debug: updateLaserPose(): target_frame_id='%s' source_frame_id='%s'",
                  target_frame_id.c_str(), source_frame_id.c_str());

          listenerTF_.lookupTransform(
              target_frame_id, source_frame_id, ros::Time(0), transform);
          tf::Vector3 translation = transform.getOrigin();
          tf::Quaternion quat = transform.getRotation();
          pose.x() = translation.x();
          pose.y() = translation.y();
          pose.z() = translation.z();
          tf::Matrix3x3 Rsrc(quat);
          mrpt::math::CMatrixDouble33 Rdes;
          for (int c = 0; c < 3; c++)
              for (int r = 0; r < 3; r++) Rdes(r, c) = Rsrc.getRow(r)[c];
          pose.setRotationMatrix(Rdes);
          static_tf_[key] = pose;
          ROS_INFO("Static tf '%s' with '%s'",
                   key.c_str(), pose.asString().c_str());
      }
      catch (tf::TransformException ex)
      {
          ROS_INFO("getStaticTF");
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          return false;
      }
  }
  des = static_tf_[key];
  return true;

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
    obj_listener_node.saveMap();
    rate.sleep();
  }
}
