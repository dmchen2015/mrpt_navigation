#ifndef OBJECT_LISTENER_NODE_H
#define OBJECT_LISTENER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tuw_object_msgs/ObjectDetection.h>
#include <boost/filesystem.hpp>
#include <mrpt/maps/CMultiMetricMap.h>
#include <fstream>
#include <string>
#include <memory>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/gui/CDisplayWindow3D.h>

/**
 * This class listens to objectdetections and inserts them into mrpt maps.
 * Furthermore, it also listens to nav_msgs::OccupancyGridMap messages and converts those maps to mrpt maps
 * @brief The ObjectListener class
 */

class ObjectListenerNode
{
  public:
    ObjectListenerNode(ros::NodeHandle &n);
    ~ObjectListenerNode();

    void init();
    void callbackMap(const nav_msgs::OccupancyGrid &_msg);
    void callbackObjectDetections(const tuw_object_msgs::ObjectDetection &_msg);
    void display();

  private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_{"~"};
    ros::Subscriber sub_map_;
    ros::Subscriber sub_object_detections_;
    std::ofstream map_file_;
    std::string map_file_path_;
    std::string bitmap_file_path_;
    boost::shared_ptr<mrpt::maps::CMultiMetricMap> metric_map_;
    bool load_map_;

    //opengl stuff
    mrpt::gui::CDisplayWindow3D::Ptr window_;
};

#endif // OBJECT_LISTENER_NODE_H
