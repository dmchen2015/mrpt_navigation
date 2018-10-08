#ifndef MAP_CREATOR_NODE_H
#define MAP_CREATOR_NODE_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/filesystem.hpp>
#include <mrpt/maps/CMultiMetricMap.h>
#include <fstream>
#include <string>
#include <memory>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <tf/transform_listener.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/containers/deepcopy_poly_ptr.h>
#include <mrpt_msgs/ObservationObject.h>
//#include <mrpt_msgs/ObservationRangeBearing.h>

/**
 * This class listens to objectdetections and inserts them into mrpt maps.
 * Furthermore, it also listens to nav_msgs::OccupancyGridMap messages and converts those maps to mrpt maps
 * @brief The ObjectListener class
 */

class MapCreatorNode
{
  public:
    struct ParametersNode
    {
        ParametersNode();
        ros::NodeHandle nh;
        bool load_map;
        bool update_map;
        bool debug;
        bool insert_as_simplemap;
        std::string map_file_path_;
        std::string bitmap_file_path_;
        std::string ini_file_path_;
        std::string tf_prefix;
        std::string base_frame_id;
    };


   MapCreatorNode(ros::NodeHandle &n);
    ~MapCreatorNode();

    ParametersNode *param();
    void init();
    void callbackMap(const nav_msgs::OccupancyGrid &_msg);
    //void callbackBearings(const mrpt_msgs::ObservationRangeBearing &_msg);
		void callbackObjectObservations(const mrpt_msgs::ObservationObject &_msg);
    bool getStaticTF(std::string source_frame, mrpt::poses::CPose3D &des);
    void display();
    void saveMap();

  private:

    ros::NodeHandle n_;
    ros::NodeHandle n_param_{"~"};
    ros::Subscriber sub_map_;
    ros::Subscriber sub_object_detections_;
    mrpt::maps::CMultiMetricMap metric_map_;
    ParametersNode* params_;
    tf::TransformListener listenerTF_;
    std::map<std::string, mrpt::poses::CPose3D> static_tf_;
    mrpt::poses::CPose3D map_pose_;
    //opengl stuff
    mrpt::gui::CDisplayWindow3D::Ptr window_;
};

#endif // OBJECT_LISTENER_NODE_H
