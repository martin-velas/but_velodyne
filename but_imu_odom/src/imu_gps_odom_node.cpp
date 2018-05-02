/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Veľas (ivelas@fit.vutbr.cz)
 * Date: 13/03/2017
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <limits>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/Visualizer3D.h>

// install package: qtpositioning5-dev
#include <qt5/QtPositioning/QGeoCoordinate>

using namespace std;
using namespace pcl;
using namespace ros;
using namespace tf;
using namespace but_velodyne;

const string NODE_NAME = "imu_gps_odom_node";

class Parser {

private:
  QGeoCoordinate first_coord;
  bool first_coord_set;
  ros::Publisher pose_pub;

public:
  Visualizer3D visualizer;

  Parser(NodeHandle &nh) :
    first_coord_set(false),
    pose_pub(nh.advertise<geometry_msgs::PoseStamped>("imu_gps_pose", 10)){
  }

  void callback(const sensor_msgs::NavSatFixConstPtr& gps_msg,
      const geometry_msgs::Vector3StampedConstPtr& gps_error,
      const geometry_msgs::PoseStampedConstPtr& orient_msg) {
    int precision = std::numeric_limits<double>::digits10;

    QGeoCoordinate coord(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
    if(!first_coord_set) {
      first_coord = coord;
      first_coord_set = true;
    }

    geometry_msgs::PoseStamped pose = *orient_msg;
    double azimuth = first_coord.azimuthTo(coord);
    double distance = first_coord.distanceTo(coord);
    pose.pose.position.x = cos(DEG2RAD(azimuth)) * distance;
    pose.pose.position.y = sin(DEG2RAD(azimuth)) * distance;
    pose.pose.position.z = -(coord.altitude() - first_coord.altitude());

    Eigen::Affine3d affine_tf;
    tf2::fromMsg(pose.pose, affine_tf);
    cout << gps_msg->header.stamp << " " << gps_error->vector.x << " " << gps_error->vector.y << " " << gps_error->vector.z << " ";
    Eigen::Affine3f affine_tf_float = affine_tf.cast<float>();
    KittiUtils::printPose(cout, affine_tf_float.matrix());

    visualizer.getViewer()->addCoordinateSystem(0.5, affine_tf_float);
    visualizer.showOnce(10);

    pose_pub.publish(pose);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, NODE_NAME, ros::init_options::AnonymousName);
  NodeHandle nh("~");

  message_filters::Subscriber<sensor_msgs::NavSatFix> sub1(nh, "/imu_nav", 10);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub2(nh, "/imu_nav_errors", 10);
  message_filters::Subscriber<geometry_msgs::PoseStamped> sub3(nh, "/imu_pose", 10);

  Parser parser(nh);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::NavSatFix,
      geometry_msgs::Vector3Stamped,
      geometry_msgs::PoseStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
  sync.registerCallback(boost::bind(&Parser::callback, &parser, _1, _2, _3));

  spin();

  parser.visualizer.show();

  return EXIT_SUCCESS;
}
