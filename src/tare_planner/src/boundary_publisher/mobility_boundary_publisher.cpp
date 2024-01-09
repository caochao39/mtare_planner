//
// Created by caochao on 01/21/22.
//
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <utils/misc_utils.h>

void ReadPolygonFromFile(geometry_msgs::Polygon& polygon, std::string filename)
{
  polygon.points.clear();
  std::ifstream file(filename);
  int val;
  std::string tmp_str, cur_str, last_str;
  std::string line;
  while (std::getline(file, line))
  {
    if (line == "end_header")
    {
      break;
    }
  }
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    geometry_msgs::Point32 point;
    if (!(iss >> point.x >> point.y >> point.z))
    {
      break;
    }
    polygon.points.push_back(point);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mobility_boundary_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  std::string pub_mobility_boundary_topic;

  std::string kMobilityBoundaryFilename;

  pub_mobility_boundary_topic =
      misc_utils_ns::getParam<std::string>(&nhPrivate, "pub_mobility_boundary_topic", "mobility_boundary");

  kMobilityBoundaryFilename = misc_utils_ns::getParam<std::string>(&nhPrivate, "kMobilityBoundaryFilename", "");

  std::cout << "Mobility boundary filename: " << kMobilityBoundaryFilename << std::endl;

  ros::Publisher mobility_boundary_pub = nh.advertise<geometry_msgs::PolygonStamped>(pub_mobility_boundary_topic, 2);

  std::string mobility_boundary_filename = ros::package::getPath("tare_planner");
  mobility_boundary_filename += "/config/boundary/";
  mobility_boundary_filename += kMobilityBoundaryFilename;
  std::cout << "Reading Mobility boundary from file: " << mobility_boundary_filename << std::endl;

  // Read polygon from file
  geometry_msgs::PolygonStamped mobility_boundary_polygon;
  mobility_boundary_polygon.header.frame_id = "/map";
  mobility_boundary_polygon.header.stamp = ros::Time::now();
  ReadPolygonFromFile(mobility_boundary_polygon.polygon, mobility_boundary_filename);
  std::cout << "Finished reading polygon of " << mobility_boundary_polygon.polygon.points.size() << " points"
            << std::endl;

  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    mobility_boundary_pub.publish(mobility_boundary_polygon);
    rate.sleep();
  }

  return 0;
}