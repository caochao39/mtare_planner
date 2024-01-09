//
// Created by caochao on 12/9/19.
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

bool transform_to_global_frame_update = false;
bool kNogoZoneBehindStartGate = false;
bool added_nogo_zone_behind_start_gate = false;

double kNogoZoneBehindStartGateSizeX = 20;
double kNogoZoneBehindStartGateSizeY = 20;

tf::Transform transform_to_global_frame_;

geometry_msgs::Point32 ToGlobalFrame(const geometry_msgs::Point32& local_point)
{
  tf::Vector3 vec;
  vec.setX(local_point.x);
  vec.setY(local_point.y);
  vec.setZ(local_point.z);
  vec = transform_to_global_frame_ * vec;
  geometry_msgs::Point32 global_point;
  global_point.x = vec.x();
  global_point.y = vec.y();
  global_point.z = vec.z();
  return global_point;
}

geometry_msgs::Point32 ToLocalFrame(const geometry_msgs::Point32& global_point)
{
  tf::Vector3 vec;
  vec.setX(global_point.x);
  vec.setY(global_point.y);
  vec.setZ(global_point.z);
  vec = transform_to_global_frame_.inverse() * vec;
  geometry_msgs::Point32 local_point;
  local_point.x = vec.x();
  local_point.y = vec.y();
  local_point.z = vec.z();
  return local_point;
}

void AddNogoZoneBehindStartGate(geometry_msgs::Polygon& nogo_boundary_polygon)
{
  double max_z = 0.0;
  for (int i = 0; i < nogo_boundary_polygon.points.size(); i++)
  {
    if (nogo_boundary_polygon.points[i].z > max_z)
    {
      max_z = nogo_boundary_polygon.points[i].z;
    }
  }
  double new_z = max_z + 1.0;

  geometry_msgs::Point32 global_point;
  geometry_msgs::Point32 local_point;
  global_point.x = 0.0;
  global_point.y = kNogoZoneBehindStartGateSizeY / 2;
  global_point.z = 0.0;
  local_point = ToLocalFrame(global_point);
  local_point.z = new_z;
  nogo_boundary_polygon.points.push_back(local_point);

  global_point.x = -kNogoZoneBehindStartGateSizeX;
  global_point.y = kNogoZoneBehindStartGateSizeY / 2;
  global_point.z = 0.0;
  local_point = ToLocalFrame(global_point);
  local_point.z = new_z;
  nogo_boundary_polygon.points.push_back(local_point);

  global_point.x = -kNogoZoneBehindStartGateSizeX;
  global_point.y = -kNogoZoneBehindStartGateSizeY / 2;
  global_point.z = 0.0;
  local_point = ToLocalFrame(global_point);
  local_point.z = new_z;
  nogo_boundary_polygon.points.push_back(local_point);

  global_point.x = 0.0;
  global_point.y = -kNogoZoneBehindStartGateSizeY / 2;
  global_point.z = 0.0;
  local_point = ToLocalFrame(global_point);
  local_point.z = new_z;
  nogo_boundary_polygon.points.push_back(local_point);
}

void TransformToGlobalFrameCallback(const nav_msgs::Odometry::ConstPtr& transform_msg)
{
  transform_to_global_frame_.setOrigin(tf::Vector3(
      transform_msg->pose.pose.position.x, transform_msg->pose.pose.position.y, transform_msg->pose.pose.position.z));
  transform_to_global_frame_.setRotation(
      tf::Quaternion(transform_msg->pose.pose.orientation.x, transform_msg->pose.pose.orientation.y,
                     transform_msg->pose.pose.orientation.z, transform_msg->pose.pose.orientation.w));
  transform_to_global_frame_update = true;
}

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
  ros::init(argc, argv, "coverage_boundary_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  std::string pub_coverage_boundary_topic;
  std::string pub_viewpoint_boundary_topic;
  std::string pub_nogo_boundary_topic;
  std::string sub_transform_to_darpa_world_topic;
  std::string kCoverageAreaBoundaryFilename;
  std::string kViewPointBoundaryFilename;
  std::string kNogoBoundaryFilename;

  pub_coverage_boundary_topic =
      misc_utils_ns::getParam<std::string>(&nhPrivate, "pub_coverage_boundary_topic", "coverage_boundary");
  pub_viewpoint_boundary_topic =
      misc_utils_ns::getParam<std::string>(&nhPrivate, "pub_viewpoint_boundary_topic", "viewpoint_boundary");
  pub_nogo_boundary_topic =
      misc_utils_ns::getParam<std::string>(&nhPrivate, "pub_nogo_boundary_topic", "nogo_boundary");
  sub_transform_to_darpa_world_topic = misc_utils_ns::getParam<std::string>(
      &nhPrivate, "sub_transform_to_darpa_world_topic", "/transform_to_darpa_world");
  kNogoZoneBehindStartGate = misc_utils_ns::getParam<bool>(&nhPrivate, "kNogoZoneBehindStartGate", false);

  kCoverageAreaBoundaryFilename = misc_utils_ns::getParam<std::string>(&nhPrivate, "kCoverageAreaBoundaryFilename", "");
  kViewPointBoundaryFilename = misc_utils_ns::getParam<std::string>(&nhPrivate, "kViewPointBoundaryFilename", "");
  kNogoBoundaryFilename = misc_utils_ns::getParam<std::string>(&nhPrivate, "kNogoBoundaryFilename", "");
  kNogoZoneBehindStartGate = misc_utils_ns::getParam<bool>(&nhPrivate, "kNogoZoneBehindStartGate", false);
  kNogoZoneBehindStartGateSizeX = misc_utils_ns::getParam<double>(&nhPrivate, "kNogoZoneBehindStartGateSizeX", 20);
  kNogoZoneBehindStartGateSizeY = misc_utils_ns::getParam<double>(&nhPrivate, "kNogoZoneBehindStartGateSizeY", 20);

  std::cout << "Coverage area boundary filename: " << kCoverageAreaBoundaryFilename << std::endl;
  std::cout << "Viewpoint boundary filename: " << kViewPointBoundaryFilename << std::endl;
  std::cout << "Nogo boundary filename: " << kNogoBoundaryFilename << std::endl;

  ros::Publisher coverage_boundary_pub = nh.advertise<geometry_msgs::PolygonStamped>(pub_coverage_boundary_topic, 2);
  ros::Publisher viewpoint_boundary_pub = nh.advertise<geometry_msgs::PolygonStamped>(pub_viewpoint_boundary_topic, 2);
  ros::Publisher nogo_boundary_pub = nh.advertise<geometry_msgs::PolygonStamped>(pub_nogo_boundary_topic, 2);

  ros::Subscriber transform_to_global_frame_sub =
      nh.subscribe<nav_msgs::Odometry>(sub_transform_to_darpa_world_topic, 2, TransformToGlobalFrameCallback);

  std::string coverage_boundary_filename = ros::package::getPath("tare_planner");
  coverage_boundary_filename += "/config/boundary/";
  coverage_boundary_filename += kCoverageAreaBoundaryFilename;
  std::cout << "Reading Coverage boundary from file: " << coverage_boundary_filename << std::endl;

  std::string viewpoint_boundary_filename = ros::package::getPath("tare_planner");
  viewpoint_boundary_filename += "/config/boundary/";
  viewpoint_boundary_filename += kViewPointBoundaryFilename;
  std::cout << "Reading Viewpoint bounday from file: " << viewpoint_boundary_filename << std::endl;

  std::string nogo_boundary_filename = ros::package::getPath("tare_planner");
  nogo_boundary_filename += "/config/boundary/";
  nogo_boundary_filename += kNogoBoundaryFilename;
  std::cout << "Reading Nogo bounday from file: " << nogo_boundary_filename << std::endl;

  // Read polygon from file
  geometry_msgs::PolygonStamped coverage_boundary_polygon;
  coverage_boundary_polygon.header.frame_id = "/map";
  coverage_boundary_polygon.header.stamp = ros::Time::now();
  ReadPolygonFromFile(coverage_boundary_polygon.polygon, coverage_boundary_filename);
  std::cout << "Finished reading polygon of " << coverage_boundary_polygon.polygon.points.size() << " points"
            << std::endl;

  for (int i = 0; i < coverage_boundary_polygon.polygon.points.size(); i++)
  {
    coverage_boundary_polygon.polygon.points[i].z = 0.0;
  }

  geometry_msgs::PolygonStamped viewpoint_boundary_polygon;
  viewpoint_boundary_polygon.header.frame_id = "/map";
  viewpoint_boundary_polygon.header.stamp = ros::Time::now();
  ReadPolygonFromFile(viewpoint_boundary_polygon.polygon, viewpoint_boundary_filename);
  std::cout << "Finished reading polygon of " << viewpoint_boundary_polygon.polygon.points.size() << " points"
            << std::endl;

  for (int i = 0; i < coverage_boundary_polygon.polygon.points.size(); i++)
  {
    viewpoint_boundary_polygon.polygon.points[i].z = 0.0;
  }

  geometry_msgs::PolygonStamped nogo_boundary_polygon;
  nogo_boundary_polygon.header.frame_id = "/map";
  nogo_boundary_polygon.header.stamp = ros::Time::now();
  ReadPolygonFromFile(nogo_boundary_polygon.polygon, nogo_boundary_filename);
  std::cout << "Finished reading polygon of " << nogo_boundary_polygon.polygon.points.size() << " points" << std::endl;

  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    // Publish polygon
    if (transform_to_global_frame_update && kNogoZoneBehindStartGate && !added_nogo_zone_behind_start_gate)
    {
      AddNogoZoneBehindStartGate(nogo_boundary_polygon.polygon);
      added_nogo_zone_behind_start_gate = true;
      std::cout << "Adding nogo zone behind the start gate" << std::endl;
    }
    coverage_boundary_pub.publish(coverage_boundary_polygon);
    viewpoint_boundary_pub.publish(viewpoint_boundary_polygon);
    nogo_boundary_pub.publish(nogo_boundary_polygon);
    rate.sleep();
  }

  return 0;
}