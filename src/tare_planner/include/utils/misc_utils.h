/**
 * @file utils.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Miscellaneous utility functions
 * @version 0.1
 * @date 2019-06-05
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <limits>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <iostream>
#include <cstdlib>
// uncomment to disable assert()
// #define NDEBUG
#include <cassert>
#include <algorithm>
#include <vector>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iterator>

#define MY_ASSERT(val)                                                                                                 \
  if (!(val))                                                                                                          \
  {                                                                                                                    \
    ROS_ERROR_STREAM("Error at [File: " << __FILE__ << "][Line: " << __LINE__ << "]"                                   \
                                        << "[Function: " << __FUNCTION__ << "]");                                      \
    exit(1);                                                                                                           \
  }

namespace misc_utils_ns
{
typedef pcl::PointXYZI PCLPointType;
typedef pcl::PointCloud<PCLPointType> PCLPointCloudType;

const int INF_DISTANCE = 99999999;

template <class FromPointType, class ToPointType>
void PointToPoint(const FromPointType& from_point, ToPointType& to_point)
{
  to_point.x = from_point.x;
  to_point.y = from_point.y;
  to_point.z = from_point.z;
}
geometry_msgs::Point PCL2GeoMsgPnt(const PCLPointType& pnt);
PCLPointType GeoMsgPnt2PCL(const geometry_msgs::Point& pnt);
geometry_msgs::Point GeoMsgPoint(double x, double y, double z);
PCLPointType PCLPoint(float x, float y, float z);
geometry_msgs::Point Eigen2GeoMsgPnt(const Eigen::Vector3d& pnt);
Eigen::Vector3d GeoMsgPnt2Eigen(const geometry_msgs::Point& pnt);
// TODO: make these template functions
void LeftRotatePoint(PCLPointType& pnt);
void RightRotatePoint(PCLPointType& pnt);
void LeftRotatePoint(geometry_msgs::Point& pnt);
void RightRotatePoint(geometry_msgs::Point& pnt);
template <class CloudType>
void KeyposeToMap(CloudType& cloud, const nav_msgs::Odometry::ConstPtr& keypose);
double PointXYDist(const geometry_msgs::Point& pnt1, const geometry_msgs::Point& pnt2);
double PointXYDist(const PCLPointType& pnt1, const PCLPointType& pnt2);
template <class P1, class P2>
double PointXYDist(const P1& pnt1, const P2& pnt2)
{
  return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2));
}
template <class P1, class P2>
double PointXYZDist(const P1& pnt1, const P2& pnt2)
{
  return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2) + pow((pnt1.z - pnt2.z), 2));
}
double VectorXYAngle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
double PointAngle(const geometry_msgs::Point& pnt, const geometry_msgs::Point& robot_pos);
double PointAngle(const PCLPointType& pnt, const geometry_msgs::Point& robot_pos);
bool CollinearXY(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3,
                 double threshold = 0.1);
bool LineSegIntersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& q1, const geometry_msgs::Point& p2,
                      const geometry_msgs::Point& q2);
bool LineSegIntersectWithTolerance(const geometry_msgs::Point& p1, const geometry_msgs::Point& q1,
                                   const geometry_msgs::Point& p2, const geometry_msgs::Point& q2,
                                   const double tolerance);
int ThreePointOrientation(const geometry_msgs::Point& p, const geometry_msgs::Point& q, const geometry_msgs::Point& r);
bool PointOnLineSeg(const geometry_msgs::Point& p, const geometry_msgs::Point& q, const geometry_msgs::Point& r);
double AngleOverlap(double s1, double e1, double s2, double e2);
double AngleDiff(double source_angle, double target_angle);
bool PointInPolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon);
double LineSegDistance2D(const geometry_msgs::Point& point, const geometry_msgs::Point& line_segment_start,
                         const geometry_msgs::Point& line_segment_end);
double LineSegDistance3D(const geometry_msgs::Point& point, const geometry_msgs::Point& line_segment_start,
                         const geometry_msgs::Point& line_segment_end);
double DistancePoint2DToPolygon(const geometry_msgs::Point& point, const geometry_msgs::Polygon& polygon);
void LinInterpPoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double resolution,
                     std::vector<Eigen::Vector3d>& interp_points);
template <class PCLPointType>
void LinInterpPoints(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, double resolution,
                     typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
{
  double point_dist = PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(p1, p2);
  if (point_dist < 0.01)
    return;
  PCLPointType point1;
  point1.x = p1.x;
  point1.y = p1.y;
  point1.z = p1.z;
  PCLPointType point2;
  point2.x = p1.x;
  point2.y = p1.y;
  point2.z = p1.z;
  if (point_dist < resolution)
  {
    cloud->points.push_back(point1);
    cloud->points.push_back(point2);
  }
  else
  {
    int num_points = static_cast<int>(point_dist / resolution);
    cloud->points.push_back(point1);
    for (int i = 0; i < num_points; i++)
    {
      PCLPointType point;
      point.x = static_cast<float>((p2.x - p1.x) / num_points * i + p1.x);
      point.y = static_cast<float>((p2.y - p1.y) / num_points * i + p1.y);
      point.z = static_cast<float>((p2.z - p1.z) / num_points * i + p1.z);
      cloud->points.push_back(point);
    }
    cloud->points.push_back(point2);
  }
}
double DegreeToRadian(double degree);
double RadianToDegree(double radian);
/**
 * Function to append part of path2 ([from_ind, to_ind]) to path1
 * @param path1
 * @param path2
 * @param from_ind
 * @param to_ind
 */
void ConcatenatePath(nav_msgs::Path& path1, const nav_msgs::Path& path2, int from_ind = -1, int to_ind = -1);
/**
 * Function to check if an index is within the range of a list
 * @tparam T
 * @param list
 * @param index
 * @return
 */
template <class T>
bool InRange(const std::vector<T>& list, int index)
{
  return index >= 0 && index < list.size();
}
template <typename T>
T getParam(ros::NodeHandle* nh, const std::string& name, const T default_val)
{
  T val;
  bool success = nh->getParam(name, val);
  if (!success)
  {
    ROS_ERROR_STREAM("Cannot read parameter: " << name);
    return default_val;
  }
  return val;
}
template <typename T>
T getParam(ros::NodeHandle& nh, const std::string& name, const T default_val)
{
  T val;
  bool success = nh.getParam(name, val);
  if (!success)
  {
    ROS_ERROR_STREAM("Cannot read parameter: " << name);
    return default_val;
  }
  return val;
}
/**
 * Function to publish clouds
 * @tparam T PCL PointCloud type
 * @param cloud_publisher
 * @param cloud
 * @param frame_id
 */
template <class PCLPointCloudType>
void PublishCloud(const ros::Publisher& cloud_publisher, const PCLPointCloudType& cloud, const std::string& frame_id)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.frame_id = frame_id;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_publisher.publish(cloud_msg);
}

template <class ROSMsgType>
void Publish(const ros::Publisher& publisher, ROSMsgType& msg, const std::string& frame_id)
{
  msg.header.frame_id = frame_id;
  msg.header.stamp = ros::Time::now();
  publisher.publish(msg);
}

void SetDifference(std::vector<int>& v1, std::vector<int>& v2, std::vector<int>& diff);

class Timer
{
private:
  std::string name_;
  std::chrono::time_point<std::chrono::high_resolution_clock> timer_start_;
  std::chrono::time_point<std::chrono::high_resolution_clock> timer_stop_;
  std::chrono::duration<double> duration_;

public:
  Timer(std::string name) : name_(name)
  {
  }
  ~Timer() = default;
  void Start()
  {
    timer_start_ = std::chrono::high_resolution_clock::now();
  }
  int Split(std::string unit = "ms")
  {
    duration_ = std::chrono::high_resolution_clock::now() - timer_start_;
    return GetDuration(unit);
  }
  void Stop(bool show, std::string unit = "ms")
  {
    timer_stop_ = std::chrono::high_resolution_clock::now();
    duration_ = timer_stop_ - timer_start_;
    if (show)
    {
      if (unit == "ms")
      {
        // std::cout << name_ << " takes " << GetDuration<std::chrono::milliseconds>() << " ms" << std::endl;
        ROS_INFO_STREAM(name_ << " takes " << GetDuration<std::chrono::milliseconds>() << " ms");
      }
      else if (unit == "us")
      {
        // std::cout << name_ << " takes " << GetDuration<std::chrono::microseconds>() << " us" << std::endl;
        ROS_INFO_STREAM(name_ << " takes " << GetDuration<std::chrono::microseconds>() << " us");
      }
      else if (unit == "ns")
      {
        // std::cout << name_ << " takes " << GetDuration<std::chrono::nanoseconds>() << " ns" << std::endl;
        ROS_INFO_STREAM(name_ << " takes " << GetDuration<std::chrono::nanoseconds>() << " ns");
      }
      else if (unit == "s")
      {
        // std:cout << name_ << " takes " << GetDuration<std::chrono::seconds>() << " s" << std::endl;
        ROS_INFO_STREAM(name_ << " takes " << GetDuration<std::chrono::seconds>() << " s");
      }
      else
      {
        // std::cout << "timer unit error!" << std::endl;
        ROS_ERROR("timer unit error!");
      }
    }
  }
  template <class U>
  int GetDuration()
  {
    return std::chrono::duration_cast<U>(duration_).count();
  }
  int GetDuration(std::string unit = "ms")
  {
    if (unit == "ms")
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>(duration_).count();
    }
    else if (unit == "us")
    {
      return std::chrono::duration_cast<std::chrono::microseconds>(duration_).count();
    }
    else if (unit == "ns")
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count();
    }
    else if (unit == "s")
    {
      return std::chrono::duration_cast<std::chrono::seconds>(duration_).count();
    }
    else
    {
      std::cout << "timer unit error!" << std::endl;
      return 0;
    }
  }
};

class Marker
{
private:
  std::string pub_topic_;
  std::string frame_id_;
  ros::Publisher marker_pub_;

public:
  static int id_;
  visualization_msgs::Marker marker_;
  explicit Marker(ros::NodeHandle* nh, std::string pub_topic, std::string frame_id)
    : pub_topic_(pub_topic), frame_id_(frame_id)
  {
    marker_pub_ = nh->advertise<visualization_msgs::Marker>(pub_topic_, 2);
    id_++;
    marker_.id = id_;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
  }
  explicit Marker(ros::NodeHandle& nh, std::string pub_topic, std::string frame_id)
    : pub_topic_(pub_topic), frame_id_(frame_id)
  {
    marker_pub_ = nh.advertise<visualization_msgs::Marker>(pub_topic_, 2);
    id_++;
    marker_.id = id_;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.orientation.w = 1.0;
  }
  ~Marker() = default;
  void SetColorRGBA(const std_msgs::ColorRGBA& color)
  {
    marker_.color = color;
  }
  void SetColorRGBA(double r, double g, double b, double a)
  {
    marker_.color.r = r;
    marker_.color.g = g;
    marker_.color.b = b;
    marker_.color.a = a;
  }
  void SetScale(double x, double y, double z)
  {
    marker_.scale.x = x;
    marker_.scale.y = y;
    marker_.scale.z = z;
  }
  void SetType(int type)
  {
    marker_.type = type;
  }
  void SetAction(visualization_msgs::Marker::_action_type action)
  {
    marker_.action = action;
  }
  void Publish()
  {
    misc_utils_ns::Publish<visualization_msgs::Marker>(marker_pub_, marker_, frame_id_);
  }

  typedef std::shared_ptr<Marker> Ptr;
};
int signum(int x);
double mod(double value, double modulus);
double intbound(double s, double ds);
bool InRange(const Eigen::Vector3i& sub, const Eigen::Vector3i& max_sub = Eigen::Vector3i(INT_MAX, INT_MAX, INT_MAX),
             const Eigen::Vector3i& min_sub = Eigen::Vector3i(INT_MIN, INT_MIN, INT_MIN));
void RayCast(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub, const Eigen::Vector3i& max_sub,
             const Eigen::Vector3i& min_sub, std::vector<Eigen::Vector3i>& output);
bool InFOV(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position, double vertical_half_fov,
           double range);
bool InFOVSimple(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position,
                 double vertical_fov_ratio, double range, double xy_dist_threshold = 0, double z_diff_threshold = 0,
                 bool print = false);
float ApproxAtan(float z);
float ApproxAtan2(float y, float x);
double GetPathLength(const nav_msgs::Path& path);
double GetPathLength(const std::vector<Eigen::Vector3d>& path);
double AStarSearch(const std::vector<std::vector<int>>& graph, const std::vector<std::vector<double>>& node_dist,
                   const std::vector<geometry_msgs::Point>& node_positions, int from_idx, int to_idx, bool get_path,
                   std::vector<int>& path_indices);
bool AStarSearchWithMaxPathLength(const std::vector<std::vector<int>>& graph,
                                  const std::vector<std::vector<double>>& node_dist,
                                  const std::vector<geometry_msgs::Point>& node_positions, int from_idx, int to_idx,
                                  bool get_path, std::vector<int>& path_indices, double& shortest_dist,
                                  double max_path_length = DBL_MAX);
nav_msgs::Path SimplifyPath(const nav_msgs::Path& path);
nav_msgs::Path DeduplicatePath(const nav_msgs::Path& path, double min_dist);
void SampleLineSegments(const std::vector<Eigen::Vector3d>& initial_points, double sample_resol,
                        std::vector<Eigen::Vector3d>& sample_points);
void UniquifyIntVector(std::vector<int>& list);

template <class T>
void RemoveElementFromVector(std::vector<T>& vec, const T& element)
{
  vec.erase(std::remove(vec.begin(), vec.end(), element), vec.end());
}

template <class T>
bool ElementExistsInVector(const std::vector<T>& vec, const T& element)
{
  return std::find(vec.begin(), vec.end(), element) != vec.end();
}

template <class T>
T GetMaxElement(const std::vector<T>& vec, int& max_element_index)
{
  MY_ASSERT(!vec.empty())
  T max_element = vec.front();
  for (int i = 0; i < vec.size(); i++)
  {
    if (vec[i] > max_element)
    {
      max_element = vec[i];
      max_element_index = i;
    }
  }
  return max_element;
}

template <class T>
T GetMinElement(const std::vector<T>& vec, int& min_element_index)
{
  MY_ASSERT(!vec.empty())
  T min_element = vec.front();
  for (int i = 0; i < vec.size(); i++)
  {
    if (vec[i] < min_element)
    {
      min_element = vec[i];
      min_element_index = i;
    }
  }
  return min_element;
}

template <class T>
void PrintVector(const std::vector<T>& vec)
{
  std::stringstream v_stream;
  for (const auto& v : vec)
  {
    v_stream << v << " ";
  }
  // std::cout << std::endl;
  ROS_INFO_STREAM(v_stream.str());
}
template <class T>
void Print2DVector(const std::vector<std::vector<T>>& vec)
{
  for (const auto& row : vec)
  {
    std::stringstream v_stream;
    for (const auto& e : row)
    {
      v_stream << e << " ";
      // std::cout << e << " ";
    }
    ROS_INFO_STREAM(v_stream.str());
    // std::cout << std::endl;
  }
}

// Use functions instead of operator overloading to avoid Argument-dependent-lookup (ADL) issues where the overloaded
// operator cannot be found within a namespace
template <class T>
void SaveVectorToFile(const std::vector<T>& vec, const std::string& filename)
{
  std::ofstream file(filename, std::ios::trunc);
  for (const auto& e : vec)
  {
    file << e << '\n';
  }
  file.close();
}

template <class T>
void Save2DVectorToFile(const std::vector<std::vector<T>>& vec, const std::string& filename)
{
  std::ofstream file(filename, std::ios::trunc);
  for (auto& row : vec)
  {
    for (auto e : row)
      file << e << '\t';
    file << '\n';
  }
  file.close();
}

template <class T>
void ReadVectorFromFile(std::vector<T>& vec, const std::string& filename)
{
  std::ifstream file(filename, std::ios::in);
  vec.clear();                                  // Make sure M is empty
  for (std::string line; getline(file, line);)  // Read one line at a time
  {
    std::stringstream ss(line);  // Put that line in a temporary stream
    T e;
    ss >> e;
    vec.push_back(e);  // Add the row to the matrix
  }
  file.close();
}

template <class T>
void Read2DVectorFromFile(std::vector<std::vector<T>>& vec, const std::string& filename)
{
  std::ifstream file(filename, std::ios::in);
  vec.clear();                                  // Make sure M is empty
  for (std::string line; getline(file, line);)  // Read one line at a time
  {
    std::stringstream ss(line);  // Put that line in a temporary stream
    std::vector<T> row;
    for (T e; ss >> e; row.push_back(e))
      ;                  // Stream the elements on that line into a new row
    vec.push_back(row);  // Add the row to the matrix
  }
  file.close();
}

void GetCommonElements(const std::vector<int>& vec1, const std::vector<int>& vec2, std::vector<int>& vec_common);
std_msgs::ColorRGBA Color(int r, int g, int b);

enum class TextColor
{
  WHITE = 0,
  RED = 1,
  GREEN = 2,
  BLUE = 3,
  YELLOW = 4,
  MAGENTA = 5,
  CYAN = 6
};
std::string ColoredText(const std::string& text, TextColor color);

}  // namespace misc_utils_ns
