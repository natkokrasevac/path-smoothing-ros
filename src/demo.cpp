/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kourclcpp.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kourclcpp
*********************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "path_smoothing_ros/cubic_spline_interpolator.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
};

class PathSmoothingDemo : public rclcpp::Node
{
  public:
    PathSmoothingDemo(): Node("path_smoothing_demo_node")
    {
      initialPosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/initial_pose", 1);
      finalPosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/final_pose", 1);
      pathPub = this->create_publisher<nav_msgs::msg::Path>("/initial_path", 1);
      smoothedPathPub = this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 1);

      pointsPerUnit = this->declare_parameter<int>("points_per_unit", 5);
      skipPoints = this->declare_parameter<int>("skip_points", 0);
      useEndConditions = this->declare_parameter<bool>("use_end_conditions", false);
      useMiddleConditions = this->declare_parameter<bool>("use_middle_conditions", true);
      smoothPath();
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initialPosePub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr finalPosePub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothedPathPub;

    int pointsPerUnit, skipPoints;
    bool useEndConditions, useMiddleConditions;

  void smoothPath()
  {

    std::vector<std::map<std::string, double>> poseList;
    poseList.push_back({{"x", 0.0}, {"y", 0.0}, {"yaw", 0}});
    poseList.push_back({{"x", 4.0}, {"y", 4.0}, {"yaw", 90}});

    nav_msgs::msg::Path path, smoothedPath;
    path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";

    for (int i = 0; i < poseList.size(); i++)
    {
      pose.pose.position.x = static_cast<double>(poseList[i]["x"]);
      pose.pose.position.y = static_cast<double>(poseList[i]["y"]);
      pose.pose.orientation = createQuaternionMsgFromYaw(poseList[i]["yaw"]);
      path.poses.push_back(pose);
    }

    // create a cubic spline interpolator
    path_smoothing::CubicSplineInterpolator csi(pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
    csi.interpolatePath(path, smoothedPath);
    initialPosePub->publish(path.poses.front());
    finalPosePub->publish(path.poses.back());
    pathPub->publish(path);
    smoothedPathPub->publish(smoothedPath);
    initialPosePub->publish(path.poses.at(0));
    finalPosePub->publish(path.poses.at(path.poses.size() - 1));
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathSmoothingDemo>());
  rclcpp::shutdown();
  return 0;
}


