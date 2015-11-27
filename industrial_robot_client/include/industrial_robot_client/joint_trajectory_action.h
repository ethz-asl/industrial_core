/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JOINT_TRAJTORY_ACTION_H
#define JOINT_TRAJTORY_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <industrial_msgs/RobotStatus.h>

namespace industrial_robot_client
{
namespace joint_trajectory_action
{
class JointTrajectoryAction
{
 public:
  JointTrajectoryAction();
  ~JointTrajectoryAction();
  void run() { ros::spin(); }
 private:
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointTractoryActionServer;
  ros::NodeHandle node_;
  JointTractoryActionServer action_server_;
  ros::Publisher pub_trajectory_command_;
  ros::Subscriber sub_trajectory_state_;
  ros::Subscriber sub_robot_status_;
  ros::Timer watchdog_timer_;
  bool controller_alive_;
  bool has_active_goal_;
  JointTractoryActionServer::GoalHandle active_goal_;
  trajectory_msgs::JointTrajectory current_traj_;
  static const double DEFAULT_GOAL_THRESHOLD_;// = 0.01;
  double goal_threshold_;
  std::vector<std::string> joint_names_;
  control_msgs::FollowJointTrajectoryFeedbackConstPtr last_trajectory_state_;
  industrial_msgs::RobotStatusConstPtr last_robot_status_;
  ros::Time time_to_check_;
  static const double WATCHDOG_PERIOD_;// = 1.0;
  void watchdog(const ros::TimerEvent &e);
  void goalCB(JointTractoryActionServer::GoalHandle & gh);
  void cancelCB(JointTractoryActionServer::GoalHandle & gh);
  void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);
  void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg);
  void abortGoal();
};
} // joint_trajectory_action
} // industrial_robot_client
#endif /* JOINT_TRAJTORY_ACTION_H */
