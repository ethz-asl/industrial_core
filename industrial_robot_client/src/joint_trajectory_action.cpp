/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Southwest Research Institute
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

#include <industrial_robot_client/joint_trajectory_action.h>
#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>

namespace industrial_robot_client
{
namespace joint_trajectory_action
{
const double JointTrajectoryAction::WATCHDOG_PERIOD_ = 1.0;
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

JointTrajectoryAction::JointTrajectoryAction() :
    action_server_(node_, "joint_trajectory_action", 
                   boost::bind(&JointTrajectoryAction::goalCB, this, _1), 
                   boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false), 
    has_active_goal_(false), controller_alive_(false) {
      ros::NodeHandle pn("~");
      pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);
      if (!industrial_utils::param::getJointNames("controller_joint_names", 
                                                  "robot_description", joint_names_))
        ROS_ERROR("Failed to initialize joint_names.");
      // The controller joint names parameter includes empty joint names for those joints 
      // not supported by the controller.  These are removed since the trajectory action 
      // should ignore these.
      std::remove(joint_names_.begin(), joint_names_.end(), std::string());
      ROS_INFO_STREAM("Filtered joint names to " << joint_names_.size() << " joints");
      pub_trajectory_command_ = 
          node_.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
      sub_trajectory_state_ = 
          node_.subscribe("feedback_states", 1, &JointTrajectoryAction::controllerStateCB, this);
      sub_robot_status_ = 
          node_.subscribe("robot_status", 1, &JointTrajectoryAction::robotStatusCB, this);
      watchdog_timer_ = node_.createTimer(ros::Duration(WATCHDOG_PERIOD_), 
                                          &JointTrajectoryAction::watchdog, this, true);
      action_server_.start();
    }
JointTrajectoryAction::~JointTrajectoryAction() {}
void JointTrajectoryAction::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg; //caching robot status for later use.
}
void JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
{
  // Some debug logging
  if (!last_trajectory_state_)
  {
    ROS_DEBUG("Waiting for subscription to joint trajectory state");
  }
  ROS_WARN("Trajectory state not received for %f seconds", WATCHDOG_PERIOD_);
  controller_alive_ = false;
  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    // last_trajectory_state_ is null if the subscriber never makes a connection
    if (!last_trajectory_state_)
    {
      ROS_WARN("Aborting goal because we have never heard a controller state message.");
    }
    else
    {
      ROS_WARN_STREAM("Aborting goal because we haven't heard from the controller in " 
                      << WATCHDOG_PERIOD_ << " seconds");
    }
    abortGoal();
  }
}
void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle & gh)
{
  ROS_INFO("Received new goal");
  // reject all goals as long as we haven't heard from the remote controller
  if (!controller_alive_)
  {
    ROS_ERROR("Joint trajectory action rejected: waiting for (initial) feedback from controller");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Waiting for (initial) feedback from controller");
    // no point in continuing: already rejected
    return;
  }
  if (!gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(joint_names_, gh.getGoal()->trajectory.joint_names))
    {
      // Cancels the currently active goal.
      if (has_active_goal_)
      {
        ROS_WARN("Received new goal, canceling current goal");
        abortGoal();
      }
      gh.setAccepted();
      active_goal_ = gh;
      has_active_goal_ = true;
      ROS_INFO("Publishing trajectory");
      current_traj_ = active_goal_.getGoal()->trajectory;
      pub_trajectory_command_.publish(current_traj_);
    }
    else
    {
      ROS_ERROR("Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR("Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }
  // Adding some informational log messages to indicate unsupported goal constraints
  if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM("Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal tolerance in action, using paramater tolerance of " 
                    << goal_threshold_ << " instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM("Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}
void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle & gh)
{
  ROS_DEBUG("Received action cancel request");
  if (active_goal_ == gh)
  {
    // Stops the controller.
    trajectory_msgs::JointTrajectory empty;
    empty.joint_names = joint_names_;
    pub_trajectory_command_.publish(empty);
    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }
  else
  {
    ROS_WARN("Active goal and goal cancel do not match, ignoring cancel request");
  }
}
void JointTrajectoryAction::controllerStateCB(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  ROS_DEBUG("Checking controller state feedback");
  last_trajectory_state_ = msg;
  controller_alive_ = true;

  watchdog_timer_.stop();
  watchdog_timer_.start();

  if (!has_active_goal_)
  {
    ROS_DEBUG("No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    ROS_DEBUG("Current trajectory is empty, ignoring feedback");
    return;
  }
  if (!industrial_utils::isSimilar(joint_names_, msg->joint_names))
  {
    ROS_ERROR("Joint names from the controller don't match our joint names.");
    return;
  }
  ROS_INFO("Return success for action");
  active_goal_.setSucceeded();
  has_active_goal_ = false;
}
void JointTrajectoryAction::abortGoal()
{
  // Stops the controller.
  trajectory_msgs::JointTrajectory empty;
  pub_trajectory_command_.publish(empty);
  // Marks the current goal as aborted.
  active_goal_.setAborted();
  has_active_goal_ = false;
}
} // joint_trajectory_action
} // industrial_robot_client
