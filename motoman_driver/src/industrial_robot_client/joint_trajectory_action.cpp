/*
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Fraunhofer IPA
 * Author: Thiago de Freitas
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *  * Neither the name of the Fraunhofer IPA, nor the names
 *  of its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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

#include <motoman_driver/industrial_robot_client/joint_trajectory_action.h>
#include "motoman_driver/industrial_robot_client/motoman_utils.h"
#include <industrial_robot_client/utils.h>
#include <industrial_utils/param_utils.h>
#include <industrial_utils/utils.h>
#include <map>
#include <string>
#include <vector>

using industrial_robot_client::motoman_utils::getJointGroups;

namespace industrial_robot_client
{
namespace joint_trajectory_action
{

const double JointTrajectoryAction::WATCHD0G_PERIOD_ = 1.0;
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

JointTrajectoryAction::JointTrajectoryAction() :
  action_server_(node_, "joint_trajectory_action",
                 boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                 boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false)
{
  ros::NodeHandle pn("~");

  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  std::map<int, RobotGroup> robot_groups;
  getJointGroups("topic_list", robot_groups);

  for (size_t i = 0; i < robot_groups.size(); i++)
  {
    std::string joint_path_action_name = robot_groups[i].get_ns() + "/" + robot_groups[i].get_name();
    std::vector<std::string> rg_joint_names = robot_groups[i].get_joint_names();
    std::vector<double> rg_goal_tolerances = robot_groups[i].get_goal_tolerances();
    int group_number_int = robot_groups[i].get_group_id();

    all_joint_names_.insert(all_joint_names_.end(), rg_joint_names.begin(), rg_joint_names.end());

    std::map<std::string, double> rg_goal_tolerances_map;
    // handle empty goal tolerances (not set in topic_list parameter), use global goal tolerance
    if (rg_goal_tolerances.size() == 0)
      rg_goal_tolerances = std::vector<double>(rg_joint_names.size(), goal_threshold_);

    industrial_robot_client::utils::toMap(rg_joint_names, rg_goal_tolerances, rg_goal_tolerances_map);
    motoman_utils::mapMerge(all_goal_tolerances_, rg_goal_tolerances_map);

    // init maps
    has_active_goal_map_[group_number_int] = false;
    trajectory_state_recvd_map_[group_number_int] = false;

    auto actionServer_ = new actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>(
      node_, joint_path_action_name + "/joint_trajectory_action" , false);
    actionServer_->registerGoalCallback(
      boost::bind(&JointTrajectoryAction::goalCB,
                  this, _1, group_number_int));
    actionServer_->registerCancelCallback(
      boost::bind(&JointTrajectoryAction::cancelCB,
                  this, _1, group_number_int));

    pub_trajectories_[group_number_int] = node_.advertise<motoman_msgs::DynamicJointTrajectory>(
                                joint_path_action_name + "/joint_path_command", 1);
    sub_trajectories_[group_number_int] = node_.subscribe<control_msgs::FollowJointTrajectoryFeedback>(
                               joint_path_action_name + "/feedback_states", 1,
                               boost::bind(&JointTrajectoryAction::controllerStateCB,
                                           this, _1, group_number_int));
    sub_status_[group_number_int] = node_.subscribe(
                          "robot_status", 1, &JointTrajectoryAction::robotStatusCB, this);

    this->act_servers_[group_number_int] = actionServer_;

    this->act_servers_[group_number_int]->start();

    this->watchdog_timer_map_[group_number_int] = node_.createTimer(
          ros::Duration(WATCHD0G_PERIOD_), boost::bind(
            &JointTrajectoryAction::watchdog, this, _1, group_number_int));
  }

  has_active_goal_ = false;
  trajectory_state_recvd_ = false;

  pub_trajectory_command_ = node_.advertise<motoman_msgs::DynamicJointTrajectory>(
                              "joint_path_command", 1);
  sub_trajectory_state_ = this->node_.subscribe<control_msgs::FollowJointTrajectoryFeedback>(
                              "feedback_states", 1, &JointTrajectoryAction::controllerStateCB, this);
  watchdog_timer_ = node_.createTimer(ros::Duration(WATCHD0G_PERIOD_), &JointTrajectoryAction::watchdog, this);

  this->robot_groups_ = robot_groups;

  // global action server (created in constructor initializer list) start
  action_server_.start();
}

JointTrajectoryAction::~JointTrajectoryAction()
{
}

void JointTrajectoryAction::robotStatusCB(
  const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg;  // caching robot status for later use.
}

void JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
{
  auto log_name = "jta_global";
  auto log_prepend = "Watchdog global: ";

  // Some debug logging
  if (!last_trajectory_state_)
  {
    ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Waiting for subscription to joint trajectory state");
  }
  if (!trajectory_state_recvd_)
  {
    ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Trajectory state not received since last watchdog");
  }

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_)
  {
    if (!trajectory_state_recvd_)
    {
      // last_trajectory_state_ is null if the subscriber never makes a connection
      if (!last_trajectory_state_)
      {
        ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Aborting goal because we have never heard a controller state message.");
      }
      else
      {
        ROS_WARN_STREAM_NAMED(log_name, log_prepend <<
          "Aborting goal because we haven't heard from the controller in " << WATCHD0G_PERIOD_ << " seconds");
      }
      abortGoal();
    }
  }

  // Reset the global trajectory state received flag
  trajectory_state_recvd_ = false;
}

void JointTrajectoryAction::watchdog(const ros::TimerEvent &e, int group_number)
{
  auto log_name = "jta_gX";
  auto log_prepend = "Watchdog #" + std::to_string(group_number) + ": ";

  // Some debug logging
  if (!last_trajectory_state_map_[group_number])
  {
    ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Waiting for subscription to joint trajectory state");
  }
  if (!trajectory_state_recvd_map_[group_number])
  {
    ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Trajectory state not received since last watchdog");
  }

  // Aborts the active goal if the controller does not appear to be active.
  if (has_active_goal_map_[group_number])
  {
    if (!trajectory_state_recvd_map_[group_number])
    {
      // last_trajectory_state_ is null if the subscriber never makes a connection
      if (!last_trajectory_state_map_[group_number])
      {
        ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Aborting goal because we have never heard a controller state message.");
      }
      else
      {
        ROS_WARN_STREAM_NAMED(log_name, log_prepend <<
          "Aborting goal because we haven't heard from the controller in " << WATCHD0G_PERIOD_ << " seconds");
      }
      abortGoal(group_number);
    }
  }

  // Reset the per-group trajectory state received flag
  trajectory_state_recvd_map_[group_number] = false;
}

bool JointTrajectoryAction::allGroupsFinished() const {
  typedef std::map<int, bool>::const_iterator iterator;

  for (iterator i = has_active_goal_map_.begin(); i != has_active_goal_map_.end(); ++i) {
    if (i->second) return false;
  }
  return true;
}

void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh)
{
  auto log_name = "jta_global";
  auto log_prepend = "GoalCB global: ";

  if (!gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(
          all_joint_names_,
          gh.getGoal()->trajectory.joint_names))
    {
      // Cancels the currently active goal.
      if (has_active_goal_)
      {
        ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Received new goal, canceling current goal");
        abortGoal();
      }

      // also abort any per-group goals, because we are starting a new global goal
      for (int group_index = 0; group_index < robot_groups_.size(); group_index++) {
        if (has_active_goal_map_[group_index])
        {
          ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Received new goal, canceling current goal");
          abortGoal(group_index);
        }
      }

      // Sends the trajectory along to the controller
      gh.setAccepted();
      active_goal_ = gh;
      has_active_goal_ = true;

      ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Publishing trajectory");

      current_traj_ = active_goal_.getGoal()->trajectory;

      for (int group_index = 0; group_index < robot_groups_.size(); group_index++) {
        // Tell the rest of the system this robot id has an active goal.
        has_active_goal_map_[group_index] = true;
        // Set the goal as the active goal for this robot id.
        active_goal_map_[group_index] = gh;
        current_traj_map_[group_index] = active_goal_map_[group_index].getGoal()->trajectory;
      }
      
      motoman_msgs::DynamicJointTrajectory dyn_traj;
      
      for (int i = 0; i < gh.getGoal()->trajectory.points.size(); i++) {
        motoman_msgs::DynamicJointPoint dpoint;
        
        for (int rbt_idx = 0; rbt_idx < robot_groups_.size(); rbt_idx++) {
          int num_joints = robot_groups_[rbt_idx].get_joint_names().size();
          motoman_msgs::DynamicJointsGroup dyn_group;
          
          // Check if positions vector is empty.
          if (gh.getGoal()->trajectory.points[i].positions.empty()) {
            std::vector<double> positions(num_joints, 0.0);
            dyn_group.positions = positions;
          }
          
          // Check if velocities vector is empty.
          if (gh.getGoal()->trajectory.points[i].velocities.empty()) {
            std::vector<double> velocities(num_joints, 0.0);
            dyn_group.velocities = velocities;
          }
          
          // Check if acceleration vector is empty.
          if (gh.getGoal()->trajectory.points[i].accelerations.empty()) {
            std::vector<double> accelerations(num_joints, 0.0);
            dyn_group.accelerations = accelerations;
          }
          
          // Check if effort vector is empty.
          if (gh.getGoal()->trajectory.points[i].effort.empty()) {
            std::vector<double> effort(num_joints, 0.0);
            dyn_group.effort = effort;
          }
          
          // for each joint in robot group
          for (int j = 0; j < num_joints; j++) {
            // find this joint index trajectory joints
            size_t ros_idx = std::find(gh.getGoal()->trajectory.joint_names.begin(), gh.getGoal()->trajectory.joint_names.end(), robot_groups_[rbt_idx].get_joint_names()[j]) - gh.getGoal()->trajectory.joint_names.begin();
            
            // Insert position value for joint j.
            if (!gh.getGoal()->trajectory.points[i].positions.empty()) {
              dyn_group.positions.insert(
                dyn_group.positions.begin() + j,
                *(gh.getGoal()->trajectory.points[i].positions.begin() + ros_idx));
            }
            
            // Insert velocity value for joint j.
            if (!gh.getGoal()->trajectory.points[i].velocities.empty()) {
              dyn_group.velocities.insert(
                dyn_group.velocities.begin() + j,
                *(gh.getGoal()->trajectory.points[i].velocities.begin() + ros_idx));
            }
            
            // Insert acceleration value for joint j.
            if (!gh.getGoal()->trajectory.points[i].accelerations.empty()) {
              dyn_group.accelerations.insert(
                dyn_group.accelerations.begin() + j,
                *(gh.getGoal()->trajectory.points[i].accelerations.begin() + ros_idx));
            }
            
            // Insert effort value for joint j.
            if (!gh.getGoal()->trajectory.points[i].effort.empty()) {
              dyn_group.effort.insert(
                dyn_group.effort.begin() + j,
                *(gh.getGoal()->trajectory.points[i].effort.begin() + ros_idx));
            }
          }
          dyn_group.time_from_start = gh.getGoal()->trajectory.points[i].time_from_start;
          dyn_group.group_number = rbt_idx;
          dyn_group.num_joints = dyn_group.positions.size();
          
          dpoint.groups.push_back(dyn_group);
        }
        dpoint.num_groups = dpoint.groups.size();
        dyn_traj.points.push_back(dpoint);
      }
      
      dyn_traj.header = gh.getGoal()->trajectory.header;
      dyn_traj.header.stamp = ros::Time::now();
      
      // Publishing with joints for all of the groups
      dyn_traj.joint_names = all_joint_names_;
      
      this->pub_trajectory_command_.publish(dyn_traj);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(log_name, log_prepend << "Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(log_name, log_prepend << "Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM_NAMED(log_name, log_prepend <<
      "Ignoring goal tolerance in action, using topic_list tolerances or paramater tolerance instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle gh)
{
  auto log_name = "jta_global";
  auto log_prepend = "CancelCB global: ";

  ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Received action cancel request");
  if (active_goal_ == gh)
  {
    // Stops the controller.
    motoman_msgs::DynamicJointTrajectory empty;
    empty.joint_names = all_joint_names_;
    this->pub_trajectory_command_.publish(empty);

    // Marks the current goal as canceled.
    active_goal_.setCanceled();
    has_active_goal_ = false;

    for (int group_index = 0; group_index < robot_groups_.size(); group_index++) {
      has_active_goal_map_[group_index] = false;
    }
  }
  else
  {
    ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Active goal and goal cancel do not match, ignoring cancel request");
  }
}

void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh, int group_number)
{
  auto log_name = "jta_gX";
  auto log_prepend = "GoalCB #" + std::to_string(group_number) + ": ";

  if (!gh.getGoal()->trajectory.points.empty())
  {
    if (industrial_utils::isSimilar(
          this->robot_groups_[group_number].get_joint_names(),
          gh.getGoal()->trajectory.joint_names))
    {
      // Cancels the currently active goal.
      if (has_active_goal_map_[group_number])
      {
        ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Received new goal, canceling current goal");
        abortGoal(group_number);
      }
      // Sends the trajectory along to the controller
      gh.setAccepted();
      active_goal_map_[group_number] = gh;
      has_active_goal_map_[group_number] = true;

      ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Publishing trajectory");

      current_traj_map_[group_number] = active_goal_map_[group_number].getGoal()->trajectory;

      int num_joints = robot_groups_[group_number].get_joint_names().size();

      motoman_msgs::DynamicJointTrajectory dyn_traj;

      for (size_t i = 0; i < current_traj_map_[group_number].points.size(); ++i)
      {
        motoman_msgs::DynamicJointsGroup dyn_group;
        motoman_msgs::DynamicJointPoint dyn_point;

        if (gh.getGoal()->trajectory.points[i].positions.empty())
        {
          std::vector<double> positions(num_joints, 0.0);
          dyn_group.positions = positions;
        }
        else
          dyn_group.positions = gh.getGoal()->trajectory.points[i].positions;

        if (gh.getGoal()->trajectory.points[i].velocities.empty())
        {
          std::vector<double> velocities(num_joints, 0.0);
          dyn_group.velocities = velocities;
        }
        else
          dyn_group.velocities = gh.getGoal()->trajectory.points[i].velocities;
        if (gh.getGoal()->trajectory.points[i].accelerations.empty())
        {
          std::vector<double> accelerations(num_joints, 0.0);
          dyn_group.accelerations = accelerations;
        }
        else
          dyn_group.accelerations = gh.getGoal()->trajectory.points[i].accelerations;
        if (gh.getGoal()->trajectory.points[i].effort.empty())
        {
          std::vector<double> effort(num_joints, 0.0);
          dyn_group.effort = effort;
        }
        else
          dyn_group.effort = gh.getGoal()->trajectory.points[i].effort;
        dyn_group.time_from_start = gh.getGoal()->trajectory.points[i].time_from_start;
        dyn_group.group_number = group_number;
        dyn_group.num_joints = robot_groups_[group_number].get_joint_names().size();
        dyn_point.groups.push_back(dyn_group);

        dyn_point.num_groups = 1;
        dyn_traj.points.push_back(dyn_point);
      }
      dyn_traj.header = gh.getGoal()->trajectory.header;
      dyn_traj.joint_names = gh.getGoal()->trajectory.joint_names;
      this->pub_trajectories_[group_number].publish(dyn_traj);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(log_name, log_prepend << "Joint trajectory action failing on invalid joints");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(rslt, "Joint names do not match");
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(log_name, log_prepend << "Joint trajectory action failed on empty trajectory");
    control_msgs::FollowJointTrajectoryResult rslt;
    rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    gh.setRejected(rslt, "Empty trajectory");
  }

  // Adding some informational log messages to indicate unsupported goal constraints
  if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
  {
    ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Ignoring goal time tolerance in action goal, may be supported in the future");
  }
  if (!gh.getGoal()->goal_tolerance.empty())
  {
    ROS_WARN_STREAM_NAMED(log_name, log_prepend <<
      "Ignoring goal tolerance in action, using topic_list tolerances or paramater tolerance instead");
  }
  if (!gh.getGoal()->path_tolerance.empty())
  {
    ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
  }
}

void JointTrajectoryAction::cancelCB(
  JointTractoryActionServer::GoalHandle gh, int group_number)
{
  auto log_name = "jta_gX";
  auto log_prepend = "CancelCB #" + std::to_string(group_number) + ": ";

  ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Received action cancel request");
  if (active_goal_map_[group_number] == gh)
  {
    // Stops the controller.
    motoman_msgs::DynamicJointTrajectory empty;
    empty.joint_names = robot_groups_[group_number].get_joint_names();
    this->pub_trajectories_[group_number].publish(empty);

    // Marks the current goal as canceled.
    active_goal_map_[group_number].setCanceled();
    has_active_goal_map_[group_number] = false;
  }
  else
  {
    ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Active goal and goal cancel do not match, ignoring cancel request");
  }
}

void JointTrajectoryAction::controllerStateCB(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg, int robot_id)
{
  auto log_name = "jta_gX";
  auto log_prepend = "ControllerStateCB #" + std::to_string(robot_id) + ": ";

  ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Checking controller state feedback");
  last_trajectory_state_map_[robot_id] = msg;
  trajectory_state_recvd_map_[robot_id] = true;

  if (!has_active_goal_map_[robot_id])
  {
    ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "No active goal, ignoring feedback");
    return;
  }

  if (current_traj_map_[robot_id].points.empty())
  {
    ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(robot_groups_[robot_id].get_joint_names(), msg->joint_names))
  {
    ROS_ERROR_STREAM_NAMED(log_name, log_prepend << "Joint names from the controller don't match our joint names.");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_map_[robot_id], current_traj_map_[robot_id], robot_id))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Inside goal constraints, stopped moving, return success for action");
        has_active_goal_map_[robot_id] = false;
        // if this group is executing a goal separate from the global goal, then set it to succeeded
        //  otherwise the global controllerCB handler will do this
        if (active_goal_map_[robot_id] != active_goal_) active_goal_map_[robot_id].setSucceeded();
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Inside goal constraints, return success for action");
        ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Robot status in motion unknown, the robot driver node and controller code should be updated");
        has_active_goal_map_[robot_id] = false;
        // if this group is executing a goal separate from the global goal, then set it to succeeded
        //  otherwise the global controllerCB handler will do this
        if (active_goal_map_[robot_id] != active_goal_) active_goal_map_[robot_id].setSucceeded();
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Inside goal constraints, return success for action");
      ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Robot status is not being published the robot driver node and controller code should be updated");
      has_active_goal_map_[robot_id] = false;
      // if this group is executing a goal separate from the global goal, then set it to succeeded
      //  otherwise the global controllerCB handler will do this
      if (active_goal_map_[robot_id] != active_goal_) active_goal_map_[robot_id].setSucceeded();
    }
  }
}

void JointTrajectoryAction::controllerStateCB(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
{
  auto log_name = "jta_global";
  auto log_prepend = "ControllerStateCB global: ";

  ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Checking controller state feedback");
  last_trajectory_state_ = msg;
  trajectory_state_recvd_ = true;

  if (!has_active_goal_)
  {
    ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "No active goal, ignoring feedback");
    return;
  }
  if (current_traj_.points.empty())
  {
    ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Current trajectory is empty, ignoring feedback");
    return;
  }

  if (!industrial_utils::isSimilar(all_joint_names_, msg->joint_names))
  {
    ROS_ERROR_STREAM_NAMED(log_name, log_prepend << "Joint names from the controller don't match our joint names.");
    return;
  }

  // Checking for goal constraints
  // Checks that we have ended inside the goal constraints and has motion stopped

  ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Checking goal constraints");
  if (withinGoalConstraints(last_trajectory_state_, current_traj_))
  {
    if (last_robot_status_)
    {
      // Additional check for motion stoppage since the controller goal may still
      // be moving.  The current robot driver calls a motion stop if it receives
      // a new trajectory while it is still moving.  If the driver is not publishing
      // the motion state (i.e. old driver), this will still work, but it warns you.
      if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
      {
        ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Inside goal constraints, stopped moving, return success for action");
        active_goal_.setSucceeded();
        has_active_goal_ = false;

        for (int group_index = 0; group_index < robot_groups_.size(); group_index++) {
          has_active_goal_map_[group_index] = false;
        }
      }
      else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
      {
        ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Inside goal constraints, return success for action");
        ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Robot status in motion unknown, the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;

        for (int group_index = 0; group_index < robot_groups_.size(); group_index++) {
          has_active_goal_map_[group_index] = false;
        }
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED(log_name, log_prepend << "Within goal constraints but robot is still moving");
      }
    }
    else
    {
      ROS_INFO_STREAM_NAMED(log_name, log_prepend << "Inside goal constraints, return success for action");
      ROS_WARN_STREAM_NAMED(log_name, log_prepend << "Robot status is not being published the robot driver node and controller code should be updated");
      active_goal_.setSucceeded();
      has_active_goal_ = false;

      for (int group_index = 0; group_index < robot_groups_.size(); group_index++) {
        has_active_goal_map_[group_index] = false;
      }
    }
  }
}

void JointTrajectoryAction::abortGoal()
{
  // Stops the controller.
  trajectory_msgs::JointTrajectory empty;
  pub_trajectory_command_.publish(empty);

  if (has_active_goal_)
  {
    // Marks the current goal as aborted.
    active_goal_.setAborted();
    has_active_goal_ = false;

    for (int group_index = 0; group_index < robot_groups_.size(); group_index++) {
      // abort also goals for individual groups, because publising empty global trajectory stopped everything
      // only if they are different from the global goal (which is being set to individual group goals when starting global trajectory execution)
      if (active_goal_map_[group_index] != active_goal_ && has_active_goal_map_[group_index])
        active_goal_map_[group_index].setAborted();

      // do this always (because it's being set to true for individual groups when starting global trajectory execution, along with global goal to individual group goals)
      has_active_goal_map_[group_index] = false;
    }
  }
}

void JointTrajectoryAction::abortGoal(int robot_id)
{
  // Stops the controller.
  motoman_msgs::DynamicJointTrajectory empty;
  pub_trajectories_[robot_id].publish(empty);

  if (has_active_goal_map_[robot_id])
  {
    // Marks the current goal as aborted.
    active_goal_map_[robot_id].setAborted();
    has_active_goal_map_[robot_id] = false;
  }
}

bool JointTrajectoryAction::withinGoalConstraints(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
  const trajectory_msgs::JointTrajectory & traj)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;

    if (motoman_utils::isWithinRange(
          last_trajectory_state_->joint_names,
          last_trajectory_state_->actual.positions,
          traj.joint_names,
          traj.points[last_point].positions,
          all_goal_tolerances_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

bool JointTrajectoryAction::withinGoalConstraints(
  const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
  const trajectory_msgs::JointTrajectory & traj, int robot_id)
{
  bool rtn = false;
  if (traj.points.empty())
  {
    ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
    rtn = false;
  }
  else
  {
    int last_point = traj.points.size() - 1;
    int group_number = robot_id;

    std::vector<double> sorted_positions(robot_groups_[group_number].get_joint_names().size());
    std::vector<std::string> sorted_joint_names(robot_groups_[group_number].get_joint_names().size());

    // TODO: I believe it's not neccessary because isWithinRange uses map inside?
    for (int sort_index = 0; sort_index < robot_groups_[group_number].get_joint_names().size(); sort_index++) {
        size_t ros_idx = std::find(traj.joint_names.begin(), traj.joint_names.end(), robot_groups_[group_number].get_joint_names()[sort_index]) - traj.joint_names.begin();
        sorted_positions[sort_index] = traj.points[last_point].positions[ros_idx];
        sorted_joint_names[sort_index] = traj.joint_names[ros_idx];
    }

    if (motoman_utils::isWithinRange(
          robot_groups_[group_number].get_joint_names(),
          last_trajectory_state_map_[group_number]->actual.positions,
          sorted_joint_names,
          sorted_positions,
          all_goal_tolerances_))
    {
      rtn = true;
    }
    else
    {
      rtn = false;
    }
  }
  return rtn;
}

}  // namespace joint_trajectory_action
}  // namespace industrial_robot_client
