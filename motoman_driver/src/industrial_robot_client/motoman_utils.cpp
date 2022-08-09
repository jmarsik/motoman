/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 * Author: Shaun Edwards
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
 *  * Neither the name of the Southwest Research Institute, nor the names
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

#include <industrial_robot_client/utils.h>
#include <industrial_utils/utils.h>
#include "motoman_driver/industrial_robot_client/motoman_utils.h"
#include "ros/ros.h"
#include <map>
#include <string>
#include <vector>

namespace industrial_robot_client
{
namespace motoman_utils
{

bool getJointGroups(const std::string topic_param, std::map<int, RobotGroup> & robot_groups)
{
  if (ros::param::has(topic_param))
  {
    XmlRpc::XmlRpcValue topics_list_rpc;
    ros::param::get(topic_param, topics_list_rpc);


    std::vector<XmlRpc::XmlRpcValue> topics_list;

    ROS_INFO_STREAM("Loading topic list");
    ROS_INFO_STREAM("Found " << topics_list_rpc.size() << " topics");

    for (int i = 0; i < topics_list_rpc.size(); i++)
    {
      XmlRpc::XmlRpcValue state_value;
      state_value = topics_list_rpc[i];
      ROS_INFO_STREAM("Topic(state_value): " << state_value);
      topics_list.push_back(state_value);
    }


    for (size_t i = 0; i < topics_list.size(); i++)
    {
      ROS_INFO_STREAM("Loading group: " << topics_list[i]);
      RobotGroup rg;
      std::vector<std::string> rg_joint_names;
      std::vector<double> rg_goal_tolerances;

      XmlRpc::XmlRpcValue joints;

      joints = topics_list[i]["joints"];
      for (int jt = 0; jt < joints.size(); jt++)
      {
        rg_joint_names.push_back(static_cast<std::string>(joints[jt]));
      }

      XmlRpc::XmlRpcValue goal_tolerances;

      if (topics_list[i].hasMember("goal_tolerances"))
      {
        goal_tolerances = topics_list[i]["goal_tolerances"];
  
        for (int jt = 0; jt < goal_tolerances.size(); jt++)
        {
          rg_goal_tolerances.push_back(static_cast<double>(goal_tolerances[jt]));
        }

        if (rg_joint_names.size() != rg_goal_tolerances.size())
        {
          ROS_ERROR_STREAM("Number of joints and their goal tolerances must be the same!");
          return false;
        }
      }

      XmlRpc::XmlRpcValue group_number;

      group_number = topics_list[i]["group"];
      int group_number_int = static_cast<int>(group_number);

      XmlRpc::XmlRpcValue name;
      std::string name_string;

      name = topics_list[i]["name"];
      name_string = static_cast<std::string>(name);

      XmlRpc::XmlRpcValue ns;
      std::string ns_string;

      ns = topics_list[i]["ns"];

      ns_string = static_cast<std::string>(ns);

      ROS_DEBUG_STREAM("Setting group: ");
      ROS_DEBUG_STREAM("  group number: " << group_number);
      ROS_DEBUG_STREAM("  group number(int): " << group_number_int);
      ROS_DEBUG_STREAM("  joints_names(size): " << rg_joint_names.size());
      ROS_DEBUG_STREAM("  name: " << name_string);
      ROS_DEBUG_STREAM("  ns: " << ns_string);
      rg.set_group_id(group_number_int);
      rg.set_joint_names(rg_joint_names);
      rg.set_goal_tolerances(rg_goal_tolerances);
      rg.set_name(name_string);
      rg.set_ns(ns_string);

      robot_groups[group_number] = rg;
    }

    ROS_INFO_STREAM("Loaded " << robot_groups.size() << " groups");
    return true;
  }
  else
  {
    ROS_INFO_STREAM("Failed to find '" << topic_param << "' parameter");
    return false;
  }
}

void mapMerge(std::map<std::string, double> & mergeTo, const std::map<std::string, double> & mergeFrom)
{
  // similar to C++ 17 merge method, which is not available in C++ 11
  for(const auto& it : mergeFrom)
  {
    mergeTo[it.first] = it.second;
  }
}

bool isWithinRange(const std::vector<std::string> & keys, const std::map<std::string, double> & lhs,
                   const std::map<std::string, double> & rhs, const std::map<std::string, double> & full_ranges)
{
  bool rtn = false;

  if ((keys.size() != rhs.size()) || (keys.size() != lhs.size()))
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Size mistmatch ::lhs size: " << lhs.size() <<
                     " rhs size: " << rhs.size() << " key size: " << keys.size());

    rtn = false;
  }
  else
  {
    rtn = true; // Assume within range, catch exception in loop below

    // This loop will not run for empty vectors, results in return of true
    for (size_t i = 0; i < keys.size(); ++i)
    {
      // find the joint goal tolerance
      auto ix = full_ranges.find(keys[i]);
      if (ix == full_ranges.end())
      {
        ROS_ERROR_STREAM(__FUNCTION__ << "::Goal tolerance for joint " << keys[i] << " not found");
        
        return false;
      }
      // Calculating the half range causes some precision loss, but it's good enough
      double half_range = full_ranges.at(keys[i]) / 2.0;

      if (fabs(lhs.at(keys[i]) - rhs.at(keys[i])) > fabs(half_range))
      {
        rtn = false;
        break;
      }
    }

  }

  return rtn;
}

bool isWithinRange(const std::vector<std::string> & lhs_keys, const std::vector<double> & lhs_values,
                   const std::vector<std::string> & rhs_keys, const std::vector<double> & rhs_values, const std::map<std::string, double> & full_ranges)
{
  bool rtn = false;
  std::map<std::string, double> lhs_map;
  std::map<std::string, double> rhs_map;
  if (industrial_utils::isSimilar(lhs_keys, rhs_keys))
  {
    if (industrial_robot_client::utils::toMap(lhs_keys, lhs_values, lhs_map) && industrial_robot_client::utils::toMap(rhs_keys, rhs_values, rhs_map))
    {
      rtn = isWithinRange(lhs_keys, lhs_map, rhs_map, full_ranges);
    }
  }
  else
  {
    ROS_ERROR_STREAM(__FUNCTION__ << "::Key vectors are not similar");
    rtn = false;
  }
  return rtn;
}

}  // namespace motoman_utils
}  // namespace industrial_robot_client

