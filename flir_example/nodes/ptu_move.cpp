/*!
 * Author: Qiyang Gu guqiyang@aicrobo.com
 * Group: AICRobo http://aicrobo.github.io
 *
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, AICRobo.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>

ros::Publisher cmdVelPub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ptu_example_move");
  std::string topic = "/cmd";
  ros::NodeHandle node;
  cmdVelPub = node.advertise<sensor_msgs::JointState>(topic, 1);

  ROS_INFO("ptu_example_move cpp start...");

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.velocity.resize(2);
  joint_state.name[0] = "ptu_pan";
  joint_state.position[0] = 1.6;
  joint_state.velocity[0] = 0.6;
  joint_state.name[1] = "ptu_tilt";
  joint_state.position[1] = 0.6;
  joint_state.velocity[1] = 0.6;
  cmdVelPub.publish(joint_state);

  return 0;
}
