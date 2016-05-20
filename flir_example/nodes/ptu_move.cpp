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
#include <sensor_msgs/JointState.h>

ros::Publisher cmdVelPub;

void jointMove(float* position, float* velocity)
{
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.velocity.resize(2);
  joint_state.name[0] = "ptu_pan";
  joint_state.position[0] = position[0];
  joint_state.velocity[0] = velocity[0];
  joint_state.name[1] = "ptu_tilt";
  joint_state.position[1] = position[0];
  joint_state.velocity[1] = velocity[1];
  cmdVelPub.publish(joint_state);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ptu_example_move");
  ros::NodeHandle node;
  cmdVelPub = node.advertise<sensor_msgs::JointState>("/cmd", 1);
  ros::Rate rate(10);

  ROS_INFO("ptu_example_move cpp start...");

  while (ros::ok())
  {
    float position1[2] = {1.6, 0.6};
    float velocity1[2] = {0.6, 0.6};
    jointMove(position1, velocity1);
    sleep(7);
    float position2[2] = {-0.8, 0.5};
    float velocity2[2] = {0.6, 0.4};
    jointMove(position2, velocity2);
    sleep(7);
  }

  ros::spin();

  return 0;
}
