/*
 * For E46 with none response
 * Copyright (C) 2016 Yuanbo She (yuanboshe@aicrobo.com)
 *
 * flir_ptu_driver ROS package
 * Copyright (C) 2014 Mike Purvis (mpurvis@clearpathrobotics.com)
 *
 * PTU ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 * Author: Toby Collett (University of Auckland)
 * Date: 2003-02-10
 *
 * Player - One Hell of a Robot Server
 * Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                     gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <flir_ptu_driver/driver.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include <signal.h>
#include <string>

using namespace flir_ptu_driver;

PTU* ptu;
float goalPanPosition;
float goalTiltPosition;
float currentPanPosition;
float currentTiltPosition;
float panVel;
float tiltVel;
float intervelTime;

/** Callback for getting new Goal JointState */
void cmdCallback(const sensor_msgs::JointStateConstPtr msg)
{
  ROS_DEBUG("PTU command callback.");

  if (msg->position.size() != 2 || msg->velocity.size() != 2)
  {
    ROS_ERROR("JointState command to PTU has wrong number of elements.");
    return;
  }

  goalPanPosition = msg->position[0];
  goalTiltPosition = msg->position[1];
  panVel = msg->velocity[0];
  tiltVel = msg->velocity[1];
  ptu->setPosition(PTU_PAN, goalPanPosition);
  ptu->setPosition(PTU_TILT, goalTiltPosition);
  ptu->setSpeed(PTU_PAN, panVel);
  ptu->setSpeed(PTU_TILT, tiltVel);
}

void shutdown(int sig)
{
  ROS_INFO("aicrobo_ptu ended!");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aicrobo_ptu");
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");
  ros::Publisher jointPub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Subscriber jointSub = nh.subscribe("cmd", 1, cmdCallback);
  signal(SIGINT, shutdown);
  ROS_INFO("aicrobo_ptu start...");

  // Get params
  int hz = ph.param("hz", PTU_DEFAULT_HZ);
  std::string port = ph.param<std::string>("port", PTU_DEFAULT_PORT);
  int32_t baud = ph.param("baud", PTU_DEFAULT_BAUD);

  // Connect to PTU
  serial::Serial serial(port, baud, serial::Timeout::simpleTimeout(1000));
  try
  {
    ROS_INFO_STREAM("Attempting to connect to FLIR PTU on " << port);
//    serial.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port " << port);
  }

  ROS_INFO_STREAM("FLIR PTU serial port opened, now initializing.");
  ptu = new PTU(&serial);
  if (!ptu->initialize())
  {
    ROS_ERROR_STREAM("Could not initialize FLIR PTU on " << port);
    return 0;
  }
  ROS_INFO("FLIR PTU initialized.");

  // On while
  ros::Rate rate(hz);
  intervelTime = 1.0 / hz;
  while (ros::ok())
  {
    // Calculate position
    float intervelPan = intervelTime * panVel;
    float intervelTilt = intervelTime * tiltVel;
    double pan = 0;
    if (goalPanPosition > currentPanPosition)
    {
      pan = ((goalPanPosition - currentPanPosition) > intervelPan) ? currentPanPosition + intervelPan : goalPanPosition;
    }
    else
    {
      pan = ((currentPanPosition - goalPanPosition) > intervelPan) ? currentPanPosition - intervelPan : goalPanPosition;
    }
    double tilt = 0;
    if (goalTiltPosition > currentTiltPosition)
    {
      tilt = ((goalTiltPosition - currentTiltPosition) > intervelTilt) ? currentTiltPosition + intervelTilt : goalTiltPosition;
    }
    else
    {
      tilt = ((currentTiltPosition - goalTiltPosition) > intervelTilt) ? currentTiltPosition - intervelTilt : goalTiltPosition;
    }
    currentPanPosition = pan;
    currentTiltPosition = tilt;

    // Publish Position & Speed
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.name[0] = "ptu_pan";
    joint_state.position[0] = pan;
    joint_state.velocity[0] = panVel;
    joint_state.name[1] = "ptu_tilt";
    joint_state.position[1] = tilt;
    joint_state.velocity[1] = tiltVel;
    jointPub.publish(joint_state);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
