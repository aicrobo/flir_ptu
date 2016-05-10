/*
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

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <flir_ptu_driver/driver.h>
#include <serial/serial.h>
#include <ros/console.h>

#include <math.h>
#include <string>

using boost::lexical_cast;

namespace flir_ptu_driver
{

/** Templated wrapper function on lexical_cast to assist with extracting
 * values from serial response strings.
 */
template<typename T>
  T parseResponse(std::string responseBuffer)
  {
    std::string trimmed = responseBuffer.substr(1);
    boost::trim(trimmed);
    T parsed = lexical_cast<T>(trimmed);
    ROS_DEBUG_STREAM("Parsed response value: " << parsed);
    return parsed;
  }

bool PTU::initialized()
{
  return ser_ && ser_->isOpen();
}

bool PTU::initialize()
{
  ser_->write("ft ");  // terse feedback
  ser_->write("ed ");  // disable echo
  ser_->write("ci ");  // position mode
  ser_->read(20);

  // get pan tilt encoder res
  tr = 0.0002440;
  pr = 0.0008976;

  PMin = -3090;
  PMax = 3090;
  TMin = -3620;
  TMax = 2360;
  PSMin = 10;
  PSMax = 2000;
  TSMin = 10;
  TSMax = 2000;

  return initialized();
}

std::string PTU::sendCommand(std::string command)
{
  ser_->write(command);
  ROS_DEBUG_STREAM("TX: " << command);
  std::string buffer = ser_->readline(PTU_BUFFER_LEN);
  ROS_DEBUG_STREAM("RX: " << buffer);
  return buffer;
}

bool PTU::home()
{
  ROS_INFO("Sending command to reset PTU.");

  // Issue reset command
  ser_->flush();
  ser_->write(" r ");

  std::string actual_response, expected_response("!T!T!P!P*");

  // 30 seconds to receive full confirmation of reset action completed.
  for (int i = 0; i < 300; i++)
  {
    usleep(100000);

    if (ser_->available() >= expected_response.length())
    {
      ROS_INFO("PTU reset command response received.");
      ser_->read(actual_response, expected_response.length());
      return (actual_response == expected_response);
    }
  }

  ROS_WARN("PTU reset command response not received before timeout.");
  return false;
}

// set position in radians
bool PTU::setPosition(char type, float pos, bool block)
{
  if (!initialized())
    return false;

  // get raw encoder count to move
  int count = static_cast<int>(pos / getResolution(type));

  // Check limits
  if (count < (type == PTU_TILT ? TMin : PMin) || count > (type == PTU_TILT ? TMax : PMax))
  {
    ROS_ERROR("Pan Tilt Value out of Range: %c %f(%d) (%d-%d)\n", type, pos, count, (type == PTU_TILT ? TMin : PMin), (type == PTU_TILT ? TMax : PMax));
    return false;
  }

  std::string buffer = sendCommand(std::string() + type + "p" + lexical_cast<std::string>(count) + " ");

  if (block)
  {
    ROS_INFO("Sleep 1000");
    usleep(1000);
  }

  return true;
}

// set speed in radians/sec
bool PTU::setSpeed(char type, float pos)
{
  if (!initialized())
    return false;

  // get raw encoder speed to move
  int count = static_cast<int>(pos / getResolution(type));

  // Check limits
  if (abs(count) < (type == PTU_TILT ? TSMin : PSMin) || abs(count) > (type == PTU_TILT ? TSMax : PSMax))
  {
    ROS_ERROR("Pan Tilt Speed Value out of Range: %c %f(%d) (%d-%d)\n", type, pos, count, (type == PTU_TILT ? TSMin : PSMin), (type == PTU_TILT ? TSMax : PSMax));
    return false;
  }

  std::string buffer = sendCommand(std::string() + type + "s" + lexical_cast<std::string>(count) + " ");

  return true;
}

// set movement mode (position/velocity)
bool PTU::setMode(char type)
{
  if (!initialized())
    return false;

  std::string buffer = sendCommand(std::string("c") + type + " ");

  if (buffer.empty() || buffer[0] != '*')
  {
    ROS_ERROR("Error setting pan-tilt move mode");
    return false;
  }

  return true;
}

}  // namespace flir_ptu_driver
