#!/usr/bin/env python
# coding=utf-8
#
# Author: QiYang Gu guqiyang@aicrobo.com
# Group: AICRobo http://aicrobo.github.io
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, AICRobo.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

if __name__ == '__main__':
    rospy.init_node('ptu_cmd_angles')
    pub = rospy.Publisher("/cmd", JointState, queue_size=1)
    
    
    
    js = JointState()
    js.name = [ "ptu_pan", "ptu_tilt" ]
    js.velocity = [ 0.6, 0.6 ]
    js.position = [ -0.6, 0.6 ]
    
    #加入延时的作用是等待ros的一些准备过程，否则就是执行不成功
    
    rospy.sleep(1) 
    pub.publish(js)
        
