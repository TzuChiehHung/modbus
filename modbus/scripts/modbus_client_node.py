#!/usr/bin/env python
########################################################################### 
# This software is graciously provided by HumaRobotics 
# under the Simplified BSD License on
# github: git@www.humarobotics.com:baxter_tasker
# HumaRobotics is a trademark of Generation Robots.
# www.humarobotics.com 

# Copyright (c) 2013, Generation Robots.
# All rights reserved.
# www.generationrobots.com
#   
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation 
#  and/or other materials provided with the distribution.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
# BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
# THE POSSIBILITY OF SUCH DAMAGE. 
# 
# The views and conclusions contained in the software and documentation are 
# those of the authors and should not be interpreted as representing official 
# policies, either expressed or implied, of the FreeBSD Project.

import rospy
from modbus.modbus_wrapper_client import ModbusWrapperClient

if __name__=="__main__":
    rospy.init_node("modbus_client")

    host = rospy.get_param('host', '192.168.255.1')
    port = rospy.get_param('port', 502)
    unit = rospy.get_param('unit', 0x01)

    sub_topic = rospy.get_param('sub_topic', 'modbus_wrapper/output')
    pub_topic = rospy.get_param('pub_topic', 'modbus_wrapper/input')

    address_read_start = int(rospy.get_param('address_read_start', 0))
    address_write_start = int(rospy.get_param('address_write_start', 0))
    num_registers = int(rospy.get_param("num_registers", 20))

    rospy.loginfo('Modbus server: {}:{}'.format(host, port))
    rospy.loginfo('Modbus id: {}'.format(str(unit)))
    rospy.loginfo('address_read_start: {}'.format(address_read_start))
    rospy.loginfo('address_write_start: {}'.format(address_write_start))
    rospy.loginfo('num_registers: {}'.format(num_registers))

    nh = ModbusWrapperClient(
        host, port=port, unit=unit, reset_registers=False,
        sub_topic=sub_topic, pub_topic=pub_topic
        )

    nh.setReadingRegisters(address_read_start, num_registers)
    nh.setWritingRegisters(address_write_start, num_registers)
    rospy.loginfo("Setup complete")

    # start listening to modbus and publish changes to the rostopic
    nh.startListening()
    rospy.loginfo("Listener started")

    rospy.spin()

    # modclient.stopListening()
