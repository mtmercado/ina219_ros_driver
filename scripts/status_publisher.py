#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3

from ina219_driver import INA219Driver

MIN_THRESH = 11.2 #V
LOG_LEVEL = rospy.DEBUG

def publish_battery_status():
    rospy.init_node('battery_status_publisher', log_level=LOG_LEVEL)
    pub = rospy.Publisher('battery_status', Vector3, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    ina219 = INA219Driver()

    while not rospy.is_shutdown():
        voltage = ina219.get_voltage() #V
        current = ina219.get_current() #mA
        power = ina219.get_power() #V

        msg = Vector3()
        msg.x = voltage
        msg.y = current
        msg.z = power
        pub.publish(msg)

        if voltage <= MIN_THRESH:
            rospy.logwarn("LOW VOLTAGE WARNING: %.2f V" % voltage)
        else:
            rospy.logdebug("Battery Voltage: %.2f V | Current: %.2f mA | Power: %.2f W" % (voltage, current, power))

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_battery_status()
    except rospy.ROSInterruptException:
        pass
