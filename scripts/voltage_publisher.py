#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

from ina219_driver import INA219Driver

MIN_THRESH = 11.2 #V

def publish_battery_voltage():
    rospy.init_node('battery_voltage_publisher')
    pub = rospy.Publisher('battery_voltage', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    ina219 = INA219Driver()

    while not rospy.is_shutdown():
        # Simulate battery voltage measurement (replace this with your actual voltage value)
        voltage = ina219.get_voltage()

        # Create a Float32 message and publish it
        msg = Float32()
        msg.data = voltage
        pub.publish(msg)

        if voltage <= MIN_THRESH:
            rospy.logwarn("LOW VOLTAGE WARNING: %.2f V" % voltage)
        else:
            rospy.logdebug("Battery Voltage: %.2f V" % voltage)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_battery_voltage()
    except rospy.ROSInterruptException:
        pass
